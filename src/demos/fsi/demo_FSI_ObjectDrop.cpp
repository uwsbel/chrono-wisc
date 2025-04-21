// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <fstream>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"
#include "demo_FSI_utils.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Container dimensions
//ChVector3d csize(1.6, 1.6, 1.6);
//// Dimensions of fluid domain
//ChVector3d fsize(1.6, 1.6, 1.2);

//Container dimensions
ChVector3d csize(2.2, 2.2, 1.4);
// Dimensions of fluid domain
ChVector3d fsize(2.2, 2.2, 0.99);

 //Container dimensions
 //ChVector3d csize(1.1, 1.1, 1.4);
//ChVector3d csize(0.8, 0.8, 1.4);

 // Dimensions of fluid domain
 //ChVector3d fsize(1.1, 1.1, 0.99);

//ChVector3d fsize(0.8, 0.8, 1.19);


// Object type
enum class ObjectShape { SPHERE_PRIMITIVE, CYLINDER_PRIMITIVE, MESH };
ObjectShape object_shape = ObjectShape::MESH;

// Mesh specification (for object_shape = ObjectShape::MESH) 
std::string mesh_obj_filename = GetChronoDataFile("models/semicapsule.obj");
double mesh_scale = 1;
double mesh_bottom_offset = 0.14;  // is there a better way to determine this? 

double initial_height = 1.0;
double density = 600;
// Visibility flags
bool show_rigid = true;
bool show_rigid_bce = false;
bool show_boundary_bce = true;
bool show_particles_sph = true;

bool render = false;
int render_fps = 20;

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChFsiVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].x < 0 || pos[n].y < 0; }
};
#endif

// -----------------------------------------------------------------------------

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& viscosity_type,
                     std::string& kernel_type,
                     double& initial_spacing,
                     double& d0,
                     double& step_size,
                     double& fluid_density,
                     double& v_max,
                     std::string& run_tag) {
    ChCLI cli(argv[0], "FSI object drop demo");

    // Simulation parameters
    cli.AddOption<double>("Simulation", "step_size", "Integration step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "initial_spacing", "Initial spacing between SPH particles", std::to_string(initial_spacing));
    cli.AddOption<double>("Simulation", "d0", "SPH kernel support radius ratio h = d0 * initial_spacing", std::to_string(d0));
    
    // Object parameters
    cli.AddOption<double>("Fluid", "density", "Fluid density", std::to_string(fluid_density));
    cli.AddOption<double>("Fluid", "v_max", "Maxiemum fluid velocity", std::to_string(v_max));

    // Physics options
    cli.AddOption<std::string>("Physics", "viscosity_type", "Viscosity type (laminar_arman/laminar_dual)", viscosity_type);

    cli.AddOption<std::string>("Physics", "kernel_type", "Kernel type (wendland/cubic_spline)", kernel_type);

    cli.AddOption<std::string>("Output", "run_tag", "Run tag for output directory", run_tag);

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    // Get all parameter values
    step_size = cli.GetAsType<double>("step_size");
    initial_spacing = cli.GetAsType<double>("initial_spacing");
    fluid_density = cli.GetAsType<double>("density");
    v_max = cli.GetAsType<double>("v_max");
    d0 = cli.GetAsType<double>("d0");
    viscosity_type = cli.GetAsType<std::string>("viscosity_type");
    kernel_type = cli.GetAsType<std::string>("kernel_type");    
    run_tag = cli.GetAsType<std::string>("run_tag");

    return true;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double t_end = 5.0;             // simulation duration
    bool output = true;
    double output_fps = 20;
    const ChVector3d gravity(0, 0, -9.8);

    // given initial height, compute impact velocity

    double impact_velocity = -sqrt(2 * std::abs(gravity.z()) * (initial_height - mesh_bottom_offset));
    bool use_impact_velocity = false;
    //impact_velocity = 0;  // starts the object above water surface, but set impact velocity to 0


    // Default parameter values
    double initial_spacing = 0.015;  // initial spacing between SPH particles
    double step_size = 2e-5;      // integration step size
    double d0 = 1.5;
    std::string viscosity_type = "laminar_dual";
    double fluid_density = 998.5;
    double v_max = 8.0;
    std::string kernel_type = "wendland";
    std::string run_tag = "arman";

     //Parse command line arguments
    if (!GetProblemSpecs(argc, argv, viscosity_type, kernel_type, initial_spacing, d0, step_size, fluid_density, v_max, run_tag)) {
        return 1;
    }

    // Create the Chrono system and associated collision system
    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiProblemCartesian fsi(sysMBS, initial_spacing);
    fsi.SetVerbose(true);
    ChFsiSystemSPH& sysFSI = fsi.GetSystemFSI();

    // Set gravitational acceleration
    fsi.SetGravitationalAcceleration(gravity);

    // Set integration step size
    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

    // Set CFD fluid properties
    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = fluid_density;
    double kinematic_viscosity = 1e-6;
    fluid_props.viscosity = kinematic_viscosity * fluid_props.density;

    fsi.SetCfdSPH(fluid_props);
    
    // Set SPH solution parameters
    int num_bce_layers = 5;   // this seems quite large... but let's see if it makes a difference. 
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.num_bce_layers = num_bce_layers;

    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = d0;
    sph_params.max_velocity = v_max;

    // now we set kernel type and viscosity type
    if (kernel_type == "wendland") {
        sph_params.kernel_type = KernelType::WENDLAND;
    } else if (kernel_type == "cubic_spline") {
        sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    }
    
    if (viscosity_type == "laminar_arman") {
        sph_params.viscosity_type = ViscosityType::LAMINAR;
    } else if (viscosity_type == "laminar_dual") {
        sph_params.viscosity_type = ViscosityType::LAMINAR;
    } 

    sph_params.shifting_method = ShiftingMethod::DIFFUSION;
    sph_params.shifting_diffusion_A = 1.;
    sph_params.shifting_diffusion_AFSM = 3.;
    sph_params.shifting_diffusion_AFST = 2.;
    //sph_params.shifting_method = ShiftingMethod::XSPH;
    //sph_params.shifting_xsph_eps = 0.5;
    sph_params.eos_type = EosType::TAIT;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.num_proximity_search_steps = 1;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;

    // Set boundary and viscosity types
    sph_params.boundary_type = BoundaryType::ADAMI;
    fsi.SetSPHParameters(sph_params);

    // create ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->SetMass(10.f);
    sysMBS.AddBody(ground);
   

    // Create the rigid body
    double bottom_offset = 0;
    double mass = 0;
    ChMatrix33d inertia;
    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(ChContactMaterialData());
    switch (object_shape) {
        case ObjectShape::SPHERE_PRIMITIVE: {
            double radius = 0.12;
            bottom_offset = radius;
            ChSphere sphere(radius);
            mass = density * sphere.GetVolume();
            inertia = mass * sphere.GetGyration();
            geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(VNULL, sphere, 0));
            break;
        }
        case ObjectShape::CYLINDER_PRIMITIVE: {
            double radius = 0.12;
            double length = 0.20;
            bottom_offset = radius;
            ChCylinder cylinder(radius, length);
            mass = density * cylinder.GetVolume();
            inertia = mass * cylinder.GetGyration();
            geometry.coll_cylinders.push_back(
                utils::ChBodyGeometry::CylinderShape(VNULL, Q_ROTATE_Z_TO_X, cylinder, 0));
            break;
        }
        case ObjectShape::MESH: {
            auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_obj_filename, true, true);
            ChVector3d com;
            trimesh->ComputeMassProperties(true, mass, com, inertia, mesh_scale);
            mass *= density;
            std::cout << "mesh mass " << mass << std::endl;
            inertia *= density;
            bottom_offset = mesh_bottom_offset;
            geometry.coll_meshes.push_back(
                utils::ChBodyGeometry::TrimeshShape(VNULL, mesh_obj_filename, VNULL, mesh_scale, 0.01, 0));
            break;
        }
    }
    
    auto body = chrono_types::make_shared<ChBody>();
    body->SetName("object");

    if (use_impact_velocity) {
        body->SetPos(ChVector3d(0, 0, fsize.z() + 2 * initial_spacing + mesh_bottom_offset));
        body->SetPosDt(ChVector3d(0, 0, impact_velocity));

    } else {
        body->SetPos(ChVector3d(0, 0, initial_height + fsize.z()));
        body->SetPosDt(ChVector3d(0, 0, 0));
    }

    // body->SetPos(ChVector3d(0, 0, initial_height + fsize.z()));

    std::cout << "capsule height " << body->GetPos().z() << std::endl;
    body->SetRot(QUNIT);
    mass = 9.57;
    body->SetMass(mass);
    body->SetInertia(inertia);
    body->SetFixed(false);
    body->EnableCollision(false);
    sysMBS.AddBody(body);


    // add translational joint between ground and object
    auto joint = chrono_types::make_shared<ChLinkLockPrismatic>();
    joint->Initialize(ground, body, ChFramed(ChVector3d(0, 0, 0), QUNIT));
    sysMBS.AddLink(joint);

    if (show_rigid)
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

    // Add as an FSI body (create BCE markers on a grid)
    fsi.AddRigidBody(body, geometry, true, true);
    // Enable depth-based initial pressure for SPH particles
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(fsize.z()));


    // manually add side walls as BCE markers
    // create one chbody for all four side walls 
    auto side_walls = chrono_types::make_shared<ChBody>();
    side_walls->SetFixed(true);
    side_walls->SetPos(ChVector3d(0, 0, 0));

    // create side wall geometry as box
    auto geometry_side_walls = utils::ChBodyGeometry();
    double wall_thickness = (num_bce_layers - 1) * initial_spacing;  // thickness of the wall is the same as the thickness of the BCE layer
    // add box shape, left side, position is (-csize.x()/2 - num_bce_layers * initial_spacing/2.0, 0, csize.z()/2)
    geometry_side_walls.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(
        ChVector3d(-csize.x() / 2 - wall_thickness / 2.0 - initial_spacing, 0, csize.z() / 2), 
        Q_ROTATE_Z_TO_X,
        ChVector3d( csize.z() + 2 * num_bce_layers * initial_spacing, csize.x() + initial_spacing, wall_thickness), 0));

    // other side 
    geometry_side_walls.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(
        ChVector3d(csize.x() / 2 + wall_thickness / 2.0 + initial_spacing + 0.005, 0, csize.z() / 2), 
        -Q_ROTATE_Z_TO_X,
        ChVector3d(csize.z() + 2 * num_bce_layers * initial_spacing, csize.x() + initial_spacing, wall_thickness), 0));

    // walls in y location 
    geometry_side_walls.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(
        ChVector3d(0, -csize.y() / 2 - wall_thickness / 2.0 - initial_spacing, csize.z() / 2), 
        Q_ROTATE_Z_TO_Y,
        ChVector3d(csize.x() + 2 * num_bce_layers * initial_spacing, csize.z() + 2 * num_bce_layers * initial_spacing, wall_thickness), 0));

    // last wall ... 
    geometry_side_walls.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(
        ChVector3d(0, csize.y() / 2 + wall_thickness / 2.0 + initial_spacing + 0.005, csize.z() / 2), -Q_ROTATE_Z_TO_Y,
        ChVector3d(csize.x() + 2 * num_bce_layers * initial_spacing, csize.z() + 2 * num_bce_layers * initial_spacing,
                   wall_thickness),
        0));

     //geometry_side_walls.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(
     //    ChVector3d(0, csize.y() / 2 + wall_thickness / 2.0 + initial_spacing, csize.z() / 2),
     //    -Q_ROTATE_Z_TO_Y, ChVector3d(csize.x() + 2 * num_bce_layers * initial_spacing - 0.005, csize.z() + 2 * num_bce_layers
     //    * initial_spacing,
     //               wall_thickness),
     //    0));


    // attach geometry to side walls
    fsi.AddRigidBody(side_walls, geometry_side_walls, false, true);


    // Create SPH material and boundaries
    fsi.Construct(fsize,                          // length x width x depth
                  ChVector3d(0, 0, 0),            // position of bottom origin
                  BoxSide::Z_NEG  // all boundaries except top
    );

    // Computational domain must always contain all BCE and Rigid markers - if these leave computational domain,
    // the simulation will crash
    ChVector3d cMin(-csize.x() / 2 - num_bce_layers * initial_spacing,
                    -csize.y() / 2 - num_bce_layers * initial_spacing, -0.1);
    ChVector3d cMax(csize.x() / 2 + num_bce_layers * initial_spacing, csize.y() / 2 + num_bce_layers * initial_spacing,
                    csize.z() + initial_height + bottom_offset);
    fsi.SetComputationalDomain(ChAABB(cMin, cMax), PeriodicSide::NONE);

    // Initialize FSI problem
    fsi.Initialize();

    // Create unique output directory based on parameters
    //std::string out_dir = CreateFsiOutputName("ObjectDrop",
    //                                            initial_spacing,
    //                                            d0,
    //                                            step_size,
    //                                            fluid_density,
    //                                            v_max,
    //                                            viscosity_type,
    //                                            run_tag) + "/";

    std::string out_dir = "ObjectDrop_" + run_tag + "_original_laminar/";

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
            cerr << "Error creating directory " << out_dir + "/particles" << endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
            cerr << "Error creating directory " << out_dir + "/fsi" << endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
            cerr << "Error creating directory " << out_dir + "/vtk" << endl;
            return 1;
        }
    }
    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        ////auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 1.0);
        ////auto col_callback = chrono_types::make_shared<ParticleDensityColorCallback>(995, 1005);
        auto col_callback = chrono_types::make_shared<ParticlePressureColorCallback>(
            ChColor(1, 0, 0), ChColor(0.14f, 0.44f, 0.7f), -1000, 12000);

        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Object Drop");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(2.5 * fsize.x(), 2.5 * fsize.y(), 1.5 * fsize.z()),
                          ChVector3d(0, 0, 0.5 * fsize.z()));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    std::string out_file = out_dir + "/results.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        auto body_height = body->GetPos().z();
        ofile << time << "\t" << body_height << "\n";

        if (output && time >= out_frame / output_fps) {
                cout << " -- Output frame " << out_frame << " at t = " << time << endl;
            fsi.SaveOutputData(time, out_dir + "/particles", out_dir + "/fsi");

            out_frame++;
        }

        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }

        // Call the FSI solver
        fsi.DoStepDynamics(step_size);

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    ofile.close();

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/height.gpl");
    gplot.SetGrid();
    std::string speed_title = "Object height";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("height (m)");
    gplot.Plot(out_file, 1, 2, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif

    return 0;
}