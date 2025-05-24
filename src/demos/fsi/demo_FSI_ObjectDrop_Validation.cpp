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
//Container dimensions
//ChVector3d csize(1.0, 1.0, 1.2);
//ChVector3d fsize(1.0, 1.0, 0.8);
ChVector3d csize(2.1, 2.1, 1.4);
ChVector3d fsize(2.1, 2.1, 0.8);


// Object type
enum class ObjectShape { SPHERE_PRIMITIVE, SPHERE_BCEDUAL, MESH };
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

bool GetProblemSpecs(int argc,
    char** argv,
    std::string& case_name,
    double& initial_spacing,
    double& d0,
    double& step_size,
    double& v_max,
    std::string& integrator,
    double& offset_ratio,
    std::string& run_tag) {
    ChCLI cli(argv[0], "FSI object drop demo");

    cli.AddOption<std::string>("Physics", "case_name", "case_name(sphere_primitive/sphere_bcedual/mesh)", case_name);

    // Simulation parameters
    cli.AddOption<double>("Simulation", "initial_spacing", "Initial spacing between SPH particles", std::to_string(initial_spacing));
    cli.AddOption<double>("Simulation", "d0", "SPH kernel support radius ratio h = d0 * initial_spacing", std::to_string(d0));
    cli.AddOption<double>("Simulation", "step_size", "Integration step size", std::to_string(step_size));

    // Object parameters
    cli.AddOption<double>("Fluid", "v_max", "Maxiemum fluid velocity", std::to_string(v_max));
    cli.AddOption<double>("Fluid", "offset_ratio", "Offset ratio for sphere 0.1, 0.3 and 0.5", std::to_string(offset_ratio));

    // Physics options
    cli.AddOption<std::string>("Physics", "integrator", "Integrator type (rk2/verlet/symplectic)", integrator);
    cli.AddOption<std::string>("Output", "run_tag", "Run tag for output directory", run_tag);

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    // Get all parameter values
    case_name = cli.GetAsType<std::string>("case_name");
    step_size = cli.GetAsType<double>("step_size");
    initial_spacing = cli.GetAsType<double>("initial_spacing");
    v_max = cli.GetAsType<double>("v_max");
    d0 = cli.GetAsType<double>("d0");
    integrator = cli.GetAsType<std::string>("integrator");
    offset_ratio = cli.GetAsType<double>("offset_ratio");
    run_tag = cli.GetAsType<std::string>("run_tag");

    return true;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double t_end = 5;             // simulation duration
    bool output = true;
    double output_fps = 5;
    const ChVector3d gravity(0, 0, -9.82);

    // Default parameter values
    
    double initial_spacing = 0.015;  // initial spacing between SPH particles
    double step_size = 4e-5;      // integration step size
    double d0 = 1.5;
    double fluid_density = 998.2;
    double v_max = 4.0;
    double offset = 0.0;
    double offset_ratio = 0.1;  // offset ratio for sphere 0.1, 0.3 and 0.5
    std::string integrator = "rk2";
    std::string run_tag = "0." + std::to_string(int(offset_ratio*10)) + "D";
    std::string case_name = "sphere_primitive";

    //Parse command line arguments
    if (!GetProblemSpecs(argc, argv, case_name, initial_spacing, d0, step_size, v_max, integrator, offset_ratio, run_tag)) {
        return 1;
    }


    if (case_name == "sphere_primitive") {
		object_shape = ObjectShape::SPHERE_PRIMITIVE;
	}
    else if (case_name == "sphere_bcedual") {
		object_shape = ObjectShape::SPHERE_BCEDUAL;
	}
    else if (case_name == "mesh") {
		object_shape = ObjectShape::MESH;
	}
	else {
		std::cout << "Invalid case name. Use sphere_primitive, sphere_bcedual or mesh." << std::endl;
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
    int num_bce_layers = 3;   // this seems quite large... but let's see if it makes a difference. 
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.num_bce_layers = num_bce_layers;

    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = d0;
    sph_params.max_velocity = v_max;
    sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    sph_params.viscosity_method = ViscosityMethod::LAMINAR;
    sph_params.shifting_method = ShiftingMethod::DIFFUSION;
    sph_params.shifting_diffusion_A = 1.;
    sph_params.shifting_diffusion_AFSM = 3.;
    sph_params.shifting_diffusion_AFST = 2.;
    sph_params.eos_type = EosType::TAIT;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.num_proximity_search_steps = 1;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;

    if (integrator == "rk2") {
        sph_params.integration_scheme = IntegrationScheme::RK2;
    } else if (integrator == "verlet") {
		sph_params.integration_scheme = IntegrationScheme::VERLET;
    }
    else if (integrator == "symplectic") {
		sph_params.integration_scheme = IntegrationScheme::SYMPLECTIC;
    }


    // Set boundary and viscosity types
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    fsi.SetSPHParameters(sph_params);

    // create ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->SetMass(10.f);
    sysMBS.AddBody(ground);

    // Set surface reconstruction parameters
    ChFsiFluidSystemSPH::SplashsurfParameters splashsurf_params;
    splashsurf_params.smoothing_length = 2.0;
    splashsurf_params.cube_size = 0.3;
    splashsurf_params.surface_threshold = 0.6;

    fsi.SetSplashsurfParameters(splashsurf_params);

    // Create the rigid body
    double bottom_offset = 0;
    double mass = 0;
    ChVector3d initial_position(0, 0, 0);
    ChMatrix33d inertia;
    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(ChContactMaterialData());
    switch (object_shape) {
    case ObjectShape::SPHERE_PRIMITIVE: {
        double radius = 0.15;
        density = 500;
        bottom_offset = radius;
        ChSphere sphere(radius);
        mass = 7.056;  // from paper
        inertia = mass * sphere.GetGyration();
        geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(VNULL, sphere, 0));
        mesh_bottom_offset = offset_ratio * 2. * radius;
        initial_position.z() = fsize.z() + mesh_bottom_offset;   // having extra initial_spacing is fine to ensure object and water are not touching
        break;
        }

    case ObjectShape::SPHERE_BCEDUAL: {
        double radius = 0.15;
        density = 500;
        bottom_offset = radius;
        ChSphere sphere(radius - initial_spacing/2.0);
        mass = 7.056;  // from paper
        inertia = mass * sphere.GetGyration();
        geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(VNULL, sphere, 0));
        mesh_bottom_offset = offset_ratio * 2. * radius;
        initial_position.z() = fsize.z() + initial_spacing/2.0 + mesh_bottom_offset;
        break;
        }

    case ObjectShape::MESH: {
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_obj_filename, true, true);
        ChVector3d com;
        trimesh->ComputeMassProperties(true, mass, com, inertia, mesh_scale);
        mass = 9.75;
        inertia *= density;
        bottom_offset = mesh_bottom_offset;
        geometry.coll_meshes.push_back(
            utils::ChBodyGeometry::TrimeshShape(VNULL, mesh_obj_filename, VNULL, mesh_scale, 0.01, 0));
        initial_position.z() = fsize.z() + initial_height;
        break;
        }
    }

    auto body = chrono_types::make_shared<ChBody>();
    body->SetName("object");

    body->SetPos(initial_position);
    body->SetPosDt(ChVector3d(0, 0, 0));

    std::cout << "object height " << body->GetPos().z() << std::endl;
    std::cout << "object mass " << mass << std::endl;
    body->SetRot(QUNIT);
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

    // Create SPH material and boundaries
    fsi.Construct(fsize,                // length x width x depth
                  ChVector3d(0, 0, 0),  // position of bottom origin
                  BoxSide::NONE);

    fsi.AddBoxContainer(csize,                // length x width x height
                        ChVector3d(0, 0, 0),  // reference location
                        BoxSide::ALL & ~BoxSide::Z_POS  // creater only bottom boundary
    );



    // Computational domain must always contain all BCE and Rigid markers - if these leave computational domain,
    // the simulation will crash
    ChVector3d cMin(-csize.x() / 2 - num_bce_layers * initial_spacing,
        -csize.y() / 2 - num_bce_layers * initial_spacing, -0.1);
    ChVector3d cMax(csize.x() / 2 + num_bce_layers * initial_spacing, csize.y() / 2 + num_bce_layers * initial_spacing,
        csize.z() + initial_height + bottom_offset);
    fsi.SetComputationalDomain(ChAABB(cMin, cMax), BC_NONE);

    // Initialize FSI problem
    fsi.Initialize();

    std::string out_dir = case_name + "_" + integrator + "_" + "H0" + std::to_string(int(offset_ratio * 10)) + "D"+ run_tag;

    std::cout << "Output directory: " << out_dir << std::endl;

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

    if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
        cerr << "Error creating directory " << out_dir + "/snapshots" << endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/meshes"))) {
        cerr << "Error creating directory " << out_dir + "/meshes" << endl;
        return 1;
    }

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        ////auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 1.0);
        ////auto col_callback = chrono_types::make_shared<ParticleDensityColorCallback>(995, 1005);
        auto col_callback = chrono_types::make_shared<ParticlePressureColorCallback>(-1000, 12000, true);

        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::RED_BLUE);
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

            if (render_frame >= 70 && render_frame < 80) {
                std::ostringstream meshname;
                meshname << "mesh_" << std::setw(5) << std::setfill('0') << render_frame + 1;
                fsi.WriteReconstructedSurface(out_dir + "/meshes", meshname.str(), true);
            }

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
