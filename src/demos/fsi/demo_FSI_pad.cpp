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
// Author: Huzaifa Mustafa Unjhawala
// =============================================================================
// Normal Bevameter Validation Problem involving immersing a bevameter at a specified velocity
// and measureing the force on the bevameter tip
// Comparing against GRC-1 paper - https://www.sciencedirect.com/science/article/pii/S0022489810000388
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <fstream>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono_fsi/sph/ChFsiSystemSPH.h"


#ifdef CHRONO_VSG
#include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

#ifdef CHRONO_VSG
// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;
class MarkerPositionVisibilityCallback : public ChFsiVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}

    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};
#endif

double nu_poisson = 0.3;

// Plate material - Steel
struct solid_material {
    double youngs_modulus = 193e9;
    double friction_coefficient = 0.7;
    double density = 7.8e3;
    double restitution = 0.05;
    double adhesion = 0;
};

// -----------------------------------------------------------------------------

struct SimParams {
    // Simulation parameters
    int ps_freq;
    double initial_spacing;
    double d0_multiplier;
    double time_step;
    std::string boundary_type;
    std::string viscosity_type;
    std::string kernel_type;

    // Physics parameters
    double artificial_viscosity;
    double max_pressure;
    double plate_diameter;

    // Output/rendering parameters
    bool verbose;
    bool output;
    double output_fps;
    bool snapshots;
    bool render;
    double render_fps;
    bool write_marker_files;
    // Need to play around with these too
    double mu_s;
    double mu_2;
    double cohesion;
    double density;
    double y_modulus;
};
void SimulateMaterial(const SimParams& params);
// Function to handle CLI arguments
bool GetProblemSpecs(int argc, char** argv, SimParams& params) {
    ChCLI cli(argv[0], "FSI Normal Bevameter Validation Problem");

    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(params.ps_freq));
    cli.AddOption<double>("Simulation", "initial_spacing", "Initial spacing", std::to_string(params.initial_spacing));
    cli.AddOption<double>("Simulation", "d0_multiplier", "D0 multiplier", std::to_string(params.d0_multiplier));
    cli.AddOption<std::string>("Simulation", "boundary_type", "Boundary condition type (holmes/adami)",
        params.boundary_type);
    cli.AddOption<std::string>("Simulation", "viscosity_type",
        "Viscosity type (artificial_unilateral/artificial_bilateral)", params.viscosity_type);
    cli.AddOption<std::string>("Simulation", "kernel_type", "Kernel type (cubic/wendland)", params.kernel_type);
    cli.AddOption<double>("Simulation", "time_step", "Time step", std::to_string(params.time_step));

    cli.AddOption<double>("Physics", "artificial_viscosity", "Artificial viscosity",
        std::to_string(params.artificial_viscosity));
    cli.AddOption<double>("Physics", "max_pressure", "Max pressure", std::to_string(params.max_pressure));
    cli.AddOption<double>("Physics", "plate_diameter", "Plate diameter", std::to_string(params.plate_diameter));
    cli.AddOption<double>("Physics", "mu_s", "Friction coefficient", std::to_string(params.mu_s));
    cli.AddOption<double>("Physics", "mu_2", "Friction coefficient", std::to_string(params.mu_2));
    cli.AddOption<double>("Physics", "cohesion", "Cohesion", std::to_string(params.cohesion));
    cli.AddOption<double>("Physics", "density", "Density", std::to_string(params.density));
    cli.AddOption<double>("Physics", "y_modulus", "Young's modulus", std::to_string(params.y_modulus));
    if (!cli.Parse(argc, argv))
        return false;

    params.ps_freq = cli.GetAsType<int>("ps_freq");
    params.initial_spacing = cli.GetAsType<double>("initial_spacing");
    params.d0_multiplier = cli.GetAsType<double>("d0_multiplier");
    params.time_step = cli.GetAsType<double>("time_step");
    params.boundary_type = cli.GetAsType<std::string>("boundary_type");
    params.viscosity_type = cli.GetAsType<std::string>("viscosity_type");
    params.kernel_type = cli.GetAsType<std::string>("kernel_type");
    params.artificial_viscosity = cli.GetAsType<double>("artificial_viscosity");
    params.max_pressure = cli.GetAsType<double>("max_pressure");
    params.plate_diameter = cli.GetAsType<double>("plate_diameter");
    params.mu_s = cli.GetAsType<double>("mu_s");
    params.mu_2 = cli.GetAsType<double>("mu_2");
    params.cohesion = cli.GetAsType<double>("cohesion");
    params.density = cli.GetAsType<double>("density");
    params.y_modulus = cli.GetAsType<double>("y_modulus");
    return true;
}

int main(int argc, char* argv[]) {
    SimParams params = {/*ps_freq*/ 1,
        /*initial_spacing*/ 0.005,
        /*d0_multiplier*/ 1.3,
        /*time_step*/ 2e-4,
        /*boundary_type*/ "adami",
        /*viscosity_type*/ "artificial_bilateral",
        /*kernel_type*/ "wendland",
        /*artificial_viscosity*/ 0.5,
        /*max_pressure*/ 20 * 1000,  // 30 kPa
        /*plate_diameter*/ 0.60,     // 19 cm
        /*verbose*/ true,
        /*output*/ true,
        /*output_fps*/ 2,
        /*snapshots*/ true,
        /*render*/ false,
        /*render_fps*/ 400,
        /*write_marker_files*/ true,
        /*mu_s*/ 0.6593,
        /*mu_2*/ 0.6593,
        /*cohesions*/ 0,
        /*densities*/ 1670,
        /*y_modulus*/ 1e6 };

    if (!GetProblemSpecs(argc, argv, params)) {
        return 1;
    }

    std::cout << "Problem Specs:" << std::endl;

    std::cout << "ps_freq: " << params.ps_freq << std::endl;
    std::cout << "initial_spacing: " << params.initial_spacing << std::endl;
    std::cout << "d0_multiplier: " << params.d0_multiplier << std::endl;
    std::cout << "time_step: " << params.time_step << std::endl;
    std::cout << "boundary_type: " << params.boundary_type << std::endl;
    std::cout << "viscosity_type: " << params.viscosity_type << std::endl;
    std::cout << "kernel_type: " << params.kernel_type << std::endl;
    std::cout << "artificial_viscosity: " << params.artificial_viscosity << std::endl;
    std::cout << "mu_s: " << params.mu_s << std::endl;
    std::cout << "mu_2: " << params.mu_2 << std::endl;
    std::cout << "cohesion: " << params.cohesion << std::endl;
    std::cout << "density: " << params.density << std::endl;
    std::cout << "y_modulus: " << params.y_modulus << std::endl;
    std::cout << "write output: " << params.output << std::endl;
    std::cout << "write marker files: " << params.write_marker_files << std::endl;

    int num_materials = 1;
    for (int i = 0; i < num_materials; i++) {
        SimulateMaterial(params);
    }
}

void SimulateMaterial(const SimParams& params) {
    double t_end = 30;

    double time_preload = 5; 

    double container_diameter = params.plate_diameter * 1.1;  // Plate is 20 cm in diameter
    double container_height = 0.120;                          // 2.4 cm since experimentally the plate reaches 0.6 cm
    double cyl_length = container_height;                     // To prevent effect of sand falling on top of the plate

    // Create a physics system
    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -1.62);
    sysFSI.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);

    sysFSI.SetStepSizeCFD(params.time_step);
    sysFSI.SetStepsizeMBD(params.time_step);

    // -------------------------------------------------------------------------
    // Material and SPH parameters
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    ChFsiFluidSystemSPH::SPHParameters sph_params;

    mat_props.density = params.density;
    mat_props.Young_modulus = params.y_modulus;
    mat_props.Poisson_ratio = nu_poisson;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = params.mu_s;
    mat_props.mu_fric_2 = params.mu_2;
    mat_props.average_diam = 0.002;
    mat_props.cohesion_coeff = params.cohesion;

    sysSPH.SetElasticSPH(mat_props);

    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = params.initial_spacing;
    sph_params.d0_multiplier = params.d0_multiplier;
    sph_params.artificial_viscosity = params.artificial_viscosity;
    sph_params.shifting_xsph_eps = 0.0;
    sph_params.shifting_method = ShiftingMethod::PPST;
    // sph_params.shifting_coefficient = 0.0;

    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.kernel_threshold = 0.8;


    sph_params.num_proximity_search_steps = params.ps_freq;

    if (params.kernel_type == "cubic") {
        sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    } else if (params.kernel_type == "wendland") {
        sph_params.kernel_type = KernelType::WENDLAND;
    }
    if (params.boundary_type == "holmes") {
        sph_params.boundary_type = BoundaryType::HOLMES;
    } else {
        sph_params.boundary_type = BoundaryType::ADAMI;
    }

    // Set viscosity type
    if (params.viscosity_type == "artificial_bilateral") {
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    } else {
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;
    }

    sysSPH.SetSPHParameters(sph_params);
    // -------------------------------------------------------------------------

    // ==============================
    // Create container and granular material
    // ==============================
    // Create a container
    // sand clearance
    double clearance = 0.2 * container_height;
    double bxDim = container_diameter;
    double byDim = container_diameter;
    double fzDim = container_height;
    double bzDim = fzDim + clearance;

    // Set the periodic boundary condition
    ChVector3d cMin(-bxDim / 2 * 1.2, -byDim / 2 * 1.2, -bzDim * 1.2);
    ChVector3d cMax(bxDim / 2 * 1.2, byDim / 2 * 1.2, (bzDim + std::max(bzDim * 1.2, cyl_length)));
    sysSPH.SetComputationalBoundaries(cMin, cMax, PeriodicSide::NONE);

    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    sysMBS.AddBody(box);

    // Create granular material
    // Create SPH particle locations using a regular grid sampler
    chrono::utils::ChGridSampler<> sampler(params.initial_spacing);
    ChVector3d boxCenter(0, 0, fzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2 - params.initial_spacing, byDim / 2 - params.initial_spacing,
                          fzDim / 2 - params.initial_spacing);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
    for (const auto& p : points) {
        double pre_ini = sysSPH.GetDensity() * gz * (-p.z() + fzDim);
        double rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed());
        sysSPH.AddSPHParticle(p, rho_ini, pre_ini, sysSPH.GetViscosity(), ChVector3d(0));
    }

    solid_material solid_mat;
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(solid_mat.youngs_modulus);
    cmaterial->SetFriction(solid_mat.friction_coefficient);
    cmaterial->SetRestitution(solid_mat.restitution);
    cmaterial->SetAdhesion(solid_mat.adhesion);

    // Add collision geometry for the container walls
    chrono::utils::AddBoxContainer(box, cmaterial,                                 //
                                   ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,           //
                                   ChVector3i(2, 2, -1),                           //
                                   false);
    box->EnableCollision(false);
    // Add BCE particles attached on the walls into FSI system
    sysSPH.AddBoxContainerBCE(box,                                            //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(2, 2, -1));

    // Create the wheel -- always SECOND body in the system
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    std::string pad_obj = "fsi/ndlg_foot_layup.obj";

    trimesh->LoadWavefrontMesh(GetChronoDataFile(pad_obj), false, true);
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    double plate_mass = 17.23; // 38 lb from cad file
    ChVector3d plate_inertia = ChVector3d(1.0, 1.0, 1.5);



    // Create plate
    auto plate = chrono_types::make_shared<ChBody>();
    double plate_z_pos = fzDim + cyl_length / 2 + params.initial_spacing;
    plate->SetPos(ChVector3d(0, 0, plate_z_pos));
    plate->SetRot(ChQuaternion<>(1, 0, 0, 0));
    plate->SetFixed(false);

    plate->SetMass(plate_mass);
    plate->SetInertiaXX(plate_inertia);
    sysMBS.AddBody(plate);
    std::vector<ChVector3d> BCE_pad;
    sysSPH.CreatePoints_Mesh(*trimesh, params.initial_spacing, BCE_pad);
    sysSPH.AddPointsBCE(plate, BCE_pad, ChFrame<>(), true);
    sysFSI.AddFsiBody(plate);

    //sysSPH.AddCylinderBCE(plate, ChFrame<>(VNULL, QUNIT), params.plate_diameter / 2, cyl_length, true, true);
    sysFSI.Initialize();

    // Add motor to push the plate at a force that increases the pressure to max pressure in t_end
    auto motor = chrono_types::make_shared<ChLinkMotorLinearForce>();

     std::shared_ptr<ChFunctionInterp> force_func = chrono_types::make_shared<ChFunctionInterp>();

    // read force data from a file
  //   std::string force_file = GetChronoDataFile("fsi/loading.csv");
  //   std::ifstream file(force_file);
  //   std::vector<double> force_data; 

  //   // skip the first line, populate force_data
  //   std::string line;
  //   std::getline(file, line);
  //   while (std::getline(file, line)) {
		// std::stringstream ss(line);
		// double force;
		// ss >> force;
		// force_data.push_back(force);
	 //}
  //   int num_force_points = force_data.size();
  //   double force_time_step = (t_end - time_preload) / (num_force_points-1);
  //   

  //   force_func->AddPoint(0, 0);
  //   for (int i = 0; i < num_force_points; i++) {
		// double t = time_preload + i * force_time_step;
  //       force_func->AddPoint(t, -force_data.at(i));
	 //}


    double max_loading = 8584; 
    double max_loading_duration = 0.125;
    double settled_loading = 5688;

    force_func->AddPoint(0, 0);
    force_func->AddPoint(time_preload, max_loading);
    force_func->AddPoint(time_preload + max_loading_duration, max_loading);
    force_func->AddPoint(t_end-5, settled_loading);  // stay constant for the last 5 sec? 
    force_func->AddPoint(t_end, settled_loading);



    motor->SetForceFunction(force_func);
    motor->Initialize(plate, box, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    sysMBS.AddLink(motor);

    // Output directories
    std::string out_dir;
    if (params.output || params.snapshots) {
        // Base output directory
        std::string base_dir = GetChronoOutputPath() + "h_" + std::to_string(params.initial_spacing) + "_dt_" +
                               std::to_string(params.time_step) + "_newload";
        if (!filesystem::create_directory(filesystem::path(base_dir))) {
            std::cerr << "Error creating directory " << base_dir << std::endl;
            return;
        }

        out_dir = base_dir;

        if (params.output) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
                std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
                return;
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
                std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
                return;
            }
        }
        
        if (params.snapshots) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
                return;
            }
        }
    }

    // Create a run-time visualizer
    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    std::shared_ptr<ChFsiVisualizationVSG> visFSI;
    if (params.render) {
    visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
    visFSI->EnableFluidMarkers(true);
    visFSI->EnableBoundaryMarkers(true);
    visFSI->EnableRigidBodyMarkers(false);
    auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2);
    visFSI->SetSPHColorCallback(col_callback);
    visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
    visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
    // VSG visual system (attach visFSI as plugin)
    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();

    visVSG->SetWindowTitle("FSI Cone Penetrometer");
    visVSG->SetWindowSize(1280, 720);
    visVSG->AddCamera(ChVector3d(0, -3 * container_height, 0.75 * container_height),
                      ChVector3d(0, 0, 0.75 * container_height));
    // visVSG->SetCameraMoveScale(0.1f);
    visVSG->SetLightIntensity(0.9);
    visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

    // visVSG->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
    // visVSG->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
    visVSG->AttachSystem(&sysMBS);
    visVSG->Initialize();
    vis = visVSG;
}
#endif

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int pres_out_frame = 0;
    int render_frame = 0;
    double dT = sysFSI.GetStepSizeCFD();
    double pres_out_fps = 5;

    std::string out_file = out_dir + "/force_vs_time.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    // Add comma-separated header to the output file
    ofile << "Time,Force-x,Force-y,Force-z,position-x,position-y,penetration-depth,plate-vel-z,motor-load"
        << std::endl;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        // Calculate current penetration depth
        double current_depth = plate->GetPos().z() - fzDim - cyl_length / 2 - params.initial_spacing;

        if (params.output && time >= out_frame / params.output_fps) {
            if (params.write_marker_files) {
                sysSPH.SaveParticleData(out_dir + "/particles");
                sysSPH.SaveSolidData(out_dir + "/fsi", time);
                std::cout << " -- Wrote "
                          << "frame " << out_frame
                          << " SPH data to " << out_dir << std::endl;
            }
            out_frame++;
        }
#ifdef CHRONO_VSG
        if (params.render && time >= render_frame / params.render_fps) {
            if (!vis->Run())
                break;

            vis->Render();
            if (params.snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/" << std::setw(5) << std::setfill('0') << render_frame << ".jpg";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }
#endif
        if (time >= pres_out_frame / pres_out_fps) {
            double motor_force = motor->GetMotorForce(); 
            ofile << time << "," << plate->GetAppliedForce().x() << "," << plate->GetAppliedForce().y() << ","
                << plate->GetAppliedForce().z() << "," << plate->GetPos().x() << "," << plate->GetPos().y() << ","
                << current_depth << "," << plate->GetPosDt().z() << "," << motor_force << std::endl;
            pres_out_frame++;
            std::cout << "time, " << time 
                      << ", plate_force, " << plate->GetAppliedForce().z()
                      << ", plate_depth, " << current_depth 
                      << ", motor_force, " << motor->GetMotorForce() 
                      << ", RTF, " << sysFSI.GetRtf()
                      << std::endl;
        }

        // Advance simulation for one timestep for all systems
        sysFSI.DoStepDynamics(dT);
        time += dT;

        sim_frame++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;
}