// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Wei Hu, Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <fstream>
#include <stdio.h>
#include <cstring>
#include <iomanip>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/core/ChTimer.h"
#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// Physical properties of terrain particles
double density = 1700;
double slope_angle;  // Terrain slope

// Dimension of the terrain container
double smalldis = 1.0e-9;
// double bxDim = 5.0 + smalldis;
double bxDim = 2.0 + smalldis;
double byDim = 0.5 + smalldis;  // byDim depending on the wheel width
double bzDim = 0.1 + smalldis;

// Size of the wheel
double wheel_radius = 0.2;
double wheel_wide = 0.205;

double wheel_slip = 0.0;
// double wheel_vel = -0.05;
// double wheel_AngVel = -0.7; // for rTot = 250mm, 0.4 rad/s ~ 0.1 m/s linear velocity

// Test 4
// double wheel_vel = 0.2;
// double wheel_AngVel = 2.78;  // for rTot = 250mm, 0.4 rad/s ~ 0.1 m/s linear velocity

// Test 3
// double wheel_vel = 0.15;
// double wheel_AngVel = 2.09;  // for rTot = 250mm, 0.4 rad/s ~ 0.1 m/s linear velocity

// Test 2
// double wheel_vel = 0.15;
// double wheel_AngVel = 2.09;  // for rTot = 250mm, 0.4 rad/s ~ 0.1 m/s linear velocity

// double total_mass = 2.5 * 2.;

// Initial Position of wheel
ChVector3d wheel_IniPos(-bxDim / 2 + wheel_radius * 1.2, 0.0, wheel_radius + bzDim / 2.0);
// ChVector<> wheel_IniVel(0.0, 0.0, -5.0f);
ChVector3d wheel_IniVel(0.0, 0.0, 0.0f);

// Simulation time and stepsize
// double total_time = 5.0;
// double total_time = 60.0;
double total_time = 20;
// double total_time = 1.0;

// linear actuator and angular actuator
auto actuator = chrono_types::make_shared<ChLinkLockLinActuator>();
auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();

// Save data as csv files to see the results off-line using Paraview
bool output = true;
int out_fps = 100;
bool write_marker_files = false;
// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = false;
float render_fps = 100;

// Verbose terminal output
bool verbose_fsi = true;
bool verbose = true;

std::string drum_obj = "robot/rassor/obj/single_drum.obj";
std::string drum_BCE_csvfile = "robot/rassor/bce/single_drum.csv";

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChFsiSystemSPH& sysFSI, double wheel_vel, double wheel_AngVel, double total_mass) {
    ChFsiFluidSystemSPH& sysSPH = sysFSI.GetFluidSystemSPH();
    ChSystem& sysMBS = sysFSI.GetMultibodySystem();
    // Common contact material
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.9f);
    cmaterial->SetRestitution(0.4f);
    cmaterial->SetAdhesion(0);

    // Create a container -- always FIRST body in the system
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    // Add BCE particles attached on the walls into FSI system
    sysSPH.AddBoxContainerBCE(ground, ChFrame<>(), ChVector3d(bxDim, byDim, bzDim), ChVector3i(2, 0, -1));

    // Create the wheel -- always SECOND body in the system
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.f;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(drum_obj), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    // Compute mass inertia from mesh
    double mmass;
    double mdensity = 1500.0;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    // look into principal inertia computation!!! why it's so small!!!!
    principal_I.x() = principal_I.z();
    mcog = ChVector3d(0.0, 0.0, 0.0);

    // Set the abs orientation, position and velocity
    auto drum = chrono_types::make_shared<ChBodyAuxRef>();
    ChQuaternion wheel_Rot = QUNIT;

    // Set the COG coordinates to barycenter, without displacing the REF reference.
    // Make the COG frame a principal frame.
    drum->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    drum->SetMass(total_mass * 1.0 / 2.0);
    drum->SetInertiaXX(ChVector3d(0.0058, 0.02, 0.0058));
    std::cout << "principal inertia: " << std::endl;
    std::cout << mdensity * principal_I << std::endl;
    drum->SetPosDt(wheel_IniVel);
    drum->SetAngVelLocal(ChVector3d(0.0, 0.0, 0.0));  // set an initial anular velocity (rad/s)

    // Set the absolute position of the body:
    drum->SetFrameRefToAbs(ChFrame<>(ChVector3d(wheel_IniPos), ChQuaternion<>(wheel_Rot)));
    sysMBS.AddBody(drum);

    drum->SetFixed(false);
    drum->EnableCollision(false);

    // BCE drum vector to store the drum BCE
    std::vector<ChVector3d> BCE_drum;

    // read the drum BCE from csv file, add points to the BCE_drum vector
    std::ifstream file(GetChronoDataFile(drum_BCE_csvfile));
    std::string line;
    std::getline(file, line);  // skip the first line
    while (std::getline(file, line)) {
        std::stringstream iss(line);
        std::string val;
        std::vector<double> values;
        while (std::getline(iss, val, ',')) {
            values.push_back(std::stod(val));
        }
        BCE_drum.push_back(ChVector3d(values[0], values[1], values[2]));
    }
    // Now add BCE particles to the ChBody drum
    sysSPH.AddPointsBCE(drum, BCE_drum, ChFrame<>(), true);
    // Now add the drum to the FSI system
    sysFSI.AddFsiBody(drum);

    std::cout << "Added " << BCE_drum.size() << " BCE particles for rassor wheel" << std::endl;

    // Create the chassis -- always THIRD body in the system
    auto chassis = chrono_types::make_shared<ChBody>();
    chassis->SetMass(total_mass / 2.0);
    chassis->SetPos(drum->GetPos());
    chassis->EnableCollision(false);
    chassis->SetFixed(false);

    // Add geometry of the chassis.
    sysMBS.AddBody(chassis);

    // Create the axle -- always FOURTH body in the system # this weight affect the loading!!! not chassis!!!
    auto axle = chrono_types::make_shared<ChBody>();
    axle->SetMass(total_mass / 2.0);
    axle->SetPos(drum->GetPos());
    axle->EnableCollision(false);
    axle->SetFixed(false);

    // Add geometry of the axle.
    // chrono::utils::AddSphereGeometry(axle.get(), cmaterial, 0.5, ChVector3d(0, 0, 0));

    sysMBS.AddBody(axle);

    // Connect the chassis to the containing bin (ground) through a translational joint and create a linear actuator.
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, chassis, ChFrame<>(chassis->GetPos(), QuatFromAngleY(CH_PI_2)));
    prismatic1->SetName("prismatic_chassis_ground");
    sysMBS.AddLink(prismatic1);

    double velocity = wheel_vel;  // wheel_AngVel * wheel_radius * (1.0 - wheel_slip);
    auto actuator_fun = chrono_types::make_shared<ChFunctionRamp>(0.0, velocity);

    actuator->Initialize(ground, chassis, false, ChFrame<>(chassis->GetPos(), QUNIT),
                         ChFrame<>(chassis->GetPos() + ChVector3d(1, 0, 0), QUNIT));
    actuator->SetName("actuator");
    actuator->SetDistanceOffset(1);
    actuator->SetActuatorFunction(actuator_fun);
    sysMBS.AddLink(actuator);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(chassis, axle, ChFrame<>(chassis->GetPos(), QUNIT));
    prismatic2->SetName("prismatic_axle_chassis");
    sysMBS.AddLink(prismatic2);

    // Connect the wheel to the axle through a engine joint.
    motor->SetName("engine_wheel_axle");
    motor->Initialize(drum, axle, ChFrame<>(drum->GetPos(), QuatFromAngleAxis(-CH_PI / 2.0, ChVector3d(1, 0, 0))));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, wheel_AngVel));
    sysMBS.AddLink(motor);
}

// =============================================================================

struct SimParams {
    // Simulation parameters
    int ps_freq;
    double initial_spacing;
    double d0_multiplier;
    double time_step;
    std::string boundary_type;
    std::string viscosity_type;
    std::string kernel_type;
    double artificial_viscosity;

    // Physical parameters
    double wheel_vel;
    double wheel_AngVel;
    double total_mass;

    // Output parameters
    bool output;
    int out_fps;
    bool render;
    float render_fps;
    bool snapshots;  // Added for snapshot functionality
};

// Function to handle CLI arguments
bool GetProblemSpecs(int argc, char** argv, SimParams& params) {
    ChCLI cli(argv[0], "FSI Rassor Single Drum Demo");

    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(params.ps_freq));
    cli.AddOption<double>("Simulation", "initial_spacing", "Initial spacing", std::to_string(params.initial_spacing));
    cli.AddOption<double>("Simulation", "d0_multiplier", "D0 multiplier", std::to_string(params.d0_multiplier));
    cli.AddOption<std::string>("Simulation", "boundary_type", "Boundary condition type (holmes/adami)",
                               params.boundary_type);
    cli.AddOption<std::string>("Simulation", "viscosity_type",
                               "Viscosity type (artificial_unilateral/artificial_bilateral)", params.viscosity_type);
    cli.AddOption<std::string>("Simulation", "kernel_type", "Kernel type (cubic/wendland)", params.kernel_type);
    cli.AddOption<double>("Simulation", "time_step", "Time step", std::to_string(params.time_step));
    cli.AddOption<double>("Simulation", "artificial_viscosity", "Artificial viscosity",
                          std::to_string(params.artificial_viscosity));

    cli.AddOption<double>("Physics", "wheel_vel", "Wheel velocity", std::to_string(params.wheel_vel));
    cli.AddOption<double>("Physics", "wheel_AngVel", "Wheel angular velocity", std::to_string(params.wheel_AngVel));
    cli.AddOption<double>("Physics", "total_mass", "Total mass", std::to_string(params.total_mass));

    cli.AddOption<bool>("Output", "output", "Enable output", params.output ? "true" : "false");
    cli.AddOption<int>("Output", "out_fps", "Output frequency", std::to_string(params.out_fps));
    cli.AddOption<bool>("Visualization", "render", "Enable rendering", params.render ? "true" : "false");
    cli.AddOption<float>("Visualization", "render_fps", "Render frequency", std::to_string(params.render_fps));
    cli.AddOption<bool>("Visualization", "snapshots", "Enable writing snapshot image files",
                        params.snapshots ? "true" : "false");

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

    params.wheel_vel = cli.GetAsType<double>("wheel_vel");
    params.wheel_AngVel = cli.GetAsType<double>("wheel_AngVel");
    params.total_mass = cli.GetAsType<double>("total_mass");

    params.output = cli.GetAsType<bool>("output");
    params.out_fps = cli.GetAsType<int>("out_fps");
    params.render = cli.GetAsType<bool>("render");
    params.render_fps = cli.GetAsType<float>("render_fps");
    params.snapshots = cli.GetAsType<bool>("snapshots");

    return true;
}

int main(int argc, char* argv[]) {
    // The path to the Chrono data directory
    // SetChronoDataPath(CHRONO_DATA_DIR);
    // SetChronoDataPath("");

    // Set default simulation parameters
    SimParams params = {/*ps_freq*/ 1,
                        /*initial_spacing*/ 0.005,
                        /*d0_multiplier*/ 1.2,
                        /*time_step*/ 1e-4,
                        /*boundary_type*/ "adami",
                        /*viscosity_type*/ "artificial_bilateral",
                        /*kernel_type*/ "cubic",
                        /*artificial_viscosity*/ 0.02,
                        /*wheel_vel*/ 0.15,
                        /*wheel_AngVel*/ 2.09,
                        /*total_mass*/ 2.5 * 3.,
                        /*output*/ true,
                        /*out_fps*/ 200,
                        /*render*/ false,
                        /*render_fps*/ 100,
                        /*snapshots*/ true};

    // Parse command line arguments
    if (!GetProblemSpecs(argc, argv, params)) {
        return 1;
    }

    // Update global variables to use the values from params
    double wheel_vel = params.wheel_vel;
    double wheel_AngVel = params.wheel_AngVel;
    double total_mass = params.total_mass;

    // Create the MBS and FSI systems
    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    sysFSI.SetVerbose(verbose_fsi);

    slope_angle = 0;

    std::cout << "Problem Specifications:" << std::endl;
    std::cout << "---------------------" << std::endl;
    std::cout << "ps_freq: " << params.ps_freq << std::endl;
    std::cout << "initial_spacing: " << params.initial_spacing << std::endl;
    std::cout << "d0_multiplier: " << params.d0_multiplier << std::endl;
    std::cout << "time_step: " << params.time_step << std::endl;
    std::cout << "boundary_type: " << params.boundary_type << std::endl;
    std::cout << "viscosity_type: " << params.viscosity_type << std::endl;
    std::cout << "kernel_type: " << params.kernel_type << std::endl;
    std::cout << "artificial_viscosity: " << params.artificial_viscosity << std::endl;
    std::cout << "wheel_vel: " << params.wheel_vel << std::endl;
    std::cout << "wheel_AngVel: " << params.wheel_AngVel << std::endl;
    std::cout << "total_mass: " << params.total_mass << std::endl;
    std::cout << "slope_angle: " << slope_angle << std::endl;
    std::cout << "out_fps: " << params.out_fps << std::endl;
    std::cout << "---------------------" << std::endl;

    // Output directories
    std::string out_dir;
    if (params.output) {
        try {
            // Create base output directory first - GetChronoOutputPath() should handle this already
            std::string chrono_output_path = GetChronoOutputPath();

            // Base output directory
            std::stringstream wheel_params;
            wheel_params << std::fixed << std::setprecision(2);
            wheel_params << "wheel_vel_" << params.wheel_vel << "_wheel_AngVel_" << params.wheel_AngVel
                         << "_total_mass_" << params.total_mass;

            // std::string folder_name = "FSI_Rassor_SingleDrum_noActive";
            std::string folder_name = "FSI_Rassor_SingleDrum";
            std::string base_dir = chrono_output_path + folder_name + "/" + wheel_params.str() + "/";

            // Try to create the directory structure - ignoring errors if directories already exist
            filesystem::create_directory(filesystem::path(chrono_output_path + folder_name + "/"));
            filesystem::create_directory(filesystem::path(base_dir));

            // Create directory with all parameters in a single folder
            std::stringstream ss;
            ss << "boundary_" << params.boundary_type;
            ss << "_viscosity_" << params.viscosity_type;
            ss << "_ps_" << params.ps_freq;
            ss << "_s_" << params.initial_spacing;
            ss << "_d0_" << params.d0_multiplier;
            ss << "_av_" << params.artificial_viscosity;
            out_dir = base_dir + ss.str();

            filesystem::create_directory(filesystem::path(out_dir));
            filesystem::create_directory(filesystem::path(out_dir + "/particles"));
            filesystem::create_directory(filesystem::path(out_dir + "/fsi"));

            // Create directory for snapshots if enabled
            if (params.snapshots) {
                filesystem::create_directory(filesystem::path(out_dir + "/snapshots"));
            }
        } catch (const std::exception& e) {
            std::cerr << "Error creating directory structure: " << e.what() << std::endl;
            return 1;
        }
    }

    double gravity_G = -9.81;
    ChVector3d gravity = ChVector3d(gravity_G * sin(slope_angle), 0, gravity_G * cos(slope_angle));
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    // Set the simulation stepsize
    sysFSI.SetStepSizeCFD(params.time_step);
    sysFSI.SetStepsizeMBD(params.time_step);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = params.initial_spacing;
    sph_params.d0_multiplier = params.d0_multiplier;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.kernel_threshold = 0.8;
    sph_params.artificial_viscosity = params.artificial_viscosity;
    sph_params.num_proximity_search_steps = params.ps_freq;

    // Set kernel type
    if (params.kernel_type == "cubic") {
        sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    } else if (params.kernel_type == "wendland") {
        sph_params.kernel_type = KernelType::WENDLAND;
    }

    // Set boundary type
    if (params.boundary_type == "holmes") {
        sph_params.boundary_type = BoundaryType::HOLMES;
    } else if (params.boundary_type == "adami") {
        sph_params.boundary_type = BoundaryType::ADAMI;
    }

    // Set viscosity type
    if (params.viscosity_type == "artificial_bilateral") {
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    } else if (params.viscosity_type == "artificial_unilateral") {
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;
    }

    sysSPH.SetSPHParameters(sph_params);

    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = density;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.03;
    mat_props.mu_fric_s = 0.7;
    mat_props.mu_fric_2 = 0.7;
    mat_props.average_diam = 0.0025;
    mat_props.cohesion_coeff = 0;

    sysSPH.SetElasticSPH(mat_props);
    sysSPH.SetActiveDomain(ChVector3d(0.5, 0.4, 0.7));
    sysSPH.SetActiveDomainDelay(0.0);

    // Set the terrain container size
    sysSPH.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysSPH.SetConsistentDerivativeDiscretization(false, false);

    // Set up the periodic boundary condition (if not, set relative larger values)
    ChVector3d cMin(-bxDim / 2 - 3 * params.initial_spacing, -byDim / 2 - 0.5 * params.initial_spacing, -bzDim * 5);
    ChVector3d cMax(bxDim / 2 + 3 * params.initial_spacing, byDim / 2 + 0.5 * params.initial_spacing, bzDim * 5);
    sysSPH.SetComputationalBoundaries(cMin, cMax, PeriodicSide::Y);

    ChVector3d boxCenter(0.0, 0.0, 0.0);
    ChVector3d boxHalfDim(bxDim / 2 - params.initial_spacing, byDim / 2 - params.initial_spacing,
                          bzDim / 2 - params.initial_spacing);
    sysSPH.AddBoxSPH(boxCenter, boxHalfDim);

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(sysFSI, params.wheel_vel, params.wheel_AngVel, params.total_mass);
    // Set simulation data output level
    sysSPH.SetOutputLevel(OutputLevel::STATE);
    // Construction of the FSI system must be finalized before running
    sysFSI.Initialize();

    auto wheel = sysMBS.GetBodies()[1];

    auto reaction = actuator->GetReaction2();
    ChVector3d force = reaction.force;
    ChVector3d torque = motor->GetReaction1().torque;
    ChVector3d w_pos = wheel->GetPos();
    ChVector3d w_vel = wheel->GetPosDt();
    ChVector3d angvel = wheel->GetAngVelLocal();

    ChVector3d w_pos_init = wheel->GetPos();

    // Initialize simulation time
    double time = 0.0;
    // Start the timer
    ChTimer timer;
    timer.start();

    // Write the information into a txt file
    std::ofstream myFile;
    std::ofstream myDBP_Torque;
    if (params.output) {
        myFile.open(out_dir + "/results.txt", std::ios::trunc);
        if (!myFile.is_open()) {
            std::cerr << "Error opening file " << out_dir + "/results.txt" << std::endl;
            return 1;
        }

        myDBP_Torque.open(out_dir + "/DBP_Torque.txt", std::ios::trunc);
        if (!myDBP_Torque.is_open()) {
            std::cerr << "Error opening file " << out_dir + "/DBP_Torque.txt" << std::endl;
            return 1;
        }

        myFile << "Time,x,y,y,vx,omg_y,fx,fy,fz,trq_x,trq_y,trq_z\n";
        myFile << time << ", " << w_pos.x() - w_pos_init.x() << ", " << w_pos.y() - w_pos_init.y() << ", "
               << w_pos.z() - w_pos_init.z() << ", " << w_vel.x() << ", " << angvel.y() << ", " << force.x() << ","
               << force.y() << ", " << force.z() << ", " << torque.x() << ", " << torque.y() << ", " << torque.z()
               << "\n";
    }
    std::shared_ptr<ChVisualSystem> vis;
#ifdef CHRONO_VSG
    if (params.render) {
        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(false);
        visFSI->EnableRigidBodyMarkers(true);

        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Chrono::FSI single drum RASSOR demo");
        visVSG->SetWindowSize(1280, 720);
        visVSG->AddCamera(ChVector3d(0, -5 * byDim, 5 * bzDim), ChVector3d(0, 0, 0));
        visVSG->SetLightIntensity(0.9);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#endif  // CHRONO_VSG

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (params.out_fps * params.time_step));
    unsigned int render_steps = (unsigned int)round(1 / (params.render_fps * params.time_step));

    int current_step = 0;
    int render_frame = 0;

    double rtf_average = 0.0;
    unsigned int rtf_count = 0;

    while (time < total_time) {
        // Get the infomation of the wheel
        reaction = actuator->GetReaction2();
        force = reaction.force;
        torque = motor->GetReaction1().torque;
        w_pos = wheel->GetPos();
        w_vel = wheel->GetPosDt();
        angvel = wheel->GetAngVelLocal();

        if (time < 0.1) {
            w_pos_init = wheel->GetPos();
        }

        if (params.output && current_step % output_steps == 0) {
            myFile << time << ", " << w_pos.x() - w_pos_init.x() << ", " << w_pos.y() - w_pos_init.y() << ", "
                   << w_pos.z() - w_pos_init.z() << ", " << w_vel.x() << ", " << angvel.y() << ", " << force.x() << ", "
                   << force.y() << ", " << force.z() << ", " << torque.x() << ", " << torque.y() << ", " << torque.z()
                   << "\n";
        }

        if (params.output && current_step % render_steps == 0) {
            std::cout << "-------- Output" << std::endl;
            if (write_marker_files) {
                sysSPH.SaveParticleData(out_dir + "/particles");
                sysSPH.SaveSolidData(out_dir + "/fsi", time);
            }
            std::cout << "time: " << time << std::endl;
            std::cout << "  wheel position:         " << w_pos << std::endl;
            std::cout << "  wheel linear velocity:  " << w_vel << std::endl;
            std::cout << "  wheel angular velocity: " << angvel << std::endl;
            std::cout << "  drawbar pull:           " << force << std::endl;
            std::cout << "  wheel torque:           " << torque << std::endl;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics(params.time_step);
        double rtf = sysFSI.GetRtf();
        rtf_average = (rtf_average * rtf_count + rtf) / (rtf_count + 1);
        rtf_count++;
        time += params.time_step;
        current_step++;

        if (w_pos.x() + wheel_radius > bxDim / 2.0f) {
            std::cout << "Wheel has reached the end of the container" << std::endl;
            std::cout << "  wheel position:         " << w_pos << std::endl;
            std::cout << "container position, " << bxDim / 2.0f << std::endl;
            break;
        }

#ifdef CHRONO_VSG
        if (params.render && current_step % render_steps == 0) {
            if (!vis->Run())
                break;
            vis->Render();

            // Save snapshots if enabled
            if (params.snapshots) {
                std::cout << "-------- Snapshot" << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/" << std::setw(5) << std::setfill('0') << render_frame << ".jpg";
                vis->WriteImageToFile(filename.str());
            }
            render_frame++;
        }
#endif
    }

    // Stop the timer and record runtime info
    timer.stop();

    std::cout << "Runtime: " << timer() << " seconds\n" << std::endl;
    std::cout << "Simulation time: " << time << std::endl;
    std::cout << "Average RTF: " << rtf_average << std::endl;
    std::cout << "Simulation finished" << std::endl;

    // Write runtime information to a file
    if (params.output) {
        std::ofstream runtime_file(out_dir + "/runtime.txt");
        runtime_file << "Runtime: " << timer() << " seconds\n" << std::endl;
        runtime_file << "Simulation time: " << time << std::endl;
        runtime_file << "Average RTF: " << rtf_average << std::endl;
        runtime_file << "ps_freq: " << params.ps_freq << std::endl;
        runtime_file << "Simulation finished";
        runtime_file.close();
    }

    return 0;
}