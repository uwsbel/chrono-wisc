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
// Author: Huzaifa Unjhawala
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
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif
#include "chrono_fsi/sph/utils/UtilsTimingOutput.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"
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
double wheel_vel = 0.15;
double wheel_AngVel = 2.09;  // for rTot = 250mm, 0.4 rad/s ~ 0.1 m/s linear velocity

double total_mass = 2.5 * 2.;

// Initial Position of wheel
ChVector3d wheel_IniPos(-bxDim / 2 + wheel_radius * 1.2, 0.0, wheel_radius + bzDim / 2.0);
// ChVector<> wheel_IniVel(0.0, 0.0, -5.0f);
ChVector3d wheel_IniVel(0.0, 0.0, 0.0f);

// Simulation time and stepsize
// double total_time = 5.0;
// double total_time = 60.0;

// linear actuator and angular actuator
auto actuator = chrono_types::make_shared<ChLinkLockLinActuator>();
auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();

// Save data as csv files to see the results off-line using Paraview
bool output = false;
int out_fps = 200;

// Enable/disable run-time visualization
bool render = false;
float render_fps = 100;

// Verbose terminal output
bool verbose_fsi = true;
bool verbose = true;

std::string drum_obj = "robot/rassor/obj/single_drum.obj";
std::string drum_BCE_csvfile = "robot/rassor/bce/single_drum.csv";
// -----------------------------------------------------------------------------

bool GetProblemSpecs(int argc,
                     char** argv,
                     double& t_end,
                     int& ps_freq,
                     std::string& boundary_type,
                     std::string& viscosity_type,
                     double& d0_multiplier) {
    ChCLI cli(argv[0], "Flexible cable FSI demo");

    cli.AddOption<double>("Input", "t_end", "Simulation duration [s]");
    cli.AddOption<int>("Proximity Search", "ps_freq", "Frequency of Proximity Search");
    cli.AddOption<std::string>("Physics", "boundary_type", "Boundary condition type (holmes/adami)");
    cli.AddOption<std::string>("Physics", "viscosity_type",
                               "Viscosity type (laminar/artificial_unilateral/artificial_bilateral)");
    cli.AddOption<double>("Physics", "d0_multiplier", "SPH density multiplier");

    if (argc == 1) {
        std::cout << "Required parameters missing. See required parameters and descriptions below:\n\n";
        cli.Help();
        return false;
    }

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    t_end = cli.GetAsType<double>("t_end");
    ps_freq = cli.GetAsType<int>("ps_freq");
    boundary_type = cli.GetAsType<std::string>("boundary_type");
    viscosity_type = cli.GetAsType<std::string>("viscosity_type");
    d0_multiplier = cli.GetAsType<double>("d0_multiplier");

    return true;
}
//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChFsiSystemSPH& sysFSI) {
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

int main(int argc, char* argv[]) {
    // The path to the Chrono data directory
    // SetChronoDataPath(CHRONO_DATA_DIR);
    // SetChronoDataPath("");

    // Create the MBS and FSI systems
    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    sysFSI.SetVerbose(verbose_fsi);
    double dT = 1e-4;
    double t_end = 2;
    double iniSpacing = 0.005;
    double d0_multiplier = 1.2;
    std::string boundary_type = "adami";
    std::string viscosity_type = "artificial_bilateral";
    int ps_freq = 1;
    if (!GetProblemSpecs(argc, argv, t_end, ps_freq, boundary_type, viscosity_type, d0_multiplier)) {
        return 1;
    }

    slope_angle = 0;
    SetChronoOutputPath("BENCHMARK3_RTF/");
    // Create oputput directories
    std::string out_dir = GetChronoOutputPath() + "FSI_RASSOR_SingleDrum/";

    // Create oputput directories
    if (!filesystem::create_subdirectory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    out_dir = out_dir + "CRM_WCSPH/";

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (output) {
        if (!filesystem::create_subdirectory(filesystem::path(out_dir + "/particles"))) {
            std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
            return 1;
        }
        if (!filesystem::create_subdirectory(filesystem::path(out_dir + "/fsi"))) {
            std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
            return 1;
        }
    }
    double gravity_G = -9.81;
    ChVector3d gravity = ChVector3d(gravity_G * sin(slope_angle), 0, gravity_G * cos(slope_angle));
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);
    // Set the simulation stepsize
    sysFSI.SetStepSizeCFD(dT);
    sysFSI.SetStepsizeMBD(dT);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = iniSpacing;
    sph_params.d0_multiplier = d0_multiplier;
    sph_params.shifting_method = ShiftingMethod::XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.kernel_threshold = 0.8;
    sph_params.artificial_viscosity = 0.05;
    sph_params.num_proximity_search_steps = ps_freq;
    if (boundary_type == "holmes")
        sph_params.boundary_type = BoundaryType::HOLMES;
    else
        sph_params.boundary_type = BoundaryType::ADAMI;

    if (viscosity_type == "laminar")
        sph_params.viscosity_type = ViscosityType::LAMINAR;
    else if (viscosity_type == "artificial_bilateral")
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    else
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;

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
    sysSPH.SetActiveDomain(ChVector3d(0.45, 0.15, 0.45));
    // sysSPH.SetActiveDomainDelay(0.0);

    // Set the terrain container size
    sysSPH.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysSPH.SetConsistentDerivativeDiscretization(false, false);
    sysSPH.SetKernelType(KernelType::CUBIC_SPLINE);

    // Set up the periodic boundary condition (if not, set relative larger values)
    ChVector3d cMin(-bxDim / 2 - 3 * iniSpacing, -byDim / 2, -bzDim * 2.5);
    ChVector3d cMax(bxDim / 2 + 3 * iniSpacing, byDim / 2, bzDim * 2.5);
    sysSPH.SetComputationalBoundaries(cMin, cMax, PeriodicSide::Y);

    ChVector3d boxCenter(0.0, 0.0, 0.0);
    ChVector3d boxHalfDim(bxDim / 2 - iniSpacing, byDim / 2 - iniSpacing, bzDim / 2 - iniSpacing);
    sysSPH.AddBoxSPH(boxCenter, boxHalfDim);

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(sysFSI);
    // Set simulation data output level
    sysSPH.SetOutputLevel(OutputLevel::STATE);
    // Construction of the FSI system must be finalized before running
    sysFSI.Initialize();

    auto wheel = sysMBS.GetBodies()[1];

    auto reaction = actuator->GetReaction2();
    ChVector3d force = reaction.force;
    ChVector3d torque = reaction.torque;
    ChVector3d w_pos = wheel->GetPos();
    ChVector3d w_vel = wheel->GetPosDt();
    ChVector3d angvel = wheel->GetAngVelLocal();

    ChVector3d w_pos_init = wheel->GetPos();

    // Save wheel mesh
    // ChTriangleMeshConnected wheel_mesh;
    // wheel_mesh.LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    // wheel_mesh.RepairDuplicateVertexes(1e-9);

    // Write the information into a txt file
    std::ofstream myFile;
    std::ofstream myDBP_Torque;
    if (output) {
        myFile.open(out_dir + "/results.txt", std::ios::trunc);
        myDBP_Torque.open(out_dir + "/DBP_Torque.txt", std::ios::trunc);
        myFile << "Time,x,y,y,vx,omg_y,fx,fy,fz,trq_x,trq_y,trq_z\n";
        myFile << time << ", " << w_pos.x() - w_pos_init.x() << ", " << w_pos.y() - w_pos_init.y() << ", "
               << w_pos.z() - w_pos_init.z() << ", " << w_vel.x() << ", " << angvel.y() << ", " << force.x() << ","
               << force.y() << ", " << force.z() << ", " << torque.x() << ", " << torque.y() << ", " << torque.z()
               << "\n";
    }

#ifdef CHRONO_VSG
    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;
    if (render) {
        // FSI plugin
        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(true);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("RASSOR Single Wheel Demo");
        visVSG->SetWindowSize(1280, 720);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(0, -5 * byDim, 5 * bzDim), ChVector3d(0, 0, 0));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#endif  // CHRONO_VSG

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));
    double time = 0.0;
    int render_frame = 0;
    int current_step = 0;
    double timer_step = 0;
    double timer_CFD = 0;
    double timer_MBS = 0;
    double timer_FSI = 0;
    while (time < t_end) {
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
#ifdef CHRONO_VSG
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            render_frame++;
        }
#endif
        //}

        if (output && current_step % output_steps == 0) {
            // myFile << time << "," << w_pos.x() << "\t" << w_pos.y() << "\t" << w_pos.z() << "\t" << w_vel.x() <<
            // "\t"
            // << angvel.y() << "\t" << angvel.z() << torque.x() << "\t"
            //     << torque.y() << "\t" << torque.z() << "\n";
            myFile << time << ", " << w_pos.x() - w_pos_init.x() << ", " << w_pos.y() - w_pos_init.y() << ", "
                   << w_pos.z() - w_pos_init.z() << ", " << w_vel.x() << ", " << angvel.y() << ", " << force.x() << ", "
                   << force.y() << ", " << force.z() << ", " << torque.x() << ", " << torque.y() << ", " << torque.z()
                   << "\n";
        }

        if (output && current_step % render_steps == 0) {
            // if (output) {

            std::cout << "-------- Output" << std::endl;
            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);
            static int counter = 0;
            std::cout << "time: " << time << std::endl;
            std::cout << "  wheel position:         " << w_pos << std::endl;
            std::cout << "  wheel linear velocity:  " << w_vel << std::endl;
            std::cout << "  wheel angular velocity: " << angvel << std::endl;
            std::cout << "  drawbar pull:           " << force << std::endl;
            std::cout << "  wheel torque:           " << torque << std::endl;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics(dT);
        time += dT;
        current_step++;
        timer_step += sysFSI.GetTimerStep();
        timer_CFD += sysFSI.GetTimerCFD();
        timer_MBS += sysFSI.GetTimerMBD();
        timer_FSI += sysFSI.GetTimerFSI();
    }
    // Create Output JSON file
    rapidjson::Document doc;
    // Generate filename
    std::ostringstream d0_str;
    d0_str << std::fixed << std::setprecision(1) << d0_multiplier;
    std::string d0_formatted = d0_str.str();
    d0_formatted.erase(d0_formatted.find_last_not_of('0') + 1, std::string::npos);
    if (d0_formatted.back() == '.')
        d0_formatted.pop_back();
    std::string json_file_path = out_dir + "/rtf_" + viscosity_type + "_" + boundary_type + "_ps" +
                                 std::to_string(ps_freq) + "_d0" + d0_formatted + ".json";
    OutputParameterJSON(json_file_path, &sysFSI, t_end, dT, viscosity_type, boundary_type, ps_freq, d0_multiplier, doc);
    OutputTimingJSON(json_file_path, timer_step, timer_CFD, timer_MBS, timer_FSI, &sysFSI, doc);

    return 0;
}