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
// Author: Wei Hu, Radu Serban (Modified for SCM)
// =============================================================================

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>

#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/ChConfig.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

// Include SCM terrain instead of FSI
#include "chrono_vehicle/terrain/SCMTerrain.h"

// Visualization includes
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::vehicle;  // For SCM terrain

// Run-time visualization system
double render_fps = 120;

double smalldis = 1.0e-9;
double bxDim = 2.0 + smalldis;
double byDim = 0.5 + smalldis;
double bzDim = 0.1 + smalldis;

// Size of the wheel
double wheel_radius = 0.2;
double wheel_wide = 0.205;

double total_time = 20;

// linear actuator and angular actuator
auto actuator = chrono_types::make_shared<ChLinkLockLinActuator>();
auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();

// Save data as csv files to see the results off-line using Paraview
bool output = true;
int out_fps = 100;

// Output directories and settings
std::string out_dir = GetChronoOutputPath() + "SCM_Rassor_SingleDrum";

// Enable/disable run-time visualization
bool render = true;
bool render_wireframe = true;
bool render_sinkage = true;

// Verbose terminal output
bool verbose = true;

std::string drum_obj = "robot/rassor/obj/single_drum.obj";

struct SimParams {
    // Simulation parameters
    double time_step;
    double grid_spacing;

    // Physical parameters
    double total_mass;
    double wheel_vel;
    double wheel_AngVel;
    double gravity_G;

    // SCM terrain parameters
    double Bekker_Kphi;
    double Bekker_Kc;
    double Bekker_n;
    double Mohr_cohesion;
    double Mohr_friction;
    double Janosi_shear;
    double elastic_K;
    double damping_R;

    int sim_number;
};

// Function to handle CLI arguments
bool GetProblemSpecs(int argc, char** argv, SimParams& params) {
    ChCLI cli(argv[0], "SCM Rassor Single Drum Demo");

    cli.AddOption<double>("Simulation", "time_step", "Time step", std::to_string(params.time_step));
    cli.AddOption<double>("Simulation", "grid_spacing", "SCM grid spacing", std::to_string(params.grid_spacing));

    cli.AddOption<double>("Physics", "total_mass", "Total mass", std::to_string(params.total_mass));
    cli.AddOption<double>("Physics", "wheel_vel", "Wheel velocity", std::to_string(params.wheel_vel));
    cli.AddOption<double>("Physics", "wheel_AngVel", "Wheel angular velocity", std::to_string(params.wheel_AngVel));
    cli.AddOption<double>("Physics", "gravity_G", "Gravity", std::to_string(params.gravity_G));

    cli.AddOption<double>("SCM", "Bekker_Kphi", "Bekker Kphi (frictional modulus)", std::to_string(params.Bekker_Kphi));
    cli.AddOption<double>("SCM", "Bekker_Kc", "Bekker Kc (cohesive modulus)", std::to_string(params.Bekker_Kc));
    cli.AddOption<double>("SCM", "Bekker_n", "Bekker n exponent", std::to_string(params.Bekker_n));
    cli.AddOption<double>("SCM", "Mohr_cohesion", "Mohr cohesion (Pa)", std::to_string(params.Mohr_cohesion));
    cli.AddOption<double>("SCM", "Mohr_friction", "Mohr friction angle (degrees)",
                          std::to_string(params.Mohr_friction));
    cli.AddOption<double>("SCM", "Janosi_shear", "Janosi shear coefficient (m)", std::to_string(params.Janosi_shear));
    cli.AddOption<double>("SCM", "elastic_K", "Elastic stiffness (Pa/m)", std::to_string(params.elastic_K));
    cli.AddOption<double>("SCM", "damping_R", "Damping coefficient (Pa s/m)", std::to_string(params.damping_R));

    cli.AddOption<int>("Simulation", "sim_number", "Simulation number", std::to_string(params.sim_number));

    if (!cli.Parse(argc, argv))
        return false;

    params.time_step = cli.GetAsType<double>("time_step");
    params.grid_spacing = cli.GetAsType<double>("grid_spacing");
    params.total_mass = cli.GetAsType<double>("total_mass");
    params.wheel_vel = cli.GetAsType<double>("wheel_vel");
    params.wheel_AngVel = cli.GetAsType<double>("wheel_AngVel");
    params.gravity_G = cli.GetAsType<double>("gravity_G");

    params.Bekker_Kphi = cli.GetAsType<double>("Bekker_Kphi");
    params.Bekker_Kc = cli.GetAsType<double>("Bekker_Kc");
    params.Bekker_n = cli.GetAsType<double>("Bekker_n");
    params.Mohr_cohesion = cli.GetAsType<double>("Mohr_cohesion");
    params.Mohr_friction = cli.GetAsType<double>("Mohr_friction");
    params.Janosi_shear = cli.GetAsType<double>("Janosi_shear");
    params.elastic_K = cli.GetAsType<double>("elastic_K");
    params.damping_R = cli.GetAsType<double>("damping_R");

    params.sim_number = cli.GetAsType<int>("sim_number");
    return true;
}

//------------------------------------------------------------------
// Function to save drum to Paraview VTK files
//------------------------------------------------------------------
void WriteDrumVTK(const std::string& filename, ChTriangleMeshConnected& mesh, const ChFrame<>& frame) {
    std::ofstream outf;
    outf.open(filename);
    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "VTK from simulation" << std::endl;
    outf << "ASCII" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID" << std::endl;
    outf << "POINTS " << mesh.GetCoordsVertices().size() << " "
         << "float" << std::endl;
    for (auto& v : mesh.GetCoordsVertices()) {
        auto w = frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }
    auto nf = mesh.GetIndicesVertexes().size();
    outf << "CELLS " << nf << " " << 4 * nf << std::endl;
    for (auto& f : mesh.GetIndicesVertexes()) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }
    outf << "CELL_TYPES " << nf << std::endl;
    for (int i = 0; i < nf; i++) {
        outf << "5 " << std::endl;
    }
    outf.close();
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies are created and added to the system
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS,
                      ChVector3d& drum_IniPos,
                      ChVector3d& drum_IniVel,
                      double wheel_AngVel,
                      bool render_drum,
                      double total_mass) {
    // Common contact material
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e9);
    cmaterial->SetFriction(1.f);
    cmaterial->SetRestitution(0.4f);
    cmaterial->SetAdhesion(0);

    // Create a container -- always FIRST body in the system
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetPos(ChVector3d(0., 0., 0.));
    ground->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    // Create the drum -- always SECOND body in the system
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(drum_obj), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0),
                       ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);         // if meshes are not watertight

    // Compute mass inertia from mesh
    double mmass;
    double mdensity = 1500.;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);
    mcog = ChVector3d(0.0, 0.0, 0.0);

    // Set the abs orientation, position and velocity
    auto drum = chrono_types::make_shared<ChBodyAuxRef>();
    ChQuaternion<> drum_Rot = QUNIT;

    // Set the COG coordinates to barycenter, without displacing the REF
    // Make the COG frame a principal frame.
    drum->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    drum->SetMass(total_mass * 1.0 / 2.0);
    drum->SetInertiaXX(ChVector3d(0.0058, 0.02, 0.0058));
    drum->SetPosDt(drum_IniVel);
    drum->SetAngVelLocal(ChVector3d(0.0, 0.0, 0.0));  // set an initial angular velocity (rad/s)

    // drum material
    auto cmaterial2 = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial2->SetYoungModulus(1e9);
    cmaterial2->SetFriction(0.9f);
    cmaterial2->SetRestitution(0.4f);
    cmaterial2->SetAdhesion(0);
    // Set the absolute position of the body:
    drum->SetFrameRefToAbs(ChFrame<>(ChVector3d(drum_IniPos), ChQuaternion<>(drum_Rot)));
    drum->SetFixed(false);
    auto drum_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(cmaterial2, trimesh, false, false, 0.005);
    drum->AddCollisionShape(drum_shape);
    drum->EnableCollision(true);  // Enable collision for SCM
    if (render_drum) {
        auto drum_visual_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        drum_visual_shape->SetMesh(trimesh);
        drum->AddVisualShape(drum_visual_shape);
    }

    sysMBS.AddBody(drum);

    // Create the chassis -- always THIRD body in the system
    auto chassis = chrono_types::make_shared<ChBody>();
    chassis->SetMass(total_mass * 1.0 / 3.0);
    chassis->SetPos(drum->GetPos());
    chassis->EnableCollision(false);
    chassis->SetFixed(false);

    sysMBS.AddBody(chassis);

    // Create the axle -- always FOURTH body in the system
    auto axle = chrono_types::make_shared<ChBody>();
    axle->SetMass(total_mass * 1.0 / 2.0);
    axle->SetPos(drum->GetPos());
    axle->EnableCollision(false);
    axle->SetFixed(false);

    sysMBS.AddBody(axle);

    // Connect the chassis to the containing bin (ground) through a translational
    // joint and create a linear actuator.
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, chassis, ChFrame<>(chassis->GetPos(), QuatFromAngleY(CH_PI_2)));
    prismatic1->SetName("prismatic_chassis_ground");
    sysMBS.AddLink(prismatic1);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(chassis, axle, ChFrame<>(chassis->GetPos(), QUNIT));
    prismatic2->SetName("prismatic_axle_chassis");
    sysMBS.AddLink(prismatic2);

    // Connect the drum to the axle through a engine joint.
    motor->SetName("engine_drum_axle");
    motor->Initialize(drum, axle, ChFrame<>(drum->GetPos(), chrono::QuatFromAngleX(-CH_PI_2)));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, wheel_AngVel));
    sysMBS.AddLink(motor);
}

int main(int argc, char* argv[]) {
    // Create the MBS system
    ChSystemSMC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Default parameters for the simulation
    SimParams params = {/*time_step*/ 2.5e-4,
                        /*grid_spacing*/ 0.01,
                        /*total_mass*/ 2.5 * 3.0,
                        /*wheel_vel*/ 0.15,
                        /*wheel_AngVel*/ 2.09,
                        /*gravity_G*/ 9.81,
                        /*Bekker_Kphi*/ 2e7,     // Bekker Kphi
                        /*Bekker_Kc*/ 0,         // Bekker Kc
                        /*Bekker_n*/ 1.1,        // Bekker n exponent
                        /*Mohr_cohesion*/ 1000,  // Mohr cohesive limit (Pa)
                        /*Mohr_friction*/ 30,    // Mohr friction limit (degrees)
                        /*Janosi_shear*/ 0.01,   // Janosi shear coefficient (m)
                        /*elastic_K*/ 5e8,       // Elastic stiffness (Pa/m), before plastic yield
                        /*damping_R*/ 2e2,       // Damping (Pa s/m), proportional to negative vertical speed
                        /*sim_number*/ 0};

    if (!GetProblemSpecs(argc, argv, params)) {
        return 1;
    }

    std::cout << "Problem Specs:" << std::endl;
    std::cout << "grid_spacing: " << params.grid_spacing << std::endl;
    std::cout << "time_step: " << params.time_step << std::endl;
    std::cout << "Total Mass: " << params.total_mass << std::endl;
    std::cout << "Wheel Velocity: " << params.wheel_vel << std::endl;
    std::cout << "Wheel Angular Velocity: " << params.wheel_AngVel << std::endl;
    std::cout << "Gravity Magnitude: " << params.gravity_G << std::endl;
    std::cout << "SCM Bekker_Kphi: " << params.Bekker_Kphi << std::endl;
    std::cout << "SCM Bekker_Kc: " << params.Bekker_Kc << std::endl;
    std::cout << "SCM Bekker_n: " << params.Bekker_n << std::endl;
    std::cout << "verbose: " << verbose << std::endl;

    // Create output directories
    if (output) {
        // Create output directories
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    // Create formatted output directory path
    std::stringstream ss;
    ss << std::fixed;
    ss << out_dir << "/grid_" << std::setprecision(3) << params.grid_spacing;
    ss << "_AngVel_" << std::setprecision(2) << params.wheel_AngVel << "/";
    out_dir = ss.str();

    if (output) {
        // Create output directories
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    std::string sim_number_str = std::to_string(params.sim_number);
    out_dir = out_dir + sim_number_str + "/";

    // Output the result to verify
    std::cout << "Output directory: " << out_dir << std::endl;

    if (output) {
        // Create output directories
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
            std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
            return 1;
        }
    }

    double gravity_G = -params.gravity_G;
    ChVector3d gravity = ChVector3d(0, 0, gravity_G);
    sysMBS.SetGravitationalAcceleration(gravity);

    double dT = params.time_step;

    // Initial Position of drum
    ChVector3d drum_IniPos(-bxDim / 2 + wheel_radius * 2.0, 0.0, wheel_radius + 10 * params.grid_spacing);
    ChVector3d drum_IniVel(0.0, 0.0, 0.0);

    // Create Solid objects in the MBD system
    CreateSolidPhase(sysMBS, drum_IniPos, drum_IniVel, params.wheel_AngVel, render, params.total_mass);

    // Create the SCM terrain
    SCMTerrain terrain(&sysMBS);

    // Set SCM soil parameters
    terrain.SetSoilParameters(params.Bekker_Kphi,    // Bekker Kphi
                              params.Bekker_Kc,      // Bekker Kc
                              params.Bekker_n,       // Bekker n exponent
                              params.Mohr_cohesion,  // Mohr cohesive limit (Pa)
                              params.Mohr_friction,  // Mohr friction limit (degrees)
                              params.Janosi_shear,   // Janosi shear coefficient (m)
                              params.elastic_K,      // Elastic stiffness (Pa/m), before plastic yield
                              params.damping_R       // Damping (Pa s/m), proportional to negative vertical speed
    );

    // Setup the visualization options
    if (render_sinkage) {
        terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.1);
    } else {
        terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    }

    // Initialize the SCM terrain
    double terrain_length = bxDim * 15;
    double terrain_width = byDim * 1.5;
    terrain.Initialize(terrain_length, terrain_width, params.grid_spacing);

    // Enable bulldozing
    terrain.EnableBulldozing(true);
    terrain.SetBulldozingParameters(55,  // angle of friction for erosion of displaced material at the border of the rut
                                    0.8,  // displaced material vs downward pressed material.
                                    5,    // number of erosion refinements per timestep
                                    10);  // number of concentric vertex selections subject to erosion

    // Add moving patch
    auto drum = sysMBS.GetBodies()[1];
    terrain.AddMovingPatch(drum, ChVector3d(0, 0, 0), ChVector3d(3.0, 2.0, 1.0));

    // Set wireframe visualization if needed
    if (render_wireframe) {
        terrain.GetMesh()->SetWireframe(true);
    }

    auto reaction = actuator->GetReaction2();
    ChVector3d force = reaction.force;
    ChVector3d torque = motor->GetReaction1().torque;
    ChVector3d w_pos = drum->GetPos();
    ChVector3d w_vel = drum->GetPosDt();
    ChVector3d angvel = drum->GetAngVelLocal();

    // Save drum mesh
    ChTriangleMeshConnected drum_mesh;
    drum_mesh.LoadWavefrontMesh(GetChronoDataFile(drum_obj), false, true);
    drum_mesh.RepairDuplicateVertexes(1e-9);

    // Write the information into a txt file
    std::ofstream myFile;
    std::ofstream myDBP_Torque;
    if (output) {
        myFile.open(out_dir + "/results.txt", std::ios::trunc);
        myDBP_Torque.open(out_dir + "/DBP_Torque.txt", std::ios::trunc);
    }

    myFile << "time\t"
           << "x\t"
           << "y\t"
           << "z\t"
           << "vx\t"
           << "vy\t"
           << "vz\t"
           << "ax\t"
           << "ay\t"
           << "az\t"
           << "fx\t"
           << "fy\t"
           << "fz\t"
           << "tx\t"
           << "ty\t"
           << "tz\n";

    myDBP_Torque << "time\t"
                 << "fx\t"
                 << "tz\n";

    // ---------------------------------
    // Create the run-time visualization
    // ---------------------------------

#ifdef CHRONO_VSG
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sysMBS);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(1200, 600);
    vis->SetWindowTitle("Rassor Single Drum SCM Test");
    vis->AddCamera(ChVector3d(1.0, 2.5, 1.0));
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->SetShadows(true);
    vis->Initialize();
#else
    std::cerr << "VSG visualization system not available. Please rebuild with VSG support." << std::endl;
    return 1;
#endif

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    int render_frame = 0;

    double time = 0.0;
    int current_step = 0;

    ChTimer timer;
    timer.start();
    while (vis->Run()) {
        time = sysMBS.GetChTime();

        if (current_step % output_steps == 0) {
            std::cout << "time: " << time << std::endl;
            std::cout << "  drum position:         " << w_pos << std::endl;
            std::cout << "  drum linear velocity:  " << w_vel << std::endl;
            std::cout << "  drum angular velocity: " << angvel << std::endl;
            std::cout << "  drawbar pull:           " << force << std::endl;
            std::cout << "  drum torque:           " << torque << std::endl;
        }

        if (output && current_step % output_steps == 0) {
            std::cout << "-------- Output" << std::endl;
            myFile << time << "\t" << w_pos.x() << "\t" << w_pos.y() << "\t" << w_pos.z() << "\t" << w_vel.x() << "\t"
                   << w_vel.y() << "\t" << w_vel.z() << "\t" << angvel.x() << "\t" << angvel.y() << "\t" << angvel.z()
                   << "\t" << force.x() << "\t" << force.y() << "\t" << force.z() << "\t" << torque.x() << "\t"
                   << torque.y() << "\t" << torque.z() << "\n";
            myDBP_Torque << time << "\t" << force.x() << "\t" << torque.z() << "\n";

            // Save drum mesh to VTK
            static int counter = 0;
            std::string filename = out_dir + "/vtk/drum." + std::to_string(counter++) + ".vtk";
            WriteDrumVTK(filename, drum_mesh, drum->GetFrameRefToAbs());
        }

        if (time >= render_frame / render_fps) {
            auto& loc = drum->GetPos();
            vis->UpdateCamera(loc + ChVector3d(1.0, 2.5, 0.5), loc + ChVector3d(0, 0.25, -0.25));

            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            render_frame++;
        }

        // Get information about the drum
        reaction = actuator->GetReaction2();
        force = reaction.force;
        torque = motor->GetReaction1().torque;
        w_pos = drum->GetPos();
        w_vel = drum->GetPosDt();
        angvel = drum->GetAngVelLocal();

        // Synchronize subsystems
        terrain.Synchronize(time);
        drum->EmptyAccumulators();

        // Advance state
        terrain.Advance(dT);
        sysMBS.DoStepDynamics(dT);
        time += dT;
        current_step++;

        if (w_pos.x() + wheel_radius > terrain_length / 2.0f) {
            std::cout << "Drum has reached the end of the container" << std::endl;
            std::cout << "  drum position:         " << w_pos << std::endl;
            std::cout << "container position, " << bxDim / 2.0f << std::endl;
            break;
        }
    }
    timer.stop();
    std::cout << "Runtime: " << timer() << " seconds\n" << std::endl;
    std::cout << "Simulation time: " << time << std::endl;
    std::cout << "Simulation finished" << std::endl;

    if (output) {
        myFile.close();
        myDBP_Torque.close();
    }

    return 0;
}