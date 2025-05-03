// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban (modified)
// =============================================================================
//
// Gator wheeled vehicle on rigid terrain with blade attachment
//
// =============================================================================

#include <cstdio>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <thread>
#include <fstream>

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/core/ChRotation.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_models/vehicle/gator/Gator.h"

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// ===================================================================================================================

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Terrain dimensions
double terrain_length = 10*5;
double terrain_width = 4*5;

// Vehicle initial position
double vehicle_init_x = -2.0;
double vehicle_init_y = 0;
double vehicle_init_z = 0.5;

// Suspend vehicle
bool fix_chassis = false;

// Control command structure
struct ControlCommand {
    double start_time;
    double end_time;
    double blade_yaw;
    double blade_pitch;
    double blade_vertical;
    double throttle;
};

// ===================================================================================================================
// Forward declarations
std::vector<ControlCommand> control_schedule;

std::tuple<std::shared_ptr<gator::Gator>, std::shared_ptr<ChBody>, std::shared_ptr<ChLinkMotorRotationAngle>, std::shared_ptr<ChLinkMotorRotationAngle>, std::shared_ptr<ChLinkMotorLinearPosition>> CreateVehicle(const ChCoordsys<>& init_pos);
ControlCommand GetControlForTime(double time, const std::vector<ControlCommand>& schedule);
bool LoadControlScheduleFromFile(const std::string& filename, std::vector<ControlCommand>& schedule);

// ===================================================================================================================

int main(int argc, char* argv[]) {
    // ---------------- 
    // Problem settings
    // ----------------

    double target_speed = 1.0;
    double tend = 10;
    bool verbose = true;

    // Visualization settings
    bool render = true;                    // use run-time visualization
    double render_fps = 20;                // rendering FPS
    double control_step_size = 1/20.0;     // control FPS
    bool chase_cam = false;                // chase-cam or fixed camera

    // Parameters for the run - directly set values
    double terrain_height = 0.3;
    int exp_index = 1;

    // Load control schedule
    std::vector<ControlCommand> control_schedule;
    if (!LoadControlScheduleFromFile(vehicle::GetDataFile("ctrl_cmds/cmd"+std::to_string(exp_index)+".txt"), control_schedule)) {
        return 1;  // Exit if loading failed
    }

    // --------------
    // Create vehicle
    // --------------

    // Set vehicle initial heading angle
    auto init_heading = QuatFromAngleZ(0.0f);

    cout << "Create vehicle..." << endl;
    auto [vehicle, blade, motor_yaw, motor_pitch, motor_vertical] = CreateVehicle(ChCoordsys<>(ChVector3d(vehicle_init_x, vehicle_init_y, vehicle_init_z), init_heading));
    cout << "Finished creating vehicle"<< endl;
    auto sysMBS = vehicle->GetSystem();

    // ---------------------------------
    // Set solver and integrator for MBD
    // ---------------------------------

    double step_size = 3e-3;
    ChSolver::Type solver_type = ChSolver::Type::BARZILAIBORWEIN;
    ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;

    int num_threads_chrono = std::min(8, ChOMP::GetNumProcs());
    int num_threads_collision = 1;
    int num_threads_eigen = 1;
    int num_threads_pardiso = std::min(8, ChOMP::GetNumProcs());

    SetChronoSolver(*sysMBS, solver_type, integrator_type, num_threads_pardiso);
    sysMBS->SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);

    // Set collision system
    sysMBS->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // ----------------------
    // Create the rigid terrain
    // ----------------------

    RigidTerrain terrain(sysMBS);
    
    // Add a flat patch
    auto patch_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    patch_mat->SetFriction(0.8f);
    patch_mat->SetRestitution(0.01f);
    
    // Create the flat terrain patch
    auto patch = terrain.AddPatch(patch_mat, 
                                 ChCoordsys<>(ChVector3d(0, 0, 0)),  // Position of patch center
                                 terrain_length,                      // Length in X direction
                                 terrain_width,                       // Width in Y direction
                                 0.1,                                // Thickness
                                 true);                              // Visualization enabled
    
    // Create a texture for the terrain
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(GetChronoDataFile("textures/dirt.jpg"), 200, 200);

    // Initialize the terrain
    terrain.Initialize();
    
    // Create the driver
    ChWheeledVehicle& vehicle_ptr = vehicle->GetVehicle();
    cout << "Create driver..." << endl;
    ChDriver driver(vehicle->GetVehicle());
    driver.Initialize();

    // -----------------------------
    // Create run-time visualization
    // -----------------------------
    
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif

    std::shared_ptr<ChVehicleVisualSystem> vis;
    
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
                // Create the vehicle Irrlicht interface
                auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
                vis_irr->SetWindowTitle("Gator Rigid Terrain Demo");
                vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
                vis_irr->Initialize();
                vis_irr->AddLightDirectional();
                vis_irr->AddSkyBox();
                vis_irr->AddLogo();
                vis_irr->AttachVehicle(&vehicle_ptr);
                vis = vis_irr;
#endif
                break;
            }
            default:
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                // Create the vehicle VSG interface
                auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
                vis_vsg->SetWindowTitle("Gator Rigid Terrain Demo");
                vis_vsg->AttachVehicle(&vehicle_ptr);
                vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
                vis_vsg->SetWindowSize(ChVector2i(1200, 800));
                vis_vsg->SetWindowPosition(ChVector2i(100, 300));
                vis_vsg->SetUseSkyBox(true);
                vis_vsg->SetCameraAngleDeg(40);
                vis_vsg->SetLightIntensity(1.0f);
                vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
                vis_vsg->SetShadows(true);
                vis_vsg->Initialize();
                vis = vis_vsg;
#endif
                break;
            }
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;
    
    cout << "Start simulation..." << endl;
    
    // Enable realtime mode
    vehicle->GetVehicle().EnableRealtime(true);
    
    while (time < tend) {
        // Set current driver inputs
        ControlCommand current_cmd = GetControlForTime(time, control_schedule);
        driver.SetThrottle(current_cmd.throttle);
        driver.SetSteering(0.0);  // or set based on current_cmd if desired

        // Update blade controls
        motor_yaw->SetAngleFunction(chrono_types::make_shared<ChFunctionConst>(current_cmd.blade_yaw));
        motor_pitch->SetAngleFunction(chrono_types::make_shared<ChFunctionConst>(current_cmd.blade_pitch));
        motor_vertical->SetMotionFunction(chrono_types::make_shared<ChFunctionConst>(current_cmd.blade_vertical));

        auto driver_inputs = driver.GetInputs();

        // Run-time visualization
        if (render && vis && time >= render_frame / render_fps) {
            
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            
            render_frame++;
        }

        // Synchronize systems
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle->Synchronize(time, driver_inputs, terrain);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance system states
        driver.Advance(step_size);
        vehicle->Advance(step_size);
        terrain.Advance(step_size);
        if (vis)
            vis->Advance(step_size);

        time += step_size;
        sim_frame++;
    }
    
    cout << "Simulation completed"<< endl;
    
    return 0;
}

// ===================================================================================================================

std::tuple<std::shared_ptr<gator::Gator>, std::shared_ptr<ChBody>, std::shared_ptr<ChLinkMotorRotationAngle>, std::shared_ptr<ChLinkMotorRotationAngle>, std::shared_ptr<ChLinkMotorLinearPosition>> CreateVehicle(const ChCoordsys<>& init_pos) {
    auto gator = chrono_types::make_shared<gator::Gator>();
    gator->SetContactMethod(ChContactMethod::SMC);
    gator->SetChassisCollisionType(CollisionType::NONE);
    gator->SetChassisFixed(false);
    gator->SetInitPosition(init_pos);
    gator->SetBrakeType(BrakeType::SIMPLE);
    gator->SetTireType(TireModelType::RIGID_MESH);
    gator->SetTireStepSize(1e-3);
    gator->SetAerodynamicDrag(0.5, 5.0, 1.2);
    gator->EnableBrakeLocking(true);
    gator->Initialize();

    gator->SetChassisVisualizationType(VisualizationType::MESH);
    gator->SetSuspensionVisualizationType(VisualizationType::MESH);
    gator->SetSteeringVisualizationType(VisualizationType::MESH);
    gator->SetWheelVisualizationType(VisualizationType::MESH);
    gator->SetTireVisualizationType(VisualizationType::MESH);

    auto contact_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto blade = chrono_types::make_shared<ChBodyEasyMesh>(vehicle::GetDataFile("gator/gator_frontblade.obj"), 1000, true, true, false);
    auto offsetpos = ChVector3d(1.75, 0, -0.3);

    blade->SetPos(gator->GetChassisBody()->TransformPointLocalToParent(offsetpos));
    blade->SetRot(gator->GetChassisBody()->GetRot() * Q_ROTATE_Y_TO_X * QuatFromAngleX(-CH_PI_2));
    blade->SetMass(0.1);
    blade->SetFixed(false);
    gator->GetSystem()->AddBody(blade);

    auto vir_yaw_body = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 1000, true, false, contact_mat);
    vir_yaw_body->SetPos(gator->GetChassisBody()->TransformPointLocalToParent(offsetpos));
    gator->GetSystem()->AddBody(vir_yaw_body);
    auto vir_pitch_body = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 1000, true, false, contact_mat);
    vir_pitch_body->SetPos(gator->GetChassisBody()->TransformPointLocalToParent(offsetpos));
    gator->GetSystem()->AddBody(vir_pitch_body);
    auto vir_vertical_body = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 1000, true, false, contact_mat);

    auto motor_yaw = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    auto motor_pitch = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    auto motor_vertical = chrono_types::make_shared<ChLinkMotorLinearPosition>();

    motor_yaw->Initialize(vir_yaw_body, blade, ChFrame<>(vir_yaw_body->GetPos()));
    motor_pitch->Initialize(vir_yaw_body, vir_pitch_body, ChFrame<>(vir_pitch_body->GetPos(), Q_ROTATE_Y_TO_Z));
    motor_vertical->Initialize(gator->GetChassisBody(), vir_pitch_body, ChFrame<>(vir_vertical_body->GetPos()));

    gator->GetSystem()->Add(motor_yaw);
    gator->GetSystem()->Add(motor_pitch);
    gator->GetSystem()->Add(motor_vertical);

    return std::make_tuple(gator, blade, motor_yaw, motor_pitch, motor_vertical);
}

ControlCommand GetControlForTime(double time, const std::vector<ControlCommand>& schedule) {
    for (const auto& cmd : schedule) {
        if (time >= cmd.start_time && time < cmd.end_time) {
            return cmd;
        }
    }
    // Default fallback
    return schedule.back();
}

bool LoadControlScheduleFromFile(const std::string& filename, std::vector<ControlCommand>& schedule) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Failed to open control command file: " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::string token;
        std::vector<double> values;

        while (std::getline(iss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (...) {
                std::cerr << "Error parsing value in line: " << line << std::endl;
                return false;
            }
        }

        if (values.size() != 6) {
            std::cerr << "Each line must have 6 values. Found " << values.size() << " in: " << line << std::endl;
            return false;
        }

        ControlCommand cmd = {values[0], values[1], values[2], values[3], values[4], values[5]};
        schedule.push_back(cmd);
    }

    std::cout << "Loaded " << schedule.size() << " control commands from file." << std::endl;
    return true;
} 