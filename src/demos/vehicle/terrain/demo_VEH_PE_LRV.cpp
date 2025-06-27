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
// Authors: Radu Serban
// =============================================================================
//
// Gator acceleration test.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_models/vehicle/gator/Gator.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/powertrain/ChEngineSimple.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"

#include "chrono/physics/ChLinkMate.h"

#include "chrono_powerelectronics/circuits/ChElectronicMotor.h"


#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <iomanip>

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::gator;
using namespace chrono::powerelectronics;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
//ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Initial vehicle location and orientation (m)
// ChVector3d initLoc(-0, 0, 4);
ChQuaternion<> initRot(1, 0, 0, 0);

// Brake type (SIMPLE or SHAFTS)
BrakeType brake_type = BrakeType::SHAFTS;

// Terrain slope (radians)
double slope = 20 * CH_DEG_TO_RAD;


// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
//VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
//VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
//VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
//VisualizationType wheel_vis_type = VisualizationType::NONE;
//VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Simulation step sizes
double step_size = 3.3e-3;

// End time (used only if no run-time visualization)
double t_end = 20;
std::shared_ptr<WheeledVehicle> CreateVehicle(ChSystem& sys, const ChCoordsys<>& init_pos, std::string vehicle_type, double mass1, double mass2);

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : m_kp(kp), m_ki(ki), m_kd(kd), m_integral(0), m_prev_error(0) {}

    double Compute(double setpoint, double measured_value, double windup, double dt, double kp_fac) {
        double error = setpoint - measured_value;
        m_integral += error * dt;
        if(m_integral > windup) { // Anti-windup
            m_integral = windup;
        }
        if(m_integral < -windup) {
            m_integral = -windup;
        }
        double derivative = (error - m_prev_error) / dt;
        m_prev_error = error;
        return (kp_fac*m_kp) * error + m_ki * m_integral + m_kd * derivative;
    }

    void Reset() {
        m_integral = 0;
        m_prev_error = 0;
    }

private:
    double m_kp;
    double m_ki;
    double m_kd;
    double m_dt;
    double m_integral;
    double m_prev_error;
};

class HubMotor {

private:
    std::shared_ptr<ChElectronicMotor> motor;
    double PWM_MAX = 5.2;
    double PWM_MIN = 3.5;
    double PWM = 0.0;
    
    double last_torque = 0.0;

    double step_ctr = 0.0;

public:
    HubMotor(std::shared_ptr<ChBody> spindle) {
        motor = std::make_shared<ChElectronicMotor>(6e-3);

        double kt_motor = 0.1066;  // Motor torque constant [Nm/A]
        double ke_motor = -0.107;  // Motor back EMF constant [V/(rad/s)]
        // double ke_motor = -0.100; // Motor back EMF constant [V/(rad/s)]

        motor->InitParams(kt_motor,ke_motor);

        // double L_coil = 0.002;  // Coil inductance [H]
        // double R_coil = 69.1;  // Coil resistance [Ohm]

        double L_coil = 3e-4;  // Coil inductance [H]
        double R_coil = 0.27;  // Coil resistance [Ohm]

        motor->InitMotorModel(L_coil,R_coil);

        // Initialize the motor
        motor->Initialize();

    }
    /// Return the output engine torque on the motorshaft (N-m)
    double GetOutputMotorshaftTorque() {
        return last_torque;
        // return 100.;
    }
    
    void Advance(double step) {
        step_ctr += step;
        if (step_ctr >= 0.05) {
            motor->Advance(step_ctr);
            this->last_torque = motor->GetOutputTorque().Length();
            step_ctr = 0.0;
        }
    }

    void Synchronize(double time, const DriverInputs &driver_inputs, double motorshaft_speed) {

        // Set the PWM frequency (in Hz)
        const double PWM_FREQUENCY = 1000.0;  // 1 kHz, can be adjusted depending on system requirements
        const double PWM_PERIOD = 1.0 / PWM_FREQUENCY;  // Period in seconds

        // Calculate the duty cycle based on throttle input (assuming throttle is between 0 and 1)
        double duty_cycle = driver_inputs.m_throttle;  // Duty cycle is directly proportional to throttle input (0.0 to 1.0)

        // Determine the current phase in the PWM period (using modulo to wrap time)
        double pwm_time_in_period = fmod(time, PWM_PERIOD);

        // Calculate the ON time for the current period based on the duty cycle
        double on_time = duty_cycle * PWM_PERIOD;

        // Set the PWM signal based on whether we are within the ON time or OFF time
        // if (pwm_time_in_period < on_time) {
        //     // PWM is ON for this part of the cycle
        //     motor->SetPWM(48.0);  // Apply the calculated PWM voltage signal
        // } else {
        //     // PWM is OFF for this part of the cycle
        //     motor->SetPWM(0.0);  // No voltage applied when PWM is off
        // }

        motor->SetPWM(48.0*duty_cycle);  // Apply the calculated PWM voltage signal

        // if(duty_cycle == 0) {
        //     motor->SetPWM(0.0);
        // } else {
        //     motor->SetPWM(48.0);  // Apply the calculated PWM voltage signal
        // }
        // Set the motor shaft velocity

        motor->SetShaftAngVel(motorshaft_speed);

        // Optional: Debug output
        // std::cout << "Time: " << time << " Throttle: " << driver_inputs.m_throttle 
        //           << " Shaft Speed: " << motorshaft_speed 
        //           << " PWM State: " << (pwm_time_in_period < on_time ? "ON" : "OFF") << std::endl;
    }

};

// =============================================================================
void InitializeTerrain(SCMTerrain& terrain, const std::string& bmp_file, const std::string& range_file, double& width, double& height, bool flag) {
    // Open BMP file
    std::ifstream bmp_stream(bmp_file, std::ios::binary);
    if (!bmp_stream.is_open()) {
        throw std::runtime_error("Failed to open BMP file: " + bmp_file);
    }

    // Read BMP header to get dimensions
    bmp_stream.seekg(18); // BMP width and height start at offset 18
    int32_t bmp_width = 0;
    int32_t bmp_height = 0;
    bmp_stream.read(reinterpret_cast<char*>(&bmp_width), 4);
    bmp_stream.read(reinterpret_cast<char*>(&bmp_height), 4);
    bmp_stream.close();

    if (bmp_width <= 0 || bmp_height <= 0) {
        throw std::runtime_error("Invalid BMP dimensions in file: " + bmp_file);
    }

    // Calculate terrain dimensions
    double terrain_width = bmp_width ;
    double terrain_length = bmp_height ;


    if(flag) {
        terrain_length *= 1.5;
        terrain_width *= 1.5;
    } else {
        terrain_length *= 0.2;
        terrain_width *= 0.2;
    }

    // double terrain_width = bmp_width * 1.5;
    // double terrain_length = bmp_height * 1.5;

    // Read the range file for height information
    double max_height = 0.0;
    std::ifstream range_stream(range_file);
    if (!range_stream.is_open()) {
        throw std::runtime_error("Failed to open range file: " + range_file);
    }
    range_stream >> max_height;
    range_stream.close();

    if (max_height <= 0) {
        throw std::runtime_error("Invalid max height value in range file: " + range_file);
    }

    width = terrain_width;
    height = terrain_length;
    
    ChQuaterniond quat;
    quat.SetFromAngleZ(3.14);
    terrain.SetPlane(ChCoordsys<>(ChVector3d(0.,0.,0.), quat));

    // Initialize terrain with calculated parameters
    terrain.Initialize(bmp_file, terrain_width, terrain_length, 0.0, max_height, 0.075);

    terrain.SetPlotType(SCMTerrain::DataPlotType::PLOT_SINKAGE, 0,0.05);

}

std::shared_ptr<RigidTerrain::Patch> InitializeRigidTerrain(RigidTerrain& terrain, const std::string& bmp_file, const std::string& range_file, double& width, double& height) {
    // Open BMP file
    std::ifstream bmp_stream(bmp_file, std::ios::binary);
    if (!bmp_stream.is_open()) {
        throw std::runtime_error("Failed to open BMP file: " + bmp_file);
    }

    // Read BMP header to get dimensions
    bmp_stream.seekg(18); // BMP width and height start at offset 18
    int32_t bmp_width = 0;
    int32_t bmp_height = 0;
    bmp_stream.read(reinterpret_cast<char*>(&bmp_width), 4);
    bmp_stream.read(reinterpret_cast<char*>(&bmp_height), 4);
    bmp_stream.close();

    if (bmp_width <= 0 || bmp_height <= 0) {
        throw std::runtime_error("Invalid BMP dimensions in file: " + bmp_file);
    }

    // Calculate terrain dimensions
    double terrain_width = bmp_width * 0.2;
    double terrain_length = bmp_height * 0.2;

    // Read the range file for height information
    double max_height = 0.0;
    std::ifstream range_stream(range_file);
    if (!range_stream.is_open()) {
        throw std::runtime_error("Failed to open range file: " + range_file);
    }
    range_stream >> max_height;
    range_stream.close();

    if (max_height <= 0) {
        throw std::runtime_error("Invalid max height value in range file: " + range_file);
    }

    width = terrain_width;
    height = terrain_length;
    
    ChQuaterniond quat;
    quat.SetFromAngleZ(3.14);
    // terrain.SetPlane(ChCoordsys<>(ChVector3d(0.,0.,0.), quat));

    // Initialize terrain with calculated parameters
    // terrain.Initialize(bmp_file, terrain_width, terrain_length, 0.0, max_height, 0.05);
    terrain.Initialize();

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::NSC);
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);

    return terrain.AddPatch(patch_mat,ChCoordsys<>(ChVector3d(0.,0.,0.), quat), bmp_file, terrain_width, terrain_length, 0.0, max_height);
}


std::vector<chrono::ChVector3d> ReadTrajectoryPoints(const std::string& filepath, double width, double height, ChTerrain& terrain) {
    std::vector<chrono::ChVector3d> points;

    std::ifstream infile(filepath);
    if (!infile.is_open()) {
        throw std::runtime_error("Failed to open trajectory points file: " + filepath);
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::string x_str, y_str;

        if (std::getline(iss, x_str, ',') && std::getline(iss, y_str, ',')) {
            try {
                double x = std::stod(x_str)*0.2 - (0.5*width);
                double y = std::stod(y_str)*0.2 - (0.5*height);
                double z = terrain.GetHeight(ChVector3d(x, y,0)) + 1.0;

                points.emplace_back(x, y, z); // Assume Z = 0 for 2D path
            } catch (const std::exception& e) {
                throw std::runtime_error("Failed to parse values in line: " + line);
            }
        } else {
            throw std::runtime_error("Malformed line in trajectory points file: " + line);
        }
    }

    infile.close();

    return points;
}

// Function to visualize trajectory in Chrono
void VisualizeTrajectory(chrono::ChSystem& system, const std::vector<chrono::ChVector3d>& points) {
    for (const auto& point : points) {
        std::cout << "Added " << point << std::endl;

        // Create a sphere shape
        auto sphere_shape = chrono_types::make_shared<chrono::ChVisualShapeSphere>(0.25);

        // Create a static body to hold the sphere
        auto sphere_body = chrono_types::make_shared<chrono::ChBody>();
        sphere_body->SetPos(point);
        sphere_body->SetFixed(true); // Spheres are static
        sphere_body->AddVisualShape(sphere_shape);

        // Add the sphere to the system
        system.Add(sphere_body);
    }
}



std::shared_ptr<WheeledTrailer> CreateTrailer(ChSystem& sys, std::shared_ptr<ChChassis> chassis, const ChCoordsys<>& init_pos, double mass1) {

    std::shared_ptr<WheeledTrailer> trailer;
    std::string trailer_json = "LRV_Wagon/Polaris.json";
    std::string tire_json = "Polaris/Polaris_RigidTire.json";


    trailer = chrono_types::make_shared<WheeledTrailer>(&sys, vehicle::GetDataFile(trailer_json));
    // trailer->Initialize(ChVector3d(0,0,0));
    trailer->Initialize(chassis);
    trailer->GetChassis()->SetFixed(false);
    trailer->SetChassisVisualizationType(VisualizationType::MESH);
    trailer->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);

    // Create and initialize the tires
    for (auto& axle : trailer->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
            trailer->InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    ChVector3d relative_offset(-2.0,0.0,0.5); // Relative position offset
    ChVector3d rotated_offset = trailer->GetChassis()->GetBody()->GetRot().Rotate(relative_offset);

    // Create a cube-shaped body
    auto cube_body = chrono_types::make_shared<ChBody>();
    cube_body->SetPos(init_pos.pos + rotated_offset); // Position the cube above the trailer's chassis
    cube_body->SetMass(mass1);               // Set cube mass
    cube_body->SetInertiaXX(ChVector3d(10, 10, 10)); // Set cube inertia

    // Add a visual shape (cube)
    auto cube_visual = chrono_types::make_shared<ChVisualShapeBox>();
    cube_visual->GetGeometry().SetLengths(ChVector3d(0.5, 0.5, 0.5)); // Cube dimensions (length, width, height)
    cube_body->AddVisualShape(cube_visual);

    // Attach cube to the trailer chassis
    cube_body->SetFixed(false); // Allow the cube to move
    // cube_body->SetCollide(true);    // Enable collision


    // Add the cube to the system
    sys.Add(cube_body);

    // Optionally, add a joint to connect the cube to the trailer
    auto revolute_joint = chrono_types::make_shared<ChLinkMateFix>();
    revolute_joint->Initialize(cube_body, trailer->GetChassis()->GetBody(), ChFrame<>(ChVector3d(0,0,0), QUNIT));
    sys.AddLink(revolute_joint);

    return trailer;

}

int main(int argc, char* argv[]) {
    std::string chunk = "33";

    // PID coefficients for steering (tune these values)
    double kp_steer = 0.4;
    double ki_steer = 0.05;
    double kd_steer = 0.3;

    // PID coefficients for speed (tune these values)
    double kp_speed = 0.4;
    double ki_speed = 0.3;
    double kd_speed = 0.0;
    
    struct GainSet {
        double kp_steer;
        double ki_steer;
        double kd_steer;
        double kp_speed;
        double ki_speed;
        double kd_speed;
    };


    GainSet g0 = {0.5, 0.05, 0.5, 0.25, 0.2, 0.0};
    GainSet g1 = {0.52, 0.06, 0.5, 0.2, 0.3, 0.0};
    GainSet g2 = {0.49, 0.08, 0.5, 0.22, 0.24, 0.0};

    double mass1 = 150;
    double mass2 = 200;
    double mass_payload = 50;

    struct SoilSet {
        double kc;
        double kphi;
        double c;
        double friction_angle;
    };


    SoilSet s0 = { 8.14e5, 1.37e3, 0.03, 30 };
    SoilSet s1 = { 4e5, 1.37e3, 0.01, 20 };
    SoilSet s2 = { 9e5, 1.37e3, 0.03, 30 };

    double kc = 8.14e5;
    double kphi = 1.37e3;
    double c = 0.03;
    double fangle = 30;

    // Set speed (m/s)
    double target_speed = 1.0;

    std::string output_dir;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--chunk" && i + 1 < argc) {
            chunk = argv[++i];
        } else if (arg == "--gain-set" && i + 1 < argc) {
             auto gain_set_name = std::atof(argv[++i]);
             GainSet set;
             if(gain_set_name == 0) {
                 set = g0;
             } else if (gain_set_name == 1) {
                 set = g1;
             } else if (gain_set_name == 2) {
                 set = g2;
             }
             kp_steer = set.kp_steer;
             ki_steer = set.ki_steer;
             kd_steer = set.kd_steer;
             kp_speed = set.kp_speed;
             ki_speed = set.ki_speed;
             kd_speed = set.kd_speed;
        } else if (arg == "--soil-set" && i+1 < argc) {
             auto soil_set_name = std::atof(argv[++i]);
             SoilSet set;
             if(soil_set_name == 0) {
                 set = s0;
             } else if (soil_set_name == 1) {
                set = s1;
             } else if (soil_set_name == 2) {
                set = s2;
             }
             kc = set.kc;
             kphi = set.kphi;
             c = set.c;
             fangle = set.friction_angle;

        } else if (arg == "--mass1" && i + 1 < argc) {
            mass1 = std::atof(argv[++i]);
        } else if (arg == "--mass2" && i + 1 < argc) {
            mass2 = std::atof(argv[++i]);
        } else if (arg == "--mass_payload" && i + 1 < argc) {
            mass_payload = std::atof(argv[++i]);
        } else if (arg == "--target_speed" && i + 1 < argc) {
            target_speed = std::atof(argv[++i]);
        } else if (arg == "--output_dir" && i + 1 < argc) {
            output_dir = argv[++i];
        } else {
            std::cerr << "Unknown argument or missing value: " << arg << std::endl;
            return 1;
        }
    }

    std::cout << "Chrono version: " << CHRONO_VERSION << std::endl;

    // Create the physical system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -1.62)); // Moon gravity
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);
    sys.SetNumThreads(std::min(8, ChOMP::GetNumProcs()));

    // Create the terrain
    SCMTerrain terrain(&sys);

    // Set soil parameters for SCM terrain
    terrain.SetSoilParameters(kc,    // Bekker Kphi
                              kphi,        // Bekker Kc
                              1.1,      // Bekker n exponent
                              c,     // Mohr cohesive limit (Pa)
                              fangle,       // Mohr friction limit (degrees)
                              0.01,     // Janosi shear coefficient (m)
                              2e8,      // Elastic stiffness (Pa/m), before plastic yield
                              1e4       // Damping (Pa s/m), proportional to negative vertical speed
    );

    terrain.EnableBulldozing(true);
    terrain.SetBulldozingParameters(
        55,  // angle of friction for erosion of displaced material at the border of the rut
        1,   // displaced material vs downward pressed material.
        3,   // number of erosion refinements per timestep
        3);  // number of concentric vertex selections subject to erosion

    // RigidTerrain terrain(&sys);

    double width, height;

    bool flag = false;
    if(chunk == "99") {
        flag = true;
    }

    std::string heightmap_file = vehicle::GetDataFile("terrain/height_maps/chunk_"+chunk+".bmp");
    // std::string heightmap_file = vehicle::GetDataFile("terrain/200x50_segment_heightmap.bmp");
    
    InitializeTerrain(terrain, heightmap_file,
                      vehicle::GetDataFile("terrain/height_maps/chunk_"+chunk+"_range.txt"), width, height, flag);

    // auto patch = InitializeRigidTerrain(terrain, heightmap_file,
    //                   vehicle::GetDataFile("terrain/chunks2/chunk_"+chunk+"_range.txt"), width, height);
    // patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 100, 50);
    
    // Read trajectory points from file
    std::string trajectory_file = vehicle::GetDataFile("terrain/height_maps/chunk_"+(chunk)+"_trajectory_points.txt");
    auto points = ReadTrajectoryPoints(trajectory_file, width, height, terrain);

    // Ensure there are at least two points
    if (points.size() < 2) {
        throw std::runtime_error("Not enough trajectory points to initialize vehicle orientation.");
    }

    // Extract the first and second points
    ChVector3d first_point = points[0];
    ChVector3d second_point = points[1];

    // Compute the direction vector and yaw angle
    ChVector3d direction = second_point - first_point;
    direction.z() = 0;  // Project onto the X-Y plane
    direction.Normalize();
    double yaw = std::atan2(direction.y(), direction.x());

    // Set the initial position and rotation
    ChVector3d initLoc = first_point;
    ChQuaternion<> initRot;
    initRot.SetFromAngleZ(yaw);

    // Create the vehicle

    auto vehicle = CreateVehicle(sys, ChCoordsys<>(initLoc, initRot), "veh_name",mass1,mass2);

    auto wagon = CreateTrailer(sys, vehicle->GetChassis(), ChCoordsys<>(initLoc, initRot),mass_payload);

    std::shared_ptr<ChBody> spindle1 = vehicle->GetWheel(0, VehicleSide::LEFT)->GetSpindle();
    std::shared_ptr<ChBody> spindle2 = vehicle->GetWheel(0, VehicleSide::RIGHT)->GetSpindle();
    std::shared_ptr<ChBody> spindle3 = vehicle->GetWheel(1, VehicleSide::LEFT)->GetSpindle();
    std::shared_ptr<ChBody> spindle4 = vehicle->GetWheel(1, VehicleSide::RIGHT)->GetSpindle();

    auto hm1 = std::make_shared<HubMotor>(spindle1);
    auto hm2 = std::make_shared<HubMotor>(spindle2);
    auto hm3 = std::make_shared<HubMotor>(spindle3);
    auto hm4 = std::make_shared<HubMotor>(spindle4);

    // Add accumulators for each spindle
    unsigned int sp1_acccu = spindle1->AddAccumulator();
    unsigned int sp2_acccu = spindle2->AddAccumulator();
    unsigned int sp3_acccu = spindle3->AddAccumulator();
    unsigned int sp4_acccu = spindle4->AddAccumulator();

    // Prismatic joint
    // auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    // prismatic1->Initialize(wagon->GetChassisBody(), vehicle->GetChassisBody(), ChFrame<>(vehicle->GetChassis()->GetPos(), QuatFromAngleY(3.14159/2.)));
    // prismatic1->SetName("prismatic_chassis_ground");
    // sys.AddLink(prismatic1);


    // Adjust the terrain moving patch
    // terrain.AddMovingPatch(vehicle->GetChassisBody(), ChVector3d(0, 0, 0), ChVector3d(25.0, 25.0, 1.0));

    // Visualize the trajectory
    VisualizeTrajectory(sys, points);


    // ChContactMaterialData minfo;
    // minfo.mu = 0.9f;
    // minfo.cr = 0.01f;
    // minfo.Y = 2e7f;
    // auto patch_mat = minfo.CreateMaterial(ChContactMethod::NSC);
    // patch_mat->SetFriction(0.9f);
    // patch_mat->SetRestitution(0.01f);

    // auto patch1 = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(-25, 0, 0), QUNIT), heightmap_file, 200.0, 100.0);
    // patch1->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 100, 50);


    // Create PID controllers
    PIDController pid_steering(kp_steer, ki_steer, kd_steer);
    PIDController pid_speed(kp_speed, ki_speed, kd_speed);

    // ------------------------------------------------------------------------------
    // Create the vehicle run-time visualization interface and the interactive driver
    // ------------------------------------------------------------------------------
#ifdef CHRONO_IRRLICHT
    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Project Chrono: Gator Acceleration Test");
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 5.0, 0.5);                         // Chase distance
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(vehicle.get());
    // vis->AddTerrain(terrain);
    // If you have other visual elements like trajectory spheres, add them here
#endif

    // ---------------
    // Simulation loop
    // ---------------

    // Initialize simulation frame counters
    int step_number = 0;
    // vehicle->EnableRealtime(true);
    size_t current_waypoint_index = 1;
    ChVector3d current_waypoint = points[current_waypoint_index];
    double waypoint_threshold = 3.0;  // Threshold distance to switch to next waypoint
    double prev_steering_input = 0.0;
    double prev_throttle_input = 0.0;
    double heading_error = 0.0;
    double settling_period = 5.0;
    DriverInputs driver_inputs;
    DriverInputs driver_inputs_hm;

    auto last_vis_update_time = std::chrono::steady_clock::now();
    double vis_update_rate = 5.0; // 25Hz
    double vis_update_time = 1.0 / vis_update_rate;

    std::cout << "Init" << std::endl;
    std::filesystem::create_directories(output_dir);
    std::cout << "FinInit" << std::endl;
    std::ofstream csv_file(output_dir+"/simulation_data.csv");
    csv_file << "time,current_waypoint_index,prev_throttle_input,prev_steering_input,sr1,sr2,sr3,sr4,sa1,sa2,sa3,sa4,engine_torque1,engine_torque2,engine_torque3,engine_torque4,x,y,z,roll,pitch,yaw,vx,vy,vz,waypoints_missed,heading_error\n";
            //   << time << ","
            //     << current_waypoint_index << ","
            //     << prev_throttle_input << ","
            //     << sr1 << ","
            //     << sr2 << ","
            //     << sr3 << ","
            //     << sr4 << ","
            //     << sa1 << ","
            //     << sa2 << ","
            //     << sa3 << ","
            //     << sa4 << ","
            //     << engine_torque1 << ","
            //     << engine_torque2 << ","
            //     << engine_torque3 << ","
            //     << engine_torque4 << ","
            //     << vehicle_pos.x() << ","
            //     << vehicle_pos.y() << ","
            //     << vehicle_pos.z() << ","
            //     << roll << ","
            //     << pitch << ","
            //     << yaw << ","
            //     << vehicle_vel.x() << ","
            //     << vehicle_vel.y() << ","
            //     << vehicle_vel.z() << ","
            //     << waypoints_missed << ","
            //     << heading_error << "\n";
                
    double last_time = sys.GetChTime();

    int waypoints_missed = 0;

    while (true) {
        double time = sys.GetChTime();

        if (time >= 800) {
            std::cout << "Maximum simulation time reached. Terminating simulation." << std::endl;
            // Log termination reason in CSV
            csv_file << std::fixed << std::setprecision(6)
                     << time << ",,,,,,,,,,,,,,,,,,,,,," << waypoints_missed << ",," << "Max Time Reached\n";
            break;
        }

        // Collect vehicle state data
        ChVector3d vehicle_pos = vehicle->GetPos();
        ChQuaternion<> vehicle_rot = vehicle->GetRot();
        ChVector3d vehicle_vel = vehicle->GetChassisBody()->GetPosDt();

        // Extract longitudinal and lateral slip ratio (assuming the first axle and first wheel)
        double sr1 = vehicle->GetAxle(0)->GetWheel(VehicleSide::LEFT)->GetTire()->GetLongitudinalSlip();
        double sr2 = vehicle->GetAxle(0)->GetWheel(VehicleSide::RIGHT)->GetTire()->GetLongitudinalSlip();
        double sr3 = vehicle->GetAxle(1)->GetWheel(VehicleSide::LEFT)->GetTire()->GetLongitudinalSlip();
        double sr4 = vehicle->GetAxle(1)->GetWheel(VehicleSide::RIGHT)->GetTire()->GetLongitudinalSlip();
        
        double sa1 = vehicle->GetAxle(0)->GetWheel(VehicleSide::LEFT)->GetTire()->GetSlipAngle();
        double sa2 = vehicle->GetAxle(0)->GetWheel(VehicleSide::RIGHT)->GetTire()->GetSlipAngle();
        double sa3 = vehicle->GetAxle(1)->GetWheel(VehicleSide::LEFT)->GetTire()->GetSlipAngle();
        double sa4 = vehicle->GetAxle(1)->GetWheel(VehicleSide::RIGHT)->GetTire()->GetSlipAngle();

        // Engine torque
        // double engine_torque = vehicle->GetPowertrain()->GetMotorTorque();

        double engine_torque1 = hm1->GetOutputMotorshaftTorque();
        double engine_torque2 = hm1->GetOutputMotorshaftTorque();
        double engine_torque3 = hm1->GetOutputMotorshaftTorque();
        double engine_torque4 = hm1->GetOutputMotorshaftTorque();

        // Vehicle orientation in Euler angles (roll, pitch, yaw)
        ChVector3d euler_angles = vehicle_rot.GetRotVec();
        double roll = euler_angles.x();
        double pitch = euler_angles.y();
        double yaw = euler_angles.z();

        // Progress metrics
        // double waypoints_completed_percentage = ((current_waypoint_index - 1) / (double)points.size() - 1) * 100;
        // bool last_waypoint_reached = (current_waypoint_index == points.size() - 1);
        
        // Write to CSV
        csv_file << std::fixed << std::setprecision(6)
                << time << ","
                << current_waypoint_index << ","
                << prev_throttle_input << ","
                << prev_steering_input << ","
                << sr1 << ","
                << sr2 << ","
                << sr3 << ","
                << sr4 << ","
                << sa1 << ","
                << sa2 << ","
                << sa3 << ","
                << sa4 << ","
                << engine_torque1 << ","
                << engine_torque2 << ","
                << engine_torque3 << ","
                << engine_torque4 << ","
                << vehicle_pos.x() << ","
                << vehicle_pos.y() << ","
                << vehicle_pos.z() << ","
                << roll << ","
                << pitch << ","
                << yaw << ","
                << vehicle_vel.x() << ","
                << vehicle_vel.y() << ","
                << vehicle_vel.z() << ","
                << waypoints_missed << ","
                << heading_error << "\n";
                
        if (time < settling_period) {
            // Hold vehicle stationary
            driver_inputs.m_steering = 0.0;
            driver_inputs.m_throttle = 0.0;
            driver_inputs.m_braking = 1.0;  // Apply full brakes
        } else {
                
            // Get vehicle state
            ChVector3d vehicle_pos = vehicle->GetPos();
            ChQuaternion<> vehicle_rot = vehicle->GetRot();
            ChVector3d vehicle_dir = vehicle_rot.GetAxisZ();  // Assuming Z-axis points forward


            // Check if we have reached the current waypoint
            double distance_to_waypoint = (current_waypoint - vehicle_pos).Length();
            if (distance_to_waypoint < waypoint_threshold) {
                // Move to the next waypoint
                current_waypoint_index++;
                if (current_waypoint_index >= points.size()) {
                    // Reached the final waypoint
                    break;
                }
                current_waypoint = points[current_waypoint_index];
                pid_steering.Reset();
                pid_speed.Reset();
            }

            if (heading_error < -1.5 || heading_error > 1.5) {
                std::cout << "Missed waypoint" << std::endl;
                current_waypoint_index++;
                waypoints_missed++;
                if (current_waypoint_index >= points.size()) {
                    // Reached the final waypoint
                    break;
                }
                current_waypoint = points[current_waypoint_index];
                pid_steering.Reset();
                pid_speed.Reset();
            }

            // Compute steering angle using PID controller
            ChVector3d vec_to_waypoint = current_waypoint - vehicle_pos;
            double target_heading = std::atan2(vec_to_waypoint.y(), vec_to_waypoint.x());
            // double vehicle_heading = std::atan2(vehicle_dir.y(), vehicle_dir.x());


            ChVector3d euler_angles = vehicle_rot.GetRotVec();
            double vehicle_heading = euler_angles.z();
            double vehicle_pitch = euler_angles.x();
            
            heading_error = target_heading - vehicle_heading;

            // Normalize the heading error to the range [-pi, pi]
            while (heading_error > 3.14)
                heading_error -= 2 * 3.14;
            while (heading_error < -3.14)
                heading_error += 2 * 3.14;


            double dt = time - last_time;

            std::cout << vehicle_pitch << std::endl;
            // double clamped_pitch = fmax(0.0, vehicle_pitch);
            // double reduction = 1.0 / (1.0 + 0.2 * clamped_pitch);

            double steering_input = pid_steering.Compute(0.0, heading_error, 2, dt, 1.0);

            // Limit steering input to [-1, 1]
            steering_input = -std::clamp(steering_input, -0.5, 0.5);

            // Compute throttle input using PID controller to maintain target speed
            double current_speed = vehicle->GetSpeed();


            // Get the circle that holds both at its rim
            double theta = vehicle_heading;
            double x1 = current_waypoint.x();
            double y1 = current_waypoint.y();

            double x2 = vehicle_pos.x();
            double y2 = vehicle_pos.y();

            double rq1 = pow(x1 - x2,2) + pow(y1 - y2,2);
            double rq2 = 2 * (((x1-x2) * sin(theta)) - ((y1-y2)* cos(theta)));

            double R = rq1/rq2;

            double speed_setpoint;
            if(fabs(R) < 25.0) {
                speed_setpoint = fmax(fmin(fabs(R)*0.2,target_speed),0.5);
            } else {
                speed_setpoint = target_speed;
            }
            // End circle calc

            double speed_error = speed_setpoint - current_speed;
            double throttle_input = pid_speed.Compute(0.0, -speed_error, 1.5, dt,1.0);


            // double alpha_throttle = 0.01; // Adjust this value as needed
            // throttle_input = alpha_throttle * throttle_input + (1.0 - alpha_throttle) * prev_throttle_input;
            // prev_throttle_input = throttle_input;

            // Limit throttle input to [0, 1]
            throttle_input = std::clamp(throttle_input, 0.0, 1.0);

            if(vehicle->GetSpeed() < 1.0) {
                throttle_input = fmin(throttle_input,0.4);
            }

            if(vehicle->GetSpeed() < 2.0) {
                throttle_input = fmin(throttle_input,0.5);
            }


            if(speed_setpoint < current_speed && !isnan(speed_setpoint)) {
                driver_inputs.m_braking = 0.5*(current_speed - speed_setpoint);
                std::cout << driver_inputs.m_braking << std::endl;
            } else {
                driver_inputs.m_braking = 0.0;
            }

            double alpha = 0.1; // Adjust this value as needed
            steering_input = alpha * steering_input + (1.0 - alpha) * prev_steering_input;
            prev_steering_input = steering_input;
            prev_throttle_input = throttle_input;
        

            // Set driver inputs
            if(isnan(steering_input) || isinf(steering_input)) {
                steering_input = 0.0;
            }
            if(isnan(throttle_input) || isinf(throttle_input)) {
                throttle_input = 0.0;
            }

            driver_inputs.m_steering = steering_input;
            driver_inputs_hm.m_throttle = throttle_input;
            // driver_inputs.m_braking = 0.0;


            last_time = sys.GetChTime();
        
            std::cout << "Dt " << dt << current_waypoint_index << "... Heading Error " << heading_error << " Steering " << steering_input << " Throttle " << throttle_input << " R " << R << std::endl;
        }



        // Update modules
        terrain.Synchronize(time);
        vehicle->Synchronize(time, driver_inputs, terrain);

        hm1->Synchronize(time, driver_inputs_hm,spindle1->GetAngVelLocal()[2]*149.);
        hm2->Synchronize(time, driver_inputs_hm,spindle2->GetAngVelLocal()[2]*149.);
        hm3->Synchronize(time, driver_inputs_hm,spindle3->GetAngVelLocal()[2]*149.);
        hm4->Synchronize(time, driver_inputs_hm,spindle4->GetAngVelLocal()[2]*149.);
        


        // Advance simulation
        terrain.Advance(step_size);
        vehicle->Advance(step_size);

        hm1->Advance(step_size);
        hm2->Advance(step_size);
        hm3->Advance(step_size);
        hm4->Advance(step_size);

        spindle1->EmptyAccumulator(sp1_acccu);
        spindle2->EmptyAccumulator(sp2_acccu);
        spindle3->EmptyAccumulator(sp3_acccu);
        spindle4->EmptyAccumulator(sp4_acccu);


        spindle1->AccumulateTorque(sp1_acccu, ChVector3d(0, hm1->GetOutputMotorshaftTorque()*149., 0), true);
        spindle2->AccumulateTorque(sp2_acccu, ChVector3d(0, hm2->GetOutputMotorshaftTorque()*149., 0), true);
        spindle3->AccumulateTorque(sp3_acccu, ChVector3d(0, hm3->GetOutputMotorshaftTorque()*149., 0), true);
        spindle4->AccumulateTorque(sp4_acccu, ChVector3d(0, hm4->GetOutputMotorshaftTorque()*149., 0), true);
        

        sys.DoStepDynamics(step_size);

        step_number++;


        #ifdef CHRONO_IRRLICHT
            auto current_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = current_time - last_vis_update_time;
            if (elapsed.count() >= vis_update_time) {
                vis->BeginScene();
                vis->Render();
                vis->Advance(step_size);
                vis->EndScene();
                last_vis_update_time = current_time;
            } 
        #endif
    }
}

// Create vehicle function
std::shared_ptr<WheeledVehicle> CreateVehicle(ChSystem& sys, const ChCoordsys<>& init_pos, std::string vehicle_type,
                                                double mass1, double mass2) {
    std::string vehicle_json;
    std::string engine_json;
    std::string transmission_json;
    std::string tire_json;
    ChVector3d offset;


    vehicle_json = "LRV/Polaris.json";
    engine_json = "LRV/Polaris_EngineSimpleMap.json";
    transmission_json = "LRV/Polaris_AutomaticTransmissionSimpleMap.json";
    tire_json = "LRV/Polaris_RigidTire.json";

    // Apply vehicle-specific offset
    ChCoordsys<> adjusted_init_pos(init_pos.pos + offset, init_pos.rot);

    // Create and initialize the vehicle
    auto vehicle = chrono_types::make_shared<WheeledVehicle>(&sys, vehicle::GetDataFile(vehicle_json));
    vehicle->Initialize(adjusted_init_pos);
    vehicle->GetChassis()->SetFixed(false);
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    std::shared_ptr<ChEngine> engine = ReadEngineJSON(vehicle::GetDataFile(engine_json));

    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_json));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle->InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
            vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    ChVector3d relative_offset(0.2, 0.3, 0.5); // Relative position offset
    ChVector3d rotated_offset = init_pos.rot.Rotate(relative_offset);
    ChVector3d relative_offset2(0.2, -0.3, 0.5); // Relative position offset
    ChVector3d rotated_offset2 = init_pos.rot.Rotate(relative_offset2);

    // Create a cube-shaped body
    auto cube_body = chrono_types::make_shared<ChBody>();
    cube_body->SetPos(init_pos.pos + rotated_offset); // Position the cube above the trailer's chassis
    cube_body->SetMass(mass1);                // Set cube mass
    cube_body->SetInertiaXX(ChVector3d(10, 10, 10)); // Set cube inertia
    cube_body->SetRot(init_pos.rot);

    // Add a visual shape (cube)
    auto cube_visual = chrono_types::make_shared<ChVisualShapeBox>();
    cube_visual->GetGeometry().SetLengths(ChVector3d(0.5, 0.5, 0.5)); // Cube dimensions (length, width, height)
    cube_body->AddVisualShape(cube_visual);

    // Attach cube to the trailer chassis
    cube_body->SetFixed(false); // Allow the cube to move
    // cube_body->SetCollide(true);    // Enable collision

    // Add the cube to the system
    sys.Add(cube_body);

    auto cube_body2 = chrono_types::make_shared<ChBody>();
    cube_body2->SetPos(init_pos.pos + rotated_offset2); // Position the cube above the trailer's chassis
    cube_body2->SetMass(mass2);                // Set cube mass
    cube_body2->SetInertiaXX(ChVector3d(100, 100, 100)); // Set cube inertia

    // Add a visual shape (cube)
    auto cube_visual2 = chrono_types::make_shared<ChVisualShapeBox>();
    cube_visual2->GetGeometry().SetLengths(ChVector3d(0.5, 0.5, 0.5)); // Cube dimensions (length, width, height)
    cube_body2->AddVisualShape(cube_visual);
    cube_body2->SetRot(init_pos.rot);
    cube_body2->SetFixed(false); // Allow the cube to move

    // Add the cube to the system
    sys.Add(cube_body2);


    // Optionally, add a joint to connect the cube to the trailer
    auto revolute_joint = chrono_types::make_shared<ChLinkMateFix>();
    revolute_joint->Initialize(cube_body, vehicle->GetChassis()->GetBody(), ChFrame<>(ChVector3d(0,0,0), QUNIT));
    sys.AddLink(revolute_joint);

    auto revolute_joint2 = chrono_types::make_shared<ChLinkMateFix>();
    revolute_joint2->Initialize(cube_body2, vehicle->GetChassis()->GetBody(), ChFrame<>(ChVector3d(0,0,0), QUNIT));
    sys.AddLink(revolute_joint2);


    return vehicle;
}
