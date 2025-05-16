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
// Author: Radu Serban
// =============================================================================
//
// ART wheeled vehicle on CRM terrain formed by a custom height map
//
// =============================================================================

#include <cstdio>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <thread>
#include <fstream>
#include <filesystem>
#include <vector>  // For std::vector
#include <array>   // For std::array
#include <sstream> // For std::istringstream
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/core/ChRotation.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_models/vehicle/artcar/ARTcar.h"
#include "chrono_models/vehicle/gator/Gator.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// ===================================================================================================================

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// CRM terrain patch type
enum class PatchType { RECTANGULAR, MARKER_DATA, HEIGHT_MAP };
PatchType patch_type = PatchType::HEIGHT_MAP;

// Terrain dimensions (for RECTANGULAR or HEIGHT_MAP patch type)
double terrain_length = 10;
double terrain_width = 4;

// Vehicle initial position
double vehicle_init_x = -2.0;
double vehicle_init_y = 0;
double vehicle_init_z = 0.3;
double vehicle_back_x = -2.0; 

// Vehicle movement state
enum class VehicleState {
    FORWARD_TO_POSITIVE,
    WAIT_AT_POSITIVE,
    BACKWARD_TO_NEGATIVE,
    WAIT_AT_NEGATIVE,
    FORWARD_TO_POSITIVE_AGAIN,
};

// Forward declarations for helper functions
std::tuple<std::shared_ptr<gator::Gator>, std::shared_ptr<ChBody>, std::shared_ptr<ChLinkMotorRotationAngle>, std::shared_ptr<ChLinkMotorRotationAngle>, std::shared_ptr<ChLinkMotorLinearPosition>> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires);
void CreateFSIWheels(std::shared_ptr<gator::Gator> vehicle, CRMTerrain& terrain);
void CreateFSIBlade(std::shared_ptr<ChBody> blade, CRMTerrain& terrain);
bool LoadBladePvControlData(const std::string& filename, std::vector<std::array<double, 2>>& pv_data);

bool GetProblemSpecs(int argc,
                     char** argv,
                     double& pile_max_height,
                     std::string& push_seq,
                     std::vector<double>& veh_init_state) {
    ChCLI cli(argv[0], "Soil Leveling Validation Configuration");

    // Add options for all parameters with their default values
    cli.AddOption<double>("Terrain", "pile_height", "Maximum pile height", std::to_string(pile_max_height));
    cli.AddOption<std::string>("Vehicle", "push_seq", "Push sequence (firstpush/secondpush)", "firstpush");
    cli.AddOption<std::vector<double>>("Vehicle", "veh_init_state", "Vehicle initial state [x,y,z,qw,qx,qy,qz]", "{-2.0,0.0,0.3,1.0,0.0,0.0,0.0}");

    // Display help if no arguments
    if (argc == 1) {
        std::cout << "Using default parameters. Override with command line options:\n\n";
        cli.Help();
        return true;  // Continue with defaults
    }

    // Parse command-line arguments
    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    // Retrieve values from CLI
    pile_max_height = cli.GetAsType<double>("pile_height");
    push_seq = cli.GetAsType<std::string>("push_seq");
    veh_init_state = cli.GetAsType<std::vector<double>>("veh_init_state");

    // Validate push sequence
    if (push_seq != "firstpush" && push_seq != "secondpush") {
        std::cerr << "Error: push_seq must be either 'firstpush' or 'secondpush'" << std::endl;
        return false;
    }

    // Validate vehicle initial state vector
    if (veh_init_state.size() != 7) {
        std::cerr << "Error: veh_init_state must be a 7-element vector [x,y,z,qw,qx,qy,qz]" << std::endl;
        return false;
    }

    return true;
}

// ===================================================================================================================

int main(int argc, char* argv[]) {
    // ---------------- 
    // Problem settings
    // ----------------

    double target_speed = 1.0;
    double tend = 20;
    bool verbose = true;

    // Visualization settings
    bool render = true;                    // use run-time visualization
    double render_fps = 20;               // rendering FPS
    double control_step_size = 1/20;               // control FPS
    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = false;  // render boundary BCE markers
    bool visualization_rigid_bce = true;   // render wheel BCE markers
    bool chase_cam = false;                 // chase-cam or fixed camera

    // CRM material properties
    double density = 1700;
    double cohesion = 5e3;
    double friction = 0.8;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    // CRM (moving) active box dimension
    double active_box_hdim = 2;
    double settling_time = 0;

    // Set SPH spacing
    double spacing = 0.02;

    // Process command-line parameters
    double pile_max_height = 0.3;
    std::string push_seq;
    std::vector<double> veh_init_state;

    if (!GetProblemSpecs(argc, argv, pile_max_height, push_seq, veh_init_state)) {
        return 1;
    }
    cout << "pile_max_height: " << pile_max_height << endl;
    cout << "push_seq: " << push_seq << endl;

    std::vector<std::array<double, 2>> blade_pv_setpoints;
    std::string blade_pv_filename = "/home/harry/AutoGrading/data/control_commands/" + std::to_string(pile_max_height) + "_" + push_seq + ".txt";
    if (!LoadBladePvControlData(blade_pv_filename, blade_pv_setpoints)) {
        std::cerr << "Warning: Could not load blade P,V control data. Blade pitch/vertical will rely solely on the original schedule." << std::endl;
        return 1;
    }

    VehicleState vehicle_state = VehicleState::FORWARD_TO_POSITIVE;
    double t_back = -1.0; // Initialize to indicate not yet reached
    double t_switch = -1.0; // Initialize to indicate not yet reached

    // --------------
    // Create vehicle
    // --------------

    ChCoordsys<> init_pos;
    if (push_seq == "firstpush") {
        init_pos = ChCoordsys<>(ChVector3d(-2.0, 0.0, 0.3), QuatFromAngleZ(0.0f));
    } else { // secondpush
        ChVector3d pos(veh_init_state[0], veh_init_state[1], veh_init_state[2]);
        ChQuaternion<> rot(veh_init_state[3], veh_init_state[4], veh_init_state[5], veh_init_state[6]);
        init_pos = ChCoordsys<>(pos, rot);
    }

    cout << "Create vehicle..." << endl;
    bool fea_tires;
    auto [vehicle, blade, motor_yaw, motor_pitch, motor_vertical] = CreateVehicle(init_pos, fea_tires);
    cout << "Finished creating vehicle"<< endl;
    auto sysMBS = vehicle->GetSystem();

    // ---------------------------------
    // Set solver and integrator for MBD
    // ---------------------------------

    double step_size = 0;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;

    if (fea_tires) {
        step_size = 1e-4;
        solver_type = ChSolver::Type::PARDISO_MKL;
        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    } else {
        step_size = 5e-4;
        solver_type = ChSolver::Type::BARZILAIBORWEIN;
        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    }

    int num_threads_chrono = std::min(8, ChOMP::GetNumProcs());
    int num_threads_collision = 1;
    int num_threads_eigen = 1;
    int num_threads_pardiso = std::min(8, ChOMP::GetNumProcs());

    SetChronoSolver(*sysMBS, solver_type, integrator_type, num_threads_pardiso);
    sysMBS->SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);

    // Set collision system
    sysMBS->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // ----------------------
    // Create the CRM terrain
    // ----------------------

    CRMTerrain terrain(*sysMBS, spacing);
    ChFsiSystemSPH& sysFSI = terrain.GetSystemFSI();
    terrain.SetVerbose(verbose);
    terrain.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    terrain.SetStepSizeCFD(step_size);

    // Disable automatic integration of the (vehicle) multibody system
    terrain.DisableMBD();

    // Set SPH parameters and soil material properties
    ChFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = density;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = cohesion;
    terrain.SetElasticSPH(mat_props);

    // Set SPH solver parameters
    ChFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1;
    sph_params.kernel_threshold = 0.8;
    sph_params.artificial_viscosity = 0.5;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    sph_params.boundary_type = BoundaryType::ADAMI;
    terrain.SetSPHParameters(sph_params);

    // Set output level from SPH simulation
    terrain.SetOutputLevel(OutputLevel::STATE);

    // Add vehicle wheels as FSI solids
    CreateFSIWheels(vehicle, terrain);
    
    // Add blade as FSI solid
    CreateFSIBlade(blade, terrain);
    
    terrain.SetActiveDomain(ChVector3d(active_box_hdim));
    terrain.SetActiveDomainDelay(settling_time);

    // Construct the terrain and associated path
    cout << "Create terrain..." << endl;
    switch (patch_type) {
        case PatchType::RECTANGULAR:
            terrain.Construct({terrain_length, terrain_width, 0.25},
                              ChVector3d(0, 0, 0),
                              BoxSide::ALL & ~BoxSide::Z_POS
            );
            break;
        case PatchType::HEIGHT_MAP:
            terrain.Construct(vehicle::GetDataFile("terrain/gator/terrain.bmp"),
                              terrain_length, terrain_width,
                              {0, pile_max_height},
                              0.45,
                              true,
                              ChVector3d(0, 0, 0),
                              BoxSide::Z_NEG
            );
            break;
        case PatchType::MARKER_DATA:
            terrain.Construct(vehicle::GetDataFile("terrain/sph/S-lane_RMS/sph_particles.txt"),
                              vehicle::GetDataFile("terrain/sph/S-lane_RMS/bce_markers.txt"),
                              VNULL);
            break;
    }

    // Initialize the terrain system
    terrain.Initialize();

    auto aabb = terrain.GetSPHBoundingBox();
    cout << "  SPH particles:     " << terrain.GetNumSPHParticles() << endl;
    cout << "  Bndry BCE markers: " << terrain.GetNumBoundaryBCEMarkers() << endl;
    cout << "  SPH AABB:          " << aabb.min << "   " << aabb.max << endl;

    // Set maximum vehicle X location (based on CRM patch size)
    double x_max = aabb.max.x() - 4.5;

    // --------------------------------
    // Create the driver
    // --------------------------------
    cout << "Create driver..." << endl;
    ChDriver driver(vehicle->GetVehicle());
    driver.Initialize();

    // -----------------------------
    // Create run-time visualization
    // -----------------------------
#if defined(CHRONO_OPENGL) || defined(CHRONO_VSG)
    #ifndef CHRONO_OPENGL
        if (vis_type == ChVisualSystem::Type::OpenGL)
            vis_type = ChVisualSystem::Type::VSG;
    #endif
    #ifndef CHRONO_VSG
        if (vis_type == ChVisualSystem::Type::VSG)
            vis_type = ChVisualSystem::Type::OpenGL;
    #endif
        std::shared_ptr<ChFsiVisualization> visFSI;
        if (render) {
            switch (vis_type) {
                case ChVisualSystem::Type::OpenGL:
                #ifdef CHRONO_OPENGL
                                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
                #endif
                    break;
                    case ChVisualSystem::Type::VSG: {
                #ifdef CHRONO_VSG
                                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
                #endif
                    break;
                }
            }

            visFSI->SetTitle("Wheeled vehicle on CRM deformable terrain");
            visFSI->SetSize(1280, 720);
            visFSI->SetCameraVertical(CameraVerticalDir::Z);
            visFSI->SetImageOutput(true);
            visFSI->SetImageOutputDirectory("./demo");
            visFSI->SetCameraMoveScale(0.2f);
            visFSI->EnableFluidMarkers(visualization_sph);
            visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
            visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
            visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
            visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
            visFSI->SetSPHColorCallback(chrono_types::make_shared<ParticleHeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f),
                                                                                            aabb.min.z(), aabb.max.z()));
            auto vis_vsg = std::dynamic_pointer_cast<ChFsiVisualizationVSG>(visFSI);
            if (vis_vsg) {
                vis_vsg->SetClearColor(ChColor(0.8f, 0.85f, 0.9f));
            }
            visFSI->AttachSystem(sysMBS);
            visFSI->AddCamera(ChVector3d(2, -4, .5), ChVector3d(2, 4, 0));
            visFSI->Initialize();
        }
#endif

    // ---------------
    // Simulation loop
    // ---------------

    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;
    bool saveframes = true;
    cout << "Start simulation..." << endl;
    
    // Create CSV file for logging vehicle data
    const std::string out_dir = GetChronoOutputPath() + std::to_string(pile_max_height) + "/soil_leveling_" + push_seq;
    if (!std::filesystem::exists(out_dir)) {
        try {
            std::filesystem::create_directories(out_dir);
        } catch (const std::filesystem::filesystem_error& e) {
            std::cout << "Error creating directory: " << e.what() << std::endl;
            const std::string fallback_dir = "./soil_leveling_output";
            std::cout << "Attempting to use fallback directory: " << fallback_dir << std::endl;
            std::filesystem::create_directories(fallback_dir);
            const_cast<std::string&>(out_dir) = fallback_dir;
        }
    }

    std::ofstream csv_file(out_dir + "/vehicle_data.csv");
    if (!csv_file.is_open()) {
        std::cerr << "Error: Could not create CSV file"  << std::endl;
        return 1;
    }
    csv_file << "time,x,y,z,rot_x,rot_y,rot_z,rot_w,steering,throttle,bx,by,bz,bpitch,broll,byaw\n";

    ChTimer timer;
    bool saved_particle = false;
    bool simulation_complete = false;

    while (time < tend && !simulation_complete) {
        const auto& veh_loc = vehicle->GetVehicle().GetPos();
        auto veh_speed = vehicle->GetVehicle().GetSpeed();
        const auto& veh_rot = vehicle->GetVehicle().GetRot();
        auto front_load = blade->GetAppliedForce();
        const auto& blade_loc = vehicle->GetChassisBody()->TransformPointParentToLocal(blade->GetPos());
        const auto& blade_rot = blade->GetRot().GetCardanAnglesZYX();
        auto engine_rpm = vehicle->GetVehicle().GetEngine()->GetMotorSpeed();
        auto engine_torque = vehicle->GetVehicle().GetEngine()->GetOutputMotorshaftTorque();

        DriverInputs driver_inputs;

        // State machine to control vehicle movement
        switch (vehicle_state) {
            case VehicleState::FORWARD_TO_POSITIVE: {
                if (verbose) cout << std::fixed << std::setprecision(3) << "Time: " << time << "s | State: FORWARD_TO_POSITIVE" << endl;
                driver_inputs.m_throttle = 1.0;
                driver_inputs.m_braking = 0.0;
                vehicle->GetVehicle().GetTransmission()->SetGear(1);
                
                if (time > 1.0 && time < 3.5) {
                    motor_pitch->SetAngleFunction(chrono_types::make_shared<ChFunctionConst>(blade_pv_setpoints[0][0]));
                    motor_vertical->SetMotionFunction(chrono_types::make_shared<ChFunctionConst>(blade_pv_setpoints[0][1]));
                }

                else if (time > 3.5 && time < 6.0) {
                    motor_pitch->SetAngleFunction(chrono_types::make_shared<ChFunctionConst>(blade_pv_setpoints[1][0]));
                    motor_vertical->SetMotionFunction(chrono_types::make_shared<ChFunctionConst>(blade_pv_setpoints[1][1]));
                }

                if (time >= 6.0 || veh_loc.x() >= 2.5) {
                    if (verbose) cout << "Transition: FORWARD_TO_POSITIVE -> WAIT_AT_POSITIVE" << endl;
                    vehicle_state = VehicleState::WAIT_AT_POSITIVE;
                    t_switch = time;
                }
                break;
            }
                
            case VehicleState::WAIT_AT_POSITIVE: {
                if (verbose) cout << std::fixed << std::setprecision(3) << "Time: " << time << "s | State: WAIT_AT_POSITIVE" << endl;
                driver_inputs.m_throttle = 0.0;
                driver_inputs.m_braking = 1.0;
                motor_pitch->SetAngleFunction(chrono_types::make_shared<ChFunctionConst>(0.0));
                motor_vertical->SetMotionFunction(chrono_types::make_shared<ChFunctionConst>(-0.1));
                
                if (time >= t_switch + 1.0) {
                    if (verbose) cout << "Transition: WAIT_AT_POSITIVE -> BACKWARD_TO_NEGATIVE" << endl;
                    vehicle_state = VehicleState::BACKWARD_TO_NEGATIVE;
                    vehicle->GetVehicle().GetTransmission()->SetGear(-1);
                }
                break;
            }
                
            case VehicleState::BACKWARD_TO_NEGATIVE: {
                if (verbose) cout << std::fixed << std::setprecision(3) << "Time: " << time << "s | State: BACKWARD_TO_NEGATIVE" << endl;
                driver_inputs.m_throttle = 0.1;
                driver_inputs.m_braking = 0.0;

                double dist_to_target = ChVector3d(veh_loc.x()-vehicle_back_x,0,0).Length();
                if (dist_to_target < 0.5) {
                    t_back = time;
                    if (verbose) cout << "Reached backward target at X=" << vehicle_back_x << " at time t_back = " << t_back << "s" << endl;
                    if (verbose) cout << "Transition: BACKWARD_TO_NEGATIVE -> WAIT_AT_NEGATIVE" << endl;
                    vehicle_state = VehicleState::WAIT_AT_NEGATIVE;
                    // Save SPH data and end simulation
                    if (!saved_particle) {
                        sysFSI.GetFluidSystemSPH().SaveParticleData(out_dir);
                        cout << "Particle data saved to " << out_dir << endl;
                        saved_particle = true;
                    }
                }
                break;
            }
                
            case VehicleState::WAIT_AT_NEGATIVE: {
                if (t_back < 0) {
                    if (verbose) cout << std::fixed << std::setprecision(3) << "Time: " << time << "s | State: WAIT_AT_NEGATIVE (waiting for t_back)" << endl;
                    driver_inputs.m_throttle = 0.0;
                    driver_inputs.m_braking = 1.0;
                    break;
                }
                
                if (verbose) cout << std::fixed << std::setprecision(3) << "Time: " << time << "s | State: WAIT_AT_NEGATIVE" << endl;
                driver_inputs.m_throttle = 0.0;
                driver_inputs.m_braking = 1.0;
                motor_vertical->SetMotionFunction(chrono_types::make_shared<ChFunctionConst>(0.0));
                if (time >= t_back + 1.0) {
                    vehicle_state = VehicleState::FORWARD_TO_POSITIVE_AGAIN;
                    vehicle->GetVehicle().GetTransmission()->SetGear(1); // Switch to forward gear
                    // read the new control command file
                    std::string push_seq = "secondpush";
                    std::string new_control_command_file = "/home/harry/AutoGrading/data/control_commands/" + std::to_string(pile_max_height) + "_" + push_seq + ".txt";
                    if (!LoadBladePvControlData(new_control_command_file, blade_pv_setpoints)) {
                        std::cerr << "Warning: Could not load blade P,V control data. Blade pitch/vertical will rely solely on the original schedule." << std::endl;
                        return 1;
                    }
                    std::cout << "Loaded new control command file: " << new_control_command_file << std::endl;
                }
                break;
            }

            case VehicleState::FORWARD_TO_POSITIVE_AGAIN: {
                if (verbose) cout << std::fixed << std::setprecision(3) << "Time: " << time << "s | State: FORWARD_TO_POSITIVE_AGAIN" << endl;
                driver_inputs.m_throttle = 1.0;
                driver_inputs.m_braking = 0.0;
                if (time > t_back + 2.0 && time < t_back + 4.5){
                    motor_pitch->SetAngleFunction(chrono_types::make_shared<ChFunctionConst>(blade_pv_setpoints[0][0]));
                    motor_vertical->SetMotionFunction(chrono_types::make_shared<ChFunctionConst>(blade_pv_setpoints[0][1]));
                }
                else if (time > t_back + 4.5 && time < t_back + 7.0){
                    motor_pitch->SetAngleFunction(chrono_types::make_shared<ChFunctionConst>(blade_pv_setpoints[1][0]));
                    motor_vertical->SetMotionFunction(chrono_types::make_shared<ChFunctionConst>(blade_pv_setpoints[1][1]));
                }
                break;
            }
        }

        // Run-time visualization
#if defined(CHRONO_OPENGL) || defined(CHRONO_VSG)
        if (render && time >= render_frame / render_fps) {
            if (chase_cam) {
                ChVector3d cam_loc = veh_loc + ChVector3d(6, 0, 2.5);
                ChVector3d cam_point = veh_loc;
                visFSI->UpdateCamera(cam_loc, cam_point);
            }
            if (!visFSI->Render())
                break;
            
            render_frame++;
        }
#endif

        if (!render) {
            std::cout << time << "  " << terrain.GetRtfCFD() << "  " << terrain.GetRtfMBD() << std::endl;
        }

        // Synchronize systems
        driver.Synchronize(time);
        terrain.Synchronize(time);

        // Get driver inputs
        vehicle->Synchronize(time, driver_inputs, terrain);

        // Advance system state
        driver.Advance(step_size);
        timer.reset();
        timer.start();

        terrain.Advance(step_size);
        vehicle->Advance(step_size);

        timer.stop();
        double rtf = timer() / step_size;
        sysFSI.SetRtf(rtf);

        // Log data to CSV file
        csv_file << time << "," 
                 << veh_loc.x() << "," << veh_loc.y() << "," << veh_loc.z() << ","
                 << veh_rot.e0() << "," << veh_rot.e1() << "," << veh_rot.e2() << "," << veh_rot.e3() << ","
                 << driver_inputs.m_steering << "," << driver_inputs.m_throttle << ","
                 << blade_loc.x() << "," << blade_loc.y() << "," << blade_loc.z() << ","
                 << blade_rot.x() << "," << blade_rot.y() << "," << blade_rot.z() << "\n";
        csv_file.flush();

        time += step_size;
        sim_frame++;
    }
    
    // Close the CSV file
    csv_file.close();
    cout << "Simulation completed"<< endl;
    
    return 0;
}

// ===================================================================================================================

std::tuple<std::shared_ptr<gator::Gator>, std::shared_ptr<ChBody>, std::shared_ptr<ChLinkMotorRotationAngle>, std::shared_ptr<ChLinkMotorRotationAngle>, std::shared_ptr<ChLinkMotorLinearPosition>> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires) {
    fea_tires = false;

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

    auto vir_yaw_body = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 1000, true, false,contact_mat);
    vir_yaw_body->SetPos(gator->GetChassisBody()->TransformPointLocalToParent(offsetpos));
    gator->GetSystem()->AddBody(vir_yaw_body);
    auto vir_pitch_body = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 1000, true, false,contact_mat);
    vir_pitch_body->SetPos(gator->GetChassisBody()->TransformPointLocalToParent(offsetpos));
    gator->GetSystem()->AddBody(vir_pitch_body);
    auto vir_vertical_body = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 1000, true, false,contact_mat);

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

void CreateFSIWheels(std::shared_ptr<gator::Gator> vehicle, CRMTerrain& terrain) {
    std::string mesh_filename = vehicle::GetDataFile("gator/gator_tireF_coarse.obj");
    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(ChContactMaterialData());
    geometry.coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, mesh_filename, VNULL));

    for (auto& axle : vehicle->GetVehicle().GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire_fea = std::dynamic_pointer_cast<ChDeformableTire>(wheel->GetTire());
            if (tire_fea) {
                auto mesh = tire_fea->GetMesh();
                if (mesh->GetNumContactSurfaces() > 0) {
                    auto surf = mesh->GetContactSurface(0);
                    cout << "FEA tire HAS contact surface: ";
                    if (std::dynamic_pointer_cast<fea::ChContactSurfaceNodeCloud>(surf))
                        cout << " NODE_CLOUD" << endl;
                    else
                        cout << " TRI_MESH" << endl;
                } else {
                    cout << "FEA tire DOES NOT HAVE contact surface!" << endl;
                }
                terrain.AddFeaMesh(mesh, false);
            } else {
                terrain.AddRigidBody(wheel->GetSpindle(), geometry, false);
            }
        }
    }
}

void CreateFSIBlade(std::shared_ptr<ChBody> blade, CRMTerrain& terrain) {
    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(ChContactMaterialData());
    
    ChVector3d box_size(1.5, 0.5, 0.05);  // half-dimensions
    ChVector3d box_pos(0, -0.1, 0);
    ChQuaternion<> rot = QuatFromAngleY(CH_PI / 2);  // 90 degrees
    geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(box_pos, rot, box_size));

    size_t num_blade_BCE = terrain.AddRigidBody(blade, geometry, false);
    cout << "Added " << num_blade_BCE << " BCE markers on blade" << endl;
}

bool LoadBladePvControlData(const std::string& filename, std::vector<std::array<double, 2>>& pv_data) {
    pv_data.clear();

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open blade P,V control data file: " << filename << std::endl;
        return false;
    }

    std::string line;
    int valid_lines_read = 0;
    while (std::getline(file, line) && valid_lines_read < 2) {  // Changed to expect only 2 pairs
        std::string trimmed_line = line;
        trimmed_line.erase(std::remove_if(trimmed_line.begin(), trimmed_line.end(), ::isspace), trimmed_line.end());
        if (trimmed_line.empty()) {
            continue;
        }

        std::stringstream ss(line);
        std::string p_str, v_str;
        
        if (std::getline(ss, p_str, ',') && std::getline(ss, v_str)) {
            try {
                p_str.erase(p_str.find_last_not_of(" \n\r\t")+1);
                p_str.erase(0, p_str.find_first_not_of(" \n\r\t"));
                v_str.erase(v_str.find_last_not_of(" \n\r\t")+1);
                v_str.erase(0, v_str.find_first_not_of(" \n\r\t"));

                if (p_str.empty() || v_str.empty()) {
                    std::cerr << "Error: Empty value after parsing comma in P,V control file. Line: \"" << line << "\"" << std::endl;
                    continue;
                }
                pv_data.push_back({std::stod(p_str), std::stod(v_str)});
                valid_lines_read++;
            } catch (const std::invalid_argument& ia) {
                std::cerr << "Error: Invalid number format in P,V control file. Line: \"" << line << "\". Details: " << ia.what() << std::endl;
            } catch (const std::out_of_range& oor) {
                std::cerr << "Error: Number out of range in P,V control file. Line: \"" << line << "\". Details: " << oor.what() << std::endl;
            }
        } else {
            std::cerr << "Error: Incorrect format in P,V control file. Expected p,v format. Line: \"" << line << "\"" << std::endl;
        }
    }

    if (valid_lines_read < 2) {  // Changed to expect only 2 pairs
        std::cerr << "Error: Blade P,V control file does not contain enough valid data. Expected 2 rows of p,v pairs, found " << valid_lines_read << "." << std::endl;
        pv_data.clear();
        return false;
    }
    if (valid_lines_read > 2) {  // Changed to expect only 2 pairs
        std::cout << "Warning: Blade P,V control file contains more than 2 valid data rows. Using only the first 2." << std::endl;
        pv_data.resize(2);
    }

    std::cout << "Successfully loaded blade P,V control data (" << pv_data.size() << " pairs):" << std::endl;
    for (size_t i = 0; i < pv_data.size(); ++i) {
        std::cout << "  Set " << i + 1 << ": Pitch=" << pv_data[i][0] << ", Vertical=" << pv_data[i][1] << std::endl;
    }
    return true;
}
