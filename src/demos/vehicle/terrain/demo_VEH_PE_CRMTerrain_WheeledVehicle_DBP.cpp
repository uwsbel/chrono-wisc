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
// Polaris wheeled vehicle on CRM terrain
//
// =============================================================================

#include <cstdio>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <thread>

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/SimpleDriveline.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/powertrain/ChEngineSimple.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono_powerelectronics/circuits/ChElectronicMotor.h"

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
using namespace chrono::vehicle;
using namespace chrono::powerelectronics;
using std::cout;
using std::cerr;
using std::cin;
using std::endl;

// ===================================================================================================================

double RAD_S_TO_RPM = 60.0 / (2.0 * 3.14159265358979323846);
// CRM terrain patch type
enum class PatchType { RECTANGULAR, MARKER_DATA, HEIGHT_MAP };
PatchType patch_type = PatchType::RECTANGULAR;

// Terrain dimensions (for RECTANGULAR or HEIGHT_MAP patch type)
double terrain_length = 20;
double terrain_width = 3;

// Vehicle specification files
std::string vehicle_json = "Polaris/Polaris_LTV.json";
// std::string engine_json = "LRV/Polaris_EngineSimpleMap.json";
// std::string transmission_json = "LRV/Polaris_AutomaticTransmissionSimpleMap.json";
std::string driveline_json = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
std::string tire_json = "Polaris/Polaris_RigidTire.json";
// Also change the mesh in CreateFSIWheels()
// std::string tire_json = "LRV/Polaris_RigidMeshTire_lug.json";

// Suspend vehicle
bool fix_chassis = false;


BrakeType brake_type = BrakeType::SHAFTS;
// ===================================================================================================================

std::shared_ptr<WheeledVehicle> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires);
void CreateFSIWheels(std::shared_ptr<WheeledVehicle> vehicle, CRMTerrain& terrain);
std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);

// ===================================================================================================================

class HubMotor {

private:
    std::shared_ptr<ChElectronicMotor> motor;
    double PWM_MAX = 5.2;
    double PWM_MIN = 3.5;
    double PWM = 0.0;
    
    double last_torque = 0.0;
    double shaft_speed = 0.0;

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

        // Initialize the motor with time step
        motor->Initialize();  // Using the same time step as the main simulation
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

    /// Return the current motorshaft speed
    double GetMotorshaftSpeed() const {
        return shaft_speed;
    }

    void Synchronize(double time, const DriverInputs &driver_inputs, double motorshaft_speed) {
        this->shaft_speed = motorshaft_speed;
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

// ===================================================================================================================

int main(int argc, char* argv[]) {
    // ---------------- 
    // Problem settings
    // ----------------

    double target_speed = 4.0;
    double tend = 20;
    bool verbose = true;
    double sphere_density = 4000.0;  // Default cube density in kg/m³

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--density" && i + 1 < argc) {
            sphere_density = std::stod(argv[i + 1]);
            i++; // Skip the next argument since it's the value
        }
    }

    printf("sphere_density: %f\n", sphere_density);

    // Visualization settings
    bool render = true;                    // use run-time visualization
    double render_fps = 200;               // rendering FPS
    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = false;  // render boundary BCE markers
    bool visualization_rigid_bce = true;   // render wheel BCE markers

    // Enable saving snapshots
    bool snapshots = false;                // save snapshots during simulation

    // CRM material properties
    double density = 1800;
    double cohesion = 5e2;
    double friction = 0.9;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    // CRM (moving) active box dimension
    double active_box_hdim = 0.4;
    double settling_time = 0;

    // Set SPH spacing
    double spacing = (patch_type == PatchType::MARKER_DATA) ? 0.02 : 0.04;

    // --------------
    // Create vehicle
    // --------------

    cout << "Create vehicle..." << endl;
    double vehicle_init_height = 0.25;
    bool fea_tires;
    auto vehicle = CreateVehicle(ChCoordsys<>(ChVector3d(3.5, 0, vehicle_init_height), QUNIT), fea_tires);
    vehicle->GetChassis()->SetFixed(fix_chassis);
    auto sysMBS = vehicle->GetSystem();

    // Spindle hacks for motors
    std::shared_ptr<ChBody> spindle1 = vehicle->GetWheel(0, VehicleSide::LEFT)->GetSpindle();
    std::shared_ptr<ChBody> spindle2 = vehicle->GetWheel(0, VehicleSide::RIGHT)->GetSpindle();
    std::shared_ptr<ChBody> spindle3 = vehicle->GetWheel(1, VehicleSide::LEFT)->GetSpindle();
    std::shared_ptr<ChBody> spindle4 = vehicle->GetWheel(1, VehicleSide::RIGHT)->GetSpindle();

        // Setup accumulators for spindle
    unsigned int sp1_acccu = spindle1->AddAccumulator();
    unsigned int sp2_acccu = spindle2->AddAccumulator();
    unsigned int sp3_acccu = spindle3->AddAccumulator();
    unsigned int sp4_acccu = spindle4->AddAccumulator();

    auto hm1 = std::make_shared<HubMotor>(spindle1);
    auto hm2 = std::make_shared<HubMotor>(spindle2);
    auto hm3 = std::make_shared<HubMotor>(spindle3);
    auto hm4 = std::make_shared<HubMotor>(spindle4);


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
    terrain.SetGravitationalAcceleration(ChVector3d(0, 0, -1.62));
    terrain.SetStepSizeCFD(step_size);

    // Register the vehicle with the CRM terrain
    terrain.RegisterVehicle(vehicle.get());

    // Set SPH parameters and soil material properties
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = density;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.002;
    mat_props.cohesion_coeff = cohesion;
    terrain.SetElasticSPH(mat_props);

    // Set SPH solver parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
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
    terrain.SetActiveDomain(ChVector3d(active_box_hdim));
    terrain.SetActiveDomainDelay(settling_time);

    // Construct the terrain and associated path
    cout << "Create terrain..." << endl;
    std::shared_ptr<ChBezierCurve> path;
    switch (patch_type) {
        case PatchType::RECTANGULAR:
            // Create a rectangular terrain patch
            terrain.Construct({terrain_length, terrain_width, 0.25},  // length X width X height
                              ChVector3d(terrain_length / 2, 0, 0),   // patch center
                              BoxSide::ALL & ~BoxSide::Z_POS          // all boundaries, except top
            );
            // Create straight line path
            path = StraightLinePath(ChVector3d(0, 0, vehicle_init_height),
                                    ChVector3d(terrain_length, 0, vehicle_init_height), 1);
            break;
        case PatchType::HEIGHT_MAP:
            // Create a patch from a heigh field map image
            terrain.Construct(vehicle::GetDataFile("terrain/height_maps/bump64.bmp"),  // height map image file
                              terrain_length, terrain_width,                           // length (X) and width (Y)
                              {0, 0.3},                                                // height range
                              0.25,                                                    // depth
                              true,                                                    // uniform depth
                              ChVector3d(terrain_length / 2, 0, 0),                    // patch center
                              BoxSide::Z_NEG                                           // bottom wall
            );
            // Create straight line path
            path = StraightLinePath(ChVector3d(0, 0, vehicle_init_height),
                                    ChVector3d(terrain_length, 0, vehicle_init_height), 1);
            break;
        case PatchType::MARKER_DATA:
            // Create a patch using SPH particles and BCE markers from files
            terrain.Construct(vehicle::GetDataFile("terrain/sph/S-lane_RMS/sph_particles.txt"),  // SPH marker locations
                              vehicle::GetDataFile("terrain/sph/S-lane_RMS/bce_markers.txt"),    // BCE marker locations
                              VNULL);
            // Create path from data file
            path = CreatePath("terrain/sph/S-lane_RMS/path.txt");
            break;
    }

    // Add a draggable sphere
    auto sphere = chrono_types::make_shared<ChBody>();
    
    // Set sphere properties
    double sphere_radius = 0.397;  // Sphere radius in m - 
    
    // Calculate mass and inertia from density and size
    double sphere_volume = (4.0/3.0) * CH_PI * sphere_radius * sphere_radius * sphere_radius;
    double sphere_mass = sphere_density * sphere_volume;
    double sphere_inertia = (2.0/5.0) * sphere_mass * sphere_radius * sphere_radius;
    
    sphere->SetMass(sphere_mass);
    sphere->SetInertiaXX(ChVector3d(sphere_inertia, sphere_inertia, sphere_inertia));
    sphere->SetPos(ChVector3d(1, 0, 0.65));  // Initial position 1m behind vehicle
    sphere->SetFixed(false);
    sphere->EnableCollision(false);

    // Add collision geometry for the sphere
    auto sphere_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    sphere_mat->SetFriction(0.8f);
    sphere_mat->SetRestitution(0.1f);
    sphere_mat->SetYoungModulus(1e8);
    sphere_mat->SetPoissonRatio(0.3);

    auto sphere_shape = chrono_types::make_shared<ChCollisionShapeSphere>(sphere_mat, sphere_radius);
    sphere->AddCollisionShape(sphere_shape);

    // Add visualization for the sphere
    auto sphere_vis = chrono_types::make_shared<ChVisualShapeSphere>(sphere_radius);
    sphere_vis->SetColor(ChColor(0.8f, 0.2f, 0.2f));  // Red color
    sphere->AddVisualShape(sphere_vis);

    sysMBS->AddBody(sphere);

    // Add sphere to FSI system
    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(ChContactMaterialData());
    geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(VNULL, sphere_radius, 0));
    terrain.AddRigidBody(sphere, geometry, false);

    // Create a distance constraint between the vehicle chassis and the sphere
    auto distance_constraint = chrono_types::make_shared<ChLinkDistance>();
    distance_constraint->Initialize(vehicle->GetChassisBody(), sphere, true, 
                                  ChVector3d(-1, 0, 0.397),  // Point on chassis (1m behind)
                                  ChVector3d(0, 0, 0));    // Point on sphere (center)
    sysMBS->AddLink(distance_constraint);

    // Initialize the terrain system
    terrain.Initialize();

    auto aabb = terrain.GetSPHBoundingBox();
    cout << "  SPH particles:     " << terrain.GetNumSPHParticles() << endl;
    cout << "  Bndry BCE markers: " << terrain.GetNumBoundaryBCEMarkers() << endl;
    cout << "  SPH AABB:          " << aabb.min << "   " << aabb.max << endl;

    // Set maximum vehicle X location (based on CRM patch size)
    double x_max = aabb.max.x() - 4.5;

    // --------------------------------
    // Create the path-following driver
    // --------------------------------

    cout << "Create path..." << endl;
    ChPathFollowerDriver driver(*vehicle, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(2.0);
    driver.GetSteeringController().SetGains(1.0, 0, 0);
    driver.GetSpeedController().SetGains(0.6, 0.05, 0);
    driver.Initialize();

    // -----------------------------
    // Set up output
    // -----------------------------

    std::string out_dir = GetChronoOutputPath() + "CRM_Wheeled_Vehicle_DBP/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
            cerr << "Error creating directory " << out_dir + "/snapshots" << endl;
            return 1;
        }
    }

    std::string out_file = out_dir + "/results.txt";
    utils::ChWriterCSV csv(" ");
    std::ofstream csv_file(out_dir+"/dbp_simulation_data.csv");
    csv_file << "time,sphere_density,sr1,sr2,sr3,sr4,sa1,sa2,sa3,sa4,"
             << "motor_torque1,motor_torque2,motor_torque3,motor_torque4,"
             << "x,y,z,roll,pitch,yaw,vx,vy,vz,throttle,braking,"
             << "wheel_torque1,wheel_torque2,wheel_torque3,wheel_torque4,"
             << "wheel_speed1,wheel_speed2,wheel_speed3,wheel_speed4,"
             << "motorshaft_speed1,motorshaft_speed2,motorshaft_speed3,motorshaft_speed4\n";

    // -----------------------------
    // Create run-time visualization
    // -----------------------------

    std::shared_ptr<ChVehicleVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        visFSI->SetSPHColorCallback(chrono_types::make_shared<ParticleHeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f), aabb.min.z(), aabb.max.z()));

        // Wheeled vehicle VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
        visVSG->AttachVehicle(vehicle.get());
        visVSG->AttachPlugin(visFSI);
        visVSG->SetWindowTitle("Wheeled vehicle on CRM deformable terrain");
        visVSG->SetWindowSize(1280, 720);
        visVSG->SetWindowPosition(400, 400);
        visVSG->EnableSkyBox();
        visVSG->SetLightIntensity(1.0f);
        visVSG->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
        visVSG->SetCameraAngleDeg(40);
        visVSG->SetChaseCamera(VNULL, 6.0, 2.0);
        visVSG->SetChaseCameraPosition(ChVector3d(0, 8, 1.5));

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // ---------------
    // Simulation loop
    // ---------------

    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;
    int snapshot_frame = 0;  // Counter for sequential snapshot numbering
    bool braking = false;

    cout << "Start simulation..." << endl;

    while (time < tend) {
        // ===============================================
        // Write parameters to file
        // ===============================================
        // Collect vehicle state data
        ChVector3d vehicle_pos = vehicle->GetPos();
        ChQuaternion<> vehicle_rot = vehicle->GetRot();
        ChVector3d vehicle_vel = vehicle->GetChassisBody()->GetPosDt();

        // Set current driver inputs
        auto driver_inputs = driver.GetInputs();

        // Extract longitudinal and lateral slip ratio (assuming the first axle and first wheel)
        double sr1 = vehicle->GetAxle(0)->GetWheel(VehicleSide::LEFT)->GetTire()->GetLongitudinalSlip();
        double sr2 = vehicle->GetAxle(0)->GetWheel(VehicleSide::RIGHT)->GetTire()->GetLongitudinalSlip();
        double sr3 = vehicle->GetAxle(1)->GetWheel(VehicleSide::LEFT)->GetTire()->GetLongitudinalSlip();
        double sr4 = vehicle->GetAxle(1)->GetWheel(VehicleSide::RIGHT)->GetTire()->GetLongitudinalSlip();
        
        double sa1 = vehicle->GetAxle(0)->GetWheel(VehicleSide::LEFT)->GetTire()->GetSlipAngle();
        double sa2 = vehicle->GetAxle(0)->GetWheel(VehicleSide::RIGHT)->GetTire()->GetSlipAngle();
        double sa3 = vehicle->GetAxle(1)->GetWheel(VehicleSide::LEFT)->GetTire()->GetSlipAngle();
        double sa4 = vehicle->GetAxle(1)->GetWheel(VehicleSide::RIGHT)->GetTire()->GetSlipAngle();

        // Motor torque
        double motor_torque1 = hm1->GetOutputMotorshaftTorque();
        double motor_torque2 = hm2->GetOutputMotorshaftTorque();
        double motor_torque3 = hm3->GetOutputMotorshaftTorque();
        double motor_torque4 = hm4->GetOutputMotorshaftTorque();

        // Wheel torque
        double wheel_torque1 = spindle1->GetAccumulatedTorque(sp1_acccu).y();
        double wheel_torque2 = spindle2->GetAccumulatedTorque(sp2_acccu).y();
        double wheel_torque3 = spindle3->GetAccumulatedTorque(sp3_acccu).y();
        double wheel_torque4 = spindle4->GetAccumulatedTorque(sp4_acccu).y();

        // Wheel rpm
        double wheel_speed1 = spindle1->GetAngVelLocal()[1];
        double wheel_speed2 = spindle2->GetAngVelLocal()[1];
        double wheel_speed3 = spindle3->GetAngVelLocal()[1];
        double wheel_speed4 = spindle4->GetAngVelLocal()[1];

        // Vehicle orientation in Euler angles (roll, pitch, yaw)
        ChVector3d euler_angles = vehicle_rot.GetRotVec();
        double roll = euler_angles.x();
        double pitch = euler_angles.y();
        double yaw = euler_angles.z();

        csv_file << std::fixed << std::setprecision(6)
                << time << ","
                << sphere_density << ","
                << sr1 << ","
                << sr2 << ","
                << sr3 << ","
                << sr4 << ","
                << sa1 << ","
                << sa2 << ","
                << sa3 << ","
                << sa4 << ","
                << motor_torque1 << ","
                << motor_torque2 << ","
                << motor_torque3 << ","
                << motor_torque4 << ","
                << vehicle_pos.x() << ","
                << vehicle_pos.y() << ","
                << vehicle_pos.z() << ","
                << roll << ","
                << pitch << ","
                << yaw << ","
                << vehicle_vel.x() << ","
                << vehicle_vel.y() << ","
                << vehicle_vel.z() << ","
                << driver_inputs.m_throttle << ","
                << driver_inputs.m_braking << ","
                << wheel_torque1 << ","
                << wheel_torque2 << ","
                << wheel_torque3 << ","
                << wheel_torque4 << ","
                << wheel_speed1 << ","
                << wheel_speed2 << ","
                << wheel_speed3 << ","
                << wheel_speed4 << ","
                << hm1->GetMotorshaftSpeed() << ","
                << hm2->GetMotorshaftSpeed() << ","
                << hm3->GetMotorshaftSpeed() << ","
                << hm4->GetMotorshaftSpeed() << "\n";

        // cout << vehicle_pos.x() << " " << vehicle_pos.y() << " " << vehicle_pos.z() << endl;
        // ===============================================
        const auto& veh_loc = vehicle->GetPos();


        // Ramp up throttle to value requested by the cruise controller
        if (time < 5.0) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        } else {
            ChClampValue(driver_inputs.m_throttle, driver_inputs.m_throttle, (time - 5.0) / 0.5);
        }

        // Stop vehicle before reaching end of terrain patch, then end simulation after 2 more second2
        if (veh_loc.x() > x_max) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
            if (!braking) {
                cout << "Start braking..." << endl;
                tend = time + 2;
                braking = true;
            }
        }

        // Run-time visualization
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            
            // Save snapshot if enabled
            if (snapshots && render_frame % (int)(render_fps / 100) == 0) {  
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << snapshot_frame + 1 << ".bmp";
                vis->WriteImageToFile(filename.str());
                snapshot_frame++;
            }
            
            render_frame++;
        }
        if (!render) {
            std::cout << time << "  " << terrain.GetRtfCFD() << "  " << terrain.GetRtfMBD() << std::endl;
        }

        // Synchronize systems
        driver.Synchronize(time);
        vis->Synchronize(time, driver_inputs);
        terrain.Synchronize(time);
        vehicle->Synchronize(time, driver_inputs, terrain);

        // TODO: Check if anything special needs to be done here?
        DriverInputs driver_inputs_hm = driver_inputs;

        hm1->Synchronize(time, driver_inputs_hm,spindle1->GetAngVelLocal()[1]*80.);
        hm2->Synchronize(time, driver_inputs_hm,spindle2->GetAngVelLocal()[1]*80.);
        hm3->Synchronize(time, driver_inputs_hm,spindle3->GetAngVelLocal()[1]*80.);
        hm4->Synchronize(time, driver_inputs_hm,spindle4->GetAngVelLocal()[1]*80.);

        // Advance simulation
        terrain.Advance(step_size);
        // vehicle->Advance(step_size);

        hm1->Advance(step_size);
        hm2->Advance(step_size);
        hm3->Advance(step_size);
        hm4->Advance(step_size);

        spindle1->EmptyAccumulator(sp1_acccu);
        spindle2->EmptyAccumulator(sp2_acccu);
        spindle3->EmptyAccumulator(sp3_acccu);
        spindle4->EmptyAccumulator(sp4_acccu);

        spindle1->AccumulateTorque(sp1_acccu,ChVector3d(0,hm1->GetOutputMotorshaftTorque()*80.,0),true); 
        spindle2->AccumulateTorque(sp2_acccu,ChVector3d(0,hm2->GetOutputMotorshaftTorque()*80.,0),true);
        spindle3->AccumulateTorque(sp3_acccu,ChVector3d(0,hm3->GetOutputMotorshaftTorque()*80.,0),true);
        spindle4->AccumulateTorque(sp4_acccu,ChVector3d(0,hm4->GetOutputMotorshaftTorque()*80.,0),true);

        // Output spindle torques
        // cout << "hm1: " << hm1->GetOutputMotorshaftTorque() << " " << spindle1->GetAccumulatedTorque(sp1_acccu) << endl;
        // cout << "hm2: " << hm2->GetOutputMotorshaftTorque() << " " << spindle2->GetAccumulatedTorque(sp2_acccu) << endl;
        // cout << "hm3: " << hm3->GetOutputMotorshaftTorque() << " " << spindle3->GetAccumulatedTorque(sp3_acccu) << endl;
        // cout << "hm4: " << hm4->GetOutputMotorshaftTorque() << " " << spindle4->GetAccumulatedTorque(sp4_acccu) << endl;

        // Advance system state
        // vehicle->Advance(step_size);
        driver.Advance(step_size);
        vis->Advance(step_size);


        time += step_size;
        sim_frame++;
    }

    csv.WriteToFile(out_file);

    // Print final x-coordinate
    ChVector3d final_vehicle_pos = vehicle->GetPos();
    cout << "Final vehicle x-coordinate: " << final_vehicle_pos.x() << " m" << endl;

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/height.gpl");
    gplot.SetGrid();
    std::string title = "Vehicle ref frame height";
    gplot.SetTitle(title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("height (m)");
    gplot.Plot(out_file, 1, 4, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif

    return 0;
}

// ===================================================================================================================

std::shared_ptr<WheeledVehicle> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires) {
    fea_tires = false;

    // Create and initialize the vehicle
    auto vehicle = chrono_types::make_shared<WheeledVehicle>(vehicle::GetDataFile(vehicle_json), ChContactMethod::SMC);
    auto driveline = chrono_types::make_shared<SimpleDriveline>(vehicle::GetDataFile(driveline_json));
    vehicle->Initialize(init_pos);
    vehicle->GetChassis()->SetFixed(false);
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);


    // Create and initialize the powertrain system
    // auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_json));
    // auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_json));
    // auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    // vehicle->InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
            vehicle->InitializeTire(tire, wheel, VisualizationType::NONE);
            if (std::dynamic_pointer_cast<ChDeformableTire>(tire))
                fea_tires = true;
        }
    }

    return vehicle;
}

void CreateFSIWheels(std::shared_ptr<WheeledVehicle> vehicle, CRMTerrain& terrain) {
    // std::string mesh_filename = vehicle::GetDataFile("Polaris/scaled_obj_files/body_6_1_scaled.obj");
    std::string mesh_filename = vehicle::GetDataFile("Polaris/meshes/Polaris_tire_collision.obj");
    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(ChContactMaterialData());
    geometry.coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, mesh_filename, VNULL));

    for (auto& axle : vehicle->GetAxles()) {
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

std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file) {
    // Open input file
    std::ifstream ifile(vehicle::GetDataFile(path_file));
    std::string line;

    // Read number of knots and type of curve
    size_t numPoints;
    size_t numCols;

    std::getline(ifile, line);
    std::istringstream iss(line);
    iss >> numPoints >> numCols;

    assert(numCols == 3);

    // Read path points
    std::vector<ChVector3d> points;

    for (size_t i = 0; i < numPoints; i++) {
        double x, y, z;
        std::getline(ifile, line);
        std::istringstream jss(line);
        jss >> x >> y >> z;
        points.push_back(ChVector3d(x, y, z));
    }

    // Include point beyond CRM patch
    {
        auto np = points.size();
        points.push_back(2.0 * points[np - 1] - points[np - 2]);
    }

    // Raise all path points
    for (auto& p : points)
        p.z() += 0.1;

    ifile.close();

    return std::shared_ptr<ChBezierCurve>(new ChBezierCurve(points));
}
