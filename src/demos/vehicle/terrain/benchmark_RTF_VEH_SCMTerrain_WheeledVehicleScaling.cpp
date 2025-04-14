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
// Polaris wheeled vehicle on SCM terrain
//
// =============================================================================

#include <cstdio>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <thread>
#include <chrono>  // Added for timing measurements

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"  // Added for command line parsing

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
#endif

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

// ===================================================================================================================

// Terrain dimensions
double terrain_length = 1000;
double terrain_width = 3;
double terrain_height = 0.25;

// Vehicle specification files
std::string vehicle_json = "Polaris/Polaris.json";
std::string engine_json = "Polaris/Polaris_EngineSimpleMap.json";
std::string transmission_json = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
std::string tire_json = "Polaris/Polaris_RigidTire.json";

// Suspend vehicle
bool fix_chassis = false;

// ===================================================================================================================

std::shared_ptr<WheeledVehicle> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires);
std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);

// ===================================================================================================================

int main(int argc, char* argv[]) {
    // ----------------
    // Process command line arguments
    // ----------------

    // Create command line parser
    ChCLI cli("Polaris wheeled vehicle on SCM terrain");

    // Add command line options
    cli.AddOption<double>("", "l,length", "Terrain length", std::to_string(terrain_length));
    cli.AddOption<double>("", "w,width", "Terrain width", std::to_string(terrain_width));
    cli.AddOption<double>("", "height", "Terrain height", std::to_string(terrain_height));

    // Parse command line arguments
    if (!cli.Parse(argc, argv, true))
        return 0;

    // Get terrain dimensions from command line
    terrain_length = cli.GetAsType<double>("length");
    terrain_width = cli.GetAsType<double>("width");
    terrain_height = cli.GetAsType<double>("height");

    cout << "Terrain dimensions: " << terrain_length << " x " << terrain_width << " x " << terrain_height << endl;

    // ----------------
    // Problem settings
    // ----------------

    double target_speed = 7.0;
    double tend = 10;
    bool verbose = true;

    // Visualization settings
    bool render = false;      // use run-time visualization
    double render_fps = 200;  // rendering FPS

    // SCM active box dimension
    double active_box_hdim = 0.4;

    // --------------
    // Create vehicle
    // --------------

    cout << "Create vehicle..." << endl;
    double vehicle_init_height = 0.25;
    bool fea_tires;
    auto vehicle = CreateVehicle(ChCoordsys<>(ChVector3d(3.5, 0, vehicle_init_height), QUNIT), fea_tires);
    vehicle->GetChassis()->SetFixed(fix_chassis);
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
    // Create the SCM terrain
    // ----------------------

    auto terrain = chrono_types::make_shared<SCMTerrain>(sysMBS);

    // Displace/rotate the terrain reference plane
    terrain->SetPlane(ChCoordsys<>(ChVector3d(0, 0, -0.5)));

    // Use a regular grid
    double mesh_resolution = 0.05;  // SCM grid spacing
    terrain->Initialize(terrain_length, terrain_width, mesh_resolution);

    // Set the soil terramechanical parameters
    terrain->SetSoilParameters(2e6,   // Bekker Kphi
                              0,      // Bekker Kc
                              1.1,    // Bekker n exponent
                              0,      // Mohr cohesive limit (Pa)
                              30,     // Mohr friction limit (degrees)
                              0.01,   // Janosi shear coefficient (m)
                              2e8,    // Elastic stiffness (Pa/m), before plastic yield
                              3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    // Enable bulldozing effects
    terrain->EnableBulldozing(true);
    terrain->SetBulldozingParameters(
        55,  // angle of friction for erosion of displaced material at the border of the rut
        1,   // displaced material vs downward pressed material.
        5,   // number of erosion refinements per timestep
        6);  // number of concentric vertex selections subject to erosion

    // Add moving patches for each wheel
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            terrain->AddMovingPatch(wheel->GetSpindle(), ChVector3d(0, 0, 0), ChVector3d(active_box_hdim, active_box_hdim, active_box_hdim));
        }
    }

    // Set visualization parameters
    terrain->SetPlotType(SCMTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    terrain->SetMeshWireframe(true);

    // Create straight line path
    cout << "Create path..." << endl;
    auto path = StraightLinePath(ChVector3d(0, 0, vehicle_init_height),
                                ChVector3d(terrain_length, 0, vehicle_init_height), 1);

    // Set maximum vehicle X location (based on terrain size)
    double x_max = terrain_length - 4.5;

    // --------------------------------
    // Create the path-following driver
    // --------------------------------

    ChPathFollowerDriver driver(*vehicle, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(2.0);
    driver.GetSteeringController().SetGains(1.0, 0, 0);
    driver.GetSpeedController().SetGains(0.6, 0.05, 0);
    driver.Initialize();

    // -----------------------------
    // Set up output
    // -----------------------------

    std::string out_dir = GetChronoOutputPath() + "SCM_Wheeled_Vehicle/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    std::string out_file = out_dir + "/results.txt";
    utils::ChWriterCSV csv(" ");

    // -----------------------------
    // Create run-time visualization
    // -----------------------------

    std::shared_ptr<ChVehicleVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        auto visVSG = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
        visVSG->AttachVehicle(vehicle.get());
        visVSG->SetWindowTitle("Wheeled vehicle on SCM deformable terrain");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->EnableSkyBox();
        visVSG->SetLightIntensity(1.0f);
        visVSG->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
        visVSG->SetCameraAngleDeg(40);
        visVSG->SetChaseCamera(VNULL, 6.0, 2.0);
        visVSG->SetChaseCameraPosition(ChVector3d(0, 8, 1.5));
        visVSG->AddGuiColorbar("Pressure yield [Pa]", 0.0, 30000.2);
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
    bool braking = false;

    cout << "Start simulation..." << endl;

    // Add timing measurement for entire simulation
    auto sim_start = std::chrono::high_resolution_clock::now();
    ChTimer timer;
    while (time < tend) {
        const auto& veh_loc = vehicle->GetPos();

        // Set current driver inputs
        auto driver_inputs = driver.GetInputs();

        // Ramp up throttle to value requested by the cruise controller
        if (time < 0.5) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        } else {
            ChClampValue(driver_inputs.m_throttle, driver_inputs.m_throttle, (time - 0.5) / 0.5);
        }

        // Stop vehicle before reaching end of terrain patch, then end simulation after 2 more seconds
        if (veh_loc.x() > x_max) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
            if (!braking) {
                cout << "Start braking..." << endl;
                tend = time + 2;
                braking = true;
            }
        }

#ifdef CHRONO_VSG
        // Run-time visualization
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }
#endif

        // Synchronize systems
        driver.Synchronize(time);
#ifdef CHRONO_VSG
        if (render) {
            vis->Synchronize(time, driver_inputs);
        }
#endif
        terrain->Synchronize(time);
        vehicle->Synchronize(time, driver_inputs, *terrain);

        // Advance system state
        driver.Advance(step_size);
#ifdef CHRONO_VSG
        if (render) {
            vis->Advance(step_size);
        }
#endif
        terrain->Advance(step_size);
        vehicle->Advance(step_size);

        csv << time << vehicle->GetPos() << vehicle->GetSpeed() << endl;

        time += step_size;
        sim_frame++;
    }

    // Calculate and print timing statistics
    auto sim_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> sim_duration = sim_end - sim_start;
    double real_time_taken = sim_duration.count();
    double rtf_overall = real_time_taken / time;

    cout << "\n=== Simulation Complete ===" << endl;
    cout << "Terrain dimensions:   " << terrain_length << " x " << terrain_width << " x " << terrain_height << endl;
    cout << "Time simulated:       " << time << " seconds" << endl;
    cout << "Real time taken:      " << real_time_taken << " seconds" << endl;
    cout << "Real-time factor:     " << rtf_overall << " (>1 means slower than real-time)" << endl;
    cout << "=========================" << endl;

    return 0;
}

// ===================================================================================================================

std::shared_ptr<WheeledVehicle> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires) {
    fea_tires = false;

    // Create and initialize the vehicle
    auto vehicle = chrono_types::make_shared<WheeledVehicle>(vehicle::GetDataFile(vehicle_json), ChContactMethod::SMC);
    vehicle->Initialize(init_pos);
    vehicle->GetChassis()->SetFixed(false);
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_json));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_json));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle->InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
            vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
            if (std::dynamic_pointer_cast<ChDeformableTire>(tire))
                fea_tires = true;
        }
    }

    return vehicle;
} 