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
// M113 tracked vehicle on SCM terrain
//
// =============================================================================

#include <cstdio>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <thread>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChBlender.h"
#endif

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// ===================================================================================================================

std::shared_ptr<TrackedVehicle> CreateVehicle(const ChCoordsys<>& init_pos);
std::shared_ptr<ChBezierCurve> CreatePath();

// ===================================================================================================================

int main(int argc, char* argv[]) {
    // Set model and simulation parameters
    double target_speed = 7.0;
    double t_end = 2;
    double step_size = 5e-4;
    double active_box_hdim = 0.4;
    double mesh_resolution = 0.04;

    bool render = false;       // Set false except for debugging
    double render_fps = 100;  // rendering FPS

    bool chase_cam = true;  // chase-cam or fixed camera

    bool snapshots = false;       // Enable for visual snapshots
    bool blender_output = false;  // Enable for Blender post-processing
    double output_fps = 10.0;    // FPS for Blender output

    bool enable_moving_patch = false;

    // Process command line arguments
    ChCLI cli(argv[0], "Tracked vehicle on SCM terrain");

    // Add command line options
    cli.AddOption<double>("Physics", "mesh_resolution", "SCM grid spacing", std::to_string(mesh_resolution));
    cli.AddOption<std::string>("Physics", "enable_moving_patch", "Enable moving patch feature (true/false)", "false");
    // Parse command line arguments
    if (!cli.Parse(argc, argv, true))
        return 0;

    // Get parameter values from command line
    mesh_resolution = cli.GetAsType<double>("mesh_resolution");
    if (cli.GetAsType<std::string>("enable_moving_patch") == "true") {
        enable_moving_patch = true;
    } else {
        enable_moving_patch = false;
    }

    // Display values
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Tracked vehicle on SCM terrain" << std::endl;
    std::cout << "Problem Specifications:" << std::endl;
    std::cout << "  SCM grid spacing: " << mesh_resolution << std::endl;
    std::cout << "  Moving patch enabled: " << (enable_moving_patch ? "yes" : "no") << std::endl;

    // Set output path based on whether moving patch is enabled
    std::string benchmark_dir;
    if (enable_moving_patch) {
        benchmark_dir = GetChronoOutputPath() + "BENCHMARK3_RTF_Active_render/";
    } else {
        benchmark_dir = GetChronoOutputPath() + "BENCHMARK3_RTF_noActive_render/";
    }

    // Create output directories
    if (!filesystem::create_directory(filesystem::path(benchmark_dir))) {
        std::cerr << "Error creating directory " << benchmark_dir << std::endl;
        return 1;
    }

    std::string scm_dir = benchmark_dir + "SCM_TRACKED_VEHICLE/";
    if (!filesystem::create_directory(filesystem::path(scm_dir))) {
        std::cerr << "Error creating directory " << scm_dir << std::endl;
        return 1;
    }

    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(scm_dir + "/snapshots"))) {
            std::cerr << "Error creating directory " << scm_dir + "/snapshots" << std::endl;
            return 1;
        }
    }

    // Create vehicle
    cout << "Create vehicle..." << endl;
    ChVector3d veh_init_pos(5.0, 0, 0.7);
    auto vehicle = CreateVehicle(ChCoordsys<>(veh_init_pos, QUNIT));
    auto sysMBS = vehicle->GetSystem();
    SetChronoSolver(*sysMBS, ChSolver::Type::BARZILAIBORWEIN, ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Set collision system
    sysMBS->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the SCM terrain
    auto terrain = chrono_types::make_shared<SCMTerrain>(sysMBS);

    // Displace/rotate the terrain reference plane
    terrain->SetPlane(ChCoordsys<>(ChVector3d(0, 0, -0.5)));

    // Use a regular grid
    double length = 10;
    double width = 4;
    terrain->Initialize(length, width, mesh_resolution);

    // Set the soil terramechanical parameters
    terrain->SetSoilParameters(2e7,   // Bekker Kphi
                               0,     // Bekker Kc
                               1.1,   // Bekker n exponent
                               0,     // Mohr cohesive limit (Pa)
                               20,    // Mohr friction limit (degrees)
                               0.01,  // Janosi shear coefficient (m)
                               2e8,   // Elastic stiffness (Pa/m), before plastic yield
                               3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    // Enable bulldozing effects
    terrain->EnableBulldozing(true);
    terrain->SetBulldozingParameters(
        55,  // angle of friction for erosion of displaced material at the border of the rut
        1,   // displaced material vs downward pressed material.
        5,   // number of erosion refinements per timestep
        6);  // number of concentric vertex selections subject to erosion

    // Add moving patches for each track shoe
    if (enable_moving_patch) {
        auto nshoes_left = vehicle->GetNumTrackShoes(VehicleSide::LEFT);
        for (size_t i = 0; i < nshoes_left; i++) {
            auto shoe_body = vehicle->GetTrackShoe(VehicleSide::LEFT, i)->GetShoeBody();
            terrain->AddMovingPatch(shoe_body, ChVector3d(0, 0, 0),
                                    ChVector3d(active_box_hdim, active_box_hdim, active_box_hdim));
        }

        auto nshoes_right = vehicle->GetNumTrackShoes(VehicleSide::RIGHT);
        for (size_t i = 0; i < nshoes_right; i++) {
            auto shoe_body = vehicle->GetTrackShoe(VehicleSide::RIGHT, i)->GetShoeBody();
            terrain->AddMovingPatch(shoe_body, ChVector3d(0, 0, 0),
                                    ChVector3d(active_box_hdim, active_box_hdim, active_box_hdim));
        }
    }

    // Set visualization parameters
    terrain->SetPlotType(SCMTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    terrain->SetMeshWireframe(true);

    // Create driver
    cout << "Create path..." << endl;
    auto path = CreatePath();
    double x_max = path->GetPoint(path->GetNumPoints() - 2).x() - 3.0;
    ChPathFollowerDriver driver(*vehicle, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(2.0);
    driver.GetSteeringController().SetGains(1.0, 0, 0);
    driver.GetSpeedController().SetGains(0.6, 0.05, 0);
    driver.Initialize();

    // Create run-time visualization
    std::shared_ptr<ChVisualSystem> vis;
#ifdef CHRONO_VSG
    if (render) {
        auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
        vis_vsg->AttachSystem(sysMBS);
        vis_vsg->SetWindowTitle("Tracked vehicle on SCM terrain");
        vis_vsg->SetWindowSize(1280, 800);
        vis_vsg->SetWindowPosition(100, 100);
        vis_vsg->EnableSkyBox();
        vis_vsg->SetCameraAngleDeg(40);
        vis_vsg->SetLightIntensity(1.0f);
        vis_vsg->AddCamera(ChVector3d(0, 8, 0.5), ChVector3d(0, -1, 0));
        vis_vsg->AddGuiColorbar("Pressure yield [Pa]", 0.0, 20000.0);
        vis_vsg->Initialize();

        vis = vis_vsg;
    }
#else
    render = false;
#endif

    // Create the Blender exporter
#ifdef CHRONO_POSTPROCESS
    postprocess::ChBlender blender_exporter(sysMBS);
    if (blender_output) {
        blender_exporter.SetBasePath(scm_dir);
        blender_exporter.SetCamera(ChVector3d(-6, 6, 2.0), veh_init_pos, 45);
        blender_exporter.AddAll();
        blender_exporter.ExportScript();
    }
#endif

    // Simulation loop
    DriverInputs driver_inputs = {0, 0, 0};
    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;
    int blender_frame = 0;

    if (x_max < veh_init_pos.x())
        x_max = veh_init_pos.x() + 0.25;

    cout << "Start simulation..." << endl;
    TerrainForces shoe_forces_left(vehicle->GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle->GetNumTrackShoes(RIGHT));

    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    double total_sim_time = 0.0;
    double total_real_time = 0.0;

    while (time < t_end) {
        const auto& veh_loc = vehicle->GetPos();

        // Stop before end of patch
        if (veh_loc.x() > x_max)
            break;

        // Set current driver inputs
        driver_inputs = driver.GetInputs();

        if (time < 0.5) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        } else {
            ChClampValue(driver_inputs.m_throttle, driver_inputs.m_throttle, (time - 0.5) / 0.5);
        }

#ifdef CHRONO_VSG
        // Run-time visualization
        if (render && time >= render_frame / render_fps) {
            if (chase_cam) {
                ChVector3d cam_loc = veh_loc + ChVector3d(-6, 6, 0.5);
                ChVector3d cam_point = veh_loc;
                vis->UpdateCamera(cam_loc, cam_point);
            }
            if (!vis->Run())
                break;
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            // Save snapshots if enabled
            if (snapshots) {
                cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << scm_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                vis->WriteImageToFile(filename.str());
            }
            render_frame++;
        }
#endif

#ifdef CHRONO_POSTPROCESS
        // Export to Blender at specified FPS rate
        if (blender_output && time >= blender_frame / output_fps) {
            blender_exporter.ExportData();
            blender_frame++;
        }
#endif

        // Synchronize systems
        driver.Synchronize(time);
        terrain->Synchronize(time);
        vehicle->Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);

        // Advance systems
        driver.Advance(step_size);
        terrain->Advance(step_size);
        vehicle->Advance(step_size);

        // Update timing
        auto current_real_time = std::chrono::high_resolution_clock::now();
        total_real_time = std::chrono::duration<double>(current_real_time - start_time).count();
        total_sim_time = time;

        time += step_size;
        sim_frame++;
    }

    // Print results
    std::cout << "\nBenchmark Results:" << std::endl;
    std::cout << "-----------------" << std::endl;
    std::cout << "Terrain size: " << length << " x " << width << " m" << std::endl;
    std::cout << "Active box size: " << 0.5 << " x " << 2 * active_box_hdim << " x " << 2 * active_box_hdim << " m"
              << std::endl;
    std::cout << "Total simulated time: " << total_sim_time << " s" << std::endl;
    std::cout << "Total real time: " << total_real_time << " s" << std::endl;
    std::cout << "Real-time factor (RTF): " << total_sim_time / total_real_time << std::endl;

    // Write results to file
    std::ofstream outfile(scm_dir + "benchmark_results.txt");
    if (outfile.is_open()) {
        outfile << "Benchmark Results" << std::endl;
        outfile << "----------------" << std::endl;
        outfile << "Terrain type: SCM" << std::endl;
        outfile << "Terrain size: " << length << " x " << width << " m" << std::endl;
        outfile << "Active box size: " << 0.5 << " x " << 2 * active_box_hdim << " x " << 2 * active_box_hdim << " m"
                << std::endl;
        outfile << "Mesh resolution: " << mesh_resolution << " m" << std::endl;
        outfile << "Bulldozing enabled: yes" << std::endl;
        outfile << "Moving patch enabled: " << (enable_moving_patch ? "yes" : "no") << std::endl;
        outfile << "Total simulated time: " << total_sim_time << " s" << std::endl;
        outfile << "Total real time: " << total_real_time << " s" << std::endl;
        outfile << "Real-time factor (RTF): " << total_sim_time / total_real_time << std::endl;
        outfile.close();
        std::cout << "Results written to: " << scm_dir << "benchmark_results.txt" << std::endl;
    } else {
        std::cerr << "Error opening output file" << std::endl;
        return 1;
    }

    return 0;
}

// ===================================================================================================================

std::shared_ptr<TrackedVehicle> CreateVehicle(const ChCoordsys<>& init_pos) {
    std::string vehicle_json = "M113/vehicle/M113_Vehicle_SinglePin.json";
    std::string engine_json = "M113/powertrain/M113_EngineSimple.json";
    std::string transmission_json = "M113/powertrain/M113_AutomaticTransmissionSimpleMap.json";

    // Create and initialize the vehicle
    auto vehicle = chrono_types::make_shared<TrackedVehicle>(vehicle::GetDataFile(vehicle_json), ChContactMethod::NSC);
    vehicle->Initialize(init_pos);
    vehicle->GetChassis()->SetFixed(false);

    vehicle->SetChassisVisualizationType(VisualizationType::NONE);
    vehicle->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetIdlerWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetRollerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_json));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_json));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle->InitializePowertrain(powertrain);

    return vehicle;
}

std::shared_ptr<ChBezierCurve> CreatePath() {
    // Create a straight path
    std::vector<ChVector3d> points;
    points.push_back(ChVector3d(0, 0, 0));
    points.push_back(ChVector3d(100, 0, 0));

    return std::shared_ptr<ChBezierCurve>(new ChBezierCurve(points));
}