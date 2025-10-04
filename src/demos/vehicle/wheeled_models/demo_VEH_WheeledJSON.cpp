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
// Main driver function for a vehicle specified through JSON files.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChUtils.h"
<<<<<<< HEAD

#include "chrono_vehicle/driver/ChInteractiveDriver.h"
=======
#include <iomanip>
>>>>>>> old_cuda
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/vehicle/WheeledVehicleJSON.h"
#include "demos/SetChronoSolver.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire3443B.h"

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Trailer model selection (use only with HMMWV, Sedan, or UAZ)
bool add_trailer = false;
auto trailer_model = UT_Model();

// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

// Initial vehicle position and orientation (adjust for selected terrain)
double init_height = -0.05;
ChVector3d initLoc(0, 0, init_height);
double initYaw = 20 * CH_DEG_TO_RAD;

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Render frequency
double render_fps = 100;

// End time (used only if no run-time visualization)
double t_end = 20;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Select vehicle model (see WheeledVehicleJSON.h)
    auto models = WheeledVehicleJSON::List();

    int num_models = (int)models.size();
    int which = 0;
    std::cout << "Options:\n";
    for (int i = 0; i < num_models; i++)
        std::cout << i + 1 << "  " << models[i].second << std::endl;
    std::cout << "\nSelect vehicle: ";
    std::cin >> which;
    std::cout << std::endl;
    ChClampValue(which, 1, num_models);

    const auto& vehicle_model = models[2].first;

    // Create the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_model->VehicleJSON()), contact_method);
    vehicle.Initialize(ChCoordsys<>(initLoc, QuatFromAngleZ(initYaw)));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSubchassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(vehicle_model->EngineJSON()));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(vehicle_model->TransmissionJSON()));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (unsigned int i = 0; i < vehicle.GetNumberAxles(); i++) {
        for (auto& wheel : vehicle.GetAxle(i)->GetWheels()) {
            auto tire = chrono_types::make_shared<ANCFAirlessTire3443B>("Airless3443B");
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetRimRadius(0.13);              // Default is 0.225
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHeight(0.2);                  // Default is 0.225
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetWidth(0.24);                  // Default is 0.4
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetAlpha(0.05);                  // Default is 0.05
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusSpokes(1e9);     // Default is 76e9
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusOuterRing(1e9);  // Default is 76e9
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetPoissonsRatio(0.3);           // Default is 0.2
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivWidth(3);                  // Default is 3
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivSpokeLength(3);            // Default is 3
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivOuterRingPerSpoke(3);
            int collision_family = 7;
            auto surface_type = ChTire::ContactSurfaceType::TRIANGLE_MESH;
            double surface_dim = 0;
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetContactSurfaceType(surface_type, surface_dim,
                                                                                         collision_family);
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // Containing system
    auto sys = vehicle.GetSystem();

    // Create the trailer system (build into same ChSystem)
    std::shared_ptr<WheeledTrailer> trailer;
    if (add_trailer) {
        trailer = chrono_types::make_shared<WheeledTrailer>(sys, vehicle::GetDataFile(trailer_model.TrailerJSON()));
        trailer->Initialize(vehicle.GetChassis());
        trailer->SetChassisVisualizationType(VisualizationType::MESH);
        trailer->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        trailer->SetWheelVisualizationType(VisualizationType::MESH);
        for (auto& axle : trailer->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = chrono_types::make_shared<ANCFAirlessTire3443B>("Airless3443B");
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetRimRadius(0.13);           // Default is 0.225
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHeight(0.2);               // Default is 0.225
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetWidth(0.24);               // Default is 0.4
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetAlpha(0.05);               // Default is 0.05
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusSpokes(1e9);  // Default is 76e9
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusOuterRing(
                    1e9);                                                                      // Default is 76e9
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetPoissonsRatio(0.3);  // Default is 0.2
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivWidth(3);         // Default is 3
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivSpokeLength(3);   // Default is 3
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivOuterRingPerSpoke(3);
                int collision_family = 7;
                auto surface_type = ChTire::ContactSurfaceType::TRIANGLE_MESH;
                double surface_dim = 0;
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetContactSurfaceType(surface_type, surface_dim,
                                                                                             collision_family);
                trailer->InitializeTire(tire, wheel, VisualizationType::MESH);
            }
        }
    }

    // Associate a collision system
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the terrain
<<<<<<< HEAD
    RigidTerrain terrain(sys, vehicle::GetDataFile(rigidterrain_file));
    terrain.Initialize();
=======
    // RigidTerrain terrain(sys, vehicle::GetDataFile(rigidterrain_file));
    // terrain.Initialize();

    // ------------------
    SCMTerrain terrain(sys);
    terrain.SetSoilParameters(4e7,   // Bekker Kphi
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              20,    // Mohr friction limit (degrees)
                              0.01,  // Janosi shear coefficient (m)
                              2e8,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    // Optionally, enable bulldozing effects.
    ////terrain.EnableBulldozing(true);      // inflate soil at the border of the rut
    ////terrain.SetBulldozingParameters(55,   // angle of friction for erosion of displaced material at rut border
    ////                                0.8,  // displaced material vs downward pressed material.
    ////                                5,    // number of erosion refinements per timestep
    ////                                10);  // number of concentric vertex selections subject to erosion

    // Optionally, enable moving patch feature (single patch around vehicle chassis)
    terrain.AddMovingPatch(vehicle.GetChassisBody(), ChVector3d(0, 0, 0), ChVector3d(5, 3, 1));
    ChVector3d init_loc;
    ChVector2d patch_size;
    init_loc = ChVector3d(-15.0, -6.0, 0.6);
    patch_size = ChVector2d(40.0, 16.0);
    terrain.Initialize(patch_size.x(), patch_size.y(), 0.05);
    // Control visualization of SCM terrain
    terrain.GetMesh()->SetWireframe(true);

    // terrain.GetMesh()->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"));
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.1);
>>>>>>> old_cuda

    // Set solver and integrator
    double step_size = 3e-3;
    auto solver_type = ChSolver::Type::PARDISO_MKL;
    auto integrator_type = ChTimestepper::Type::HHT;
    // if (vehicle.HasBushings()) {
    //     solver_type = ChSolver::Type::MINRES;
    //     step_size = 2e-4;
    // }
    SetChronoSolver(*sys, solver_type, integrator_type);

    // Create the interactive VSG driver system
    ChInteractiveDriver driver(vehicle);
    driver.SetSteeringDelta(0.02);
    driver.SetThrottleDelta(0.02);
    driver.SetBrakingDelta(0.06);
    driver.Initialize();

    // Create the vehicle run-time visualization interface and the interactive driver
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::string title = "Vehicle demo - JSON specification - " + vehicle_model->ModelName();
    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle(title);
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), vehicle_model->CameraDistance(), 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&vehicle);
            vis_irr->AttachDriver(&driver);

            vis = vis_irr;
<<<<<<< HEAD
=======
            // driver = driver_irr;
>>>>>>> old_cuda
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle(title);
            vis_vsg->AttachTerrain(&terrain);
            vis_vsg->AttachVehicle(&vehicle);
            vis_vsg->AttachDriver(&driver);
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), vehicle_model->CameraDistance(), 0.5);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Initialize output directories
    const std::string out_dir = GetChronoOutputPath() + "WHEELED_JSON";
    const std::string veh_dir = out_dir + "/" + vehicle_model->ModelName();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(veh_dir))) {
        std::cout << "Error creating directory " << veh_dir << std::endl;
        return 1;
    }

    const std::string img_dir = veh_dir + "/IMG";
    if (!filesystem::create_directory(filesystem::path(img_dir))) {
        std::cout << "Error creating directory " << img_dir << std::endl;
        return 1;
    }

    // Generate JSON information with available output channels
    std::string out_json = vehicle.ExportComponentList();
    std::cout << out_json << std::endl;
    vehicle.ExportComponentList(veh_dir + "/component_list.json");

    vehicle.LogSubsystemTypes();

    // Optionally, enable output from selected vehicle subsystems
    ////vehicle.SetSuspensionOutput(0, true);
    ////vehicle.SetSuspensionOutput(1, true);
    ////vehicle.SetOutput(ChVehicleOutput::ASCII, veh_dir, "output", 0.1);

    // Simulation loop
    vehicle.EnableRealtime(false);

    int sim_frame = 0;
    int render_frame = 0;
    while (true) {
        double time = sys->GetChTime();

        if (vis) {
            if (!vis->Run())
                break;

            if (time >= render_frame / render_fps) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();
                std::ostringstream filename;
                filename << img_dir << "/img_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".jpg";
                vis->WriteImageToFile(filename.str());
                render_frame++;
            }
        } else if (time > t_end) {
            break;
        }

        // Get driver inputs
<<<<<<< HEAD
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
=======
        DriverInputs driver_inputs = driver->GetInputs();
        driver_inputs.m_throttle = 1.0;
        driver_inputs.m_steering = 0.0;
        driver_inputs.m_braking = 0.0;
        // Update modules (process inputs from other modules)
        // driver->Synchronize(time);
>>>>>>> old_cuda
        vehicle.Synchronize(time, driver_inputs, terrain);
        if (add_trailer)
            trailer->Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
<<<<<<< HEAD
        driver.Advance(step_size);
=======
        // driver->Advance(step_size);
>>>>>>> old_cuda
        vehicle.Advance(step_size);
        if (add_trailer)
            trailer->Advance(step_size);
        terrain.Advance(step_size);
        if (vis)
            vis->Advance(step_size);

        sim_frame++;
    }

    return 0;
}
