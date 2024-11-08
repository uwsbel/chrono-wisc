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
// Author: Milad Rakhsha, Wei Hu
// =============================================================================

#include <assert.h>
#include <stdlib.h>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChBodyGeometry.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;

std::shared_ptr<WheeledVehicle> CreateVehicle(ChSystemSMC& sys,
                                              ChFluidSystemSPH& sysSPH,
                                              ChFsiSystemSPH& sysFSI,
                                              const ChCoordsys<>& init_pos);
std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);

//------------------------------------------------------------------
// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// =============================================================================

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_DamBreak_Explicit.json");

    // Set model and simulation parameters
    std::string terrain_dir = "terrain/sph/S-lane_RMS";

    double t_end = 10.0;
    bool verbose = true;
    bool output = true;
    double output_fps = 20;
    bool render = true;
    double render_fps = 100;
    bool snapshots = true;
    int ps_freq = 1;

    // Dimension of the fluid domain
    double fxDim = 8;
    double fyDim = 2.0;
    double fzDim = 1.0;

    // Dimension of the space domain
    double bxDim = fxDim;
    double byDim = fyDim;
    double bzDim = fzDim + 0.05;

    // Create a physics system and an FSI system
    // TODO: do i use smc or nsc?
    ChSystemSMC sysMBS;
    ChFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.9f);
    cmaterial->SetRestitution(0.4f);

    // Create vehicle
    std::cout << "Create vehicle..." << std::endl;

    // Use the specified input JSON file
    sysSPH.ReadParametersFromFile(inputJson);
    sysFSI.SetVerbose(verbose);

    auto initSpace0 = sysSPH.GetInitialSpacing();

    // Set frequency of proximity search
    sysSPH.SetNumProximitySearchSteps(ps_freq);

    // Set up the periodic boundary condition (only in Y direction)
    ChVector3d cMin =
        ChVector3d(-bxDim / 2 - bxDim - 20.0 * initSpace0, -byDim / 2 - 1.0 * initSpace0 / 2.0, -2.0 * bzDim);
    ChVector3d cMax =
        ChVector3d(bxDim / 2 + bxDim + 20.0 * initSpace0, byDim / 2 + 1.0 * initSpace0 / 2.0, 2.0 * bzDim);
    sysSPH.SetBoundaries(cMin, cMax);

    // Create Fluid region and discretize with SPH particles
    ChVector3d boxCenter(-bxDim / 2 + fxDim / 2, 0.0, fzDim / 2);
    ChVector3d boxHalfDim(fxDim / 2 - initSpace0, fyDim / 2, fzDim / 2 - initSpace0);

    // Use a chrono sampler to create a bucket of points
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    chrono::utils::ChGenerator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    size_t numPart = points.size();
    double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
    for (int i = 0; i < numPart; i++) {
        // Calculate the pressure of a steady state (p = rho*g*h)
        auto pre_ini = sysSPH.GetDensity() * gz * (-points[i].z() + fzDim);
        auto rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed());
        sysSPH.AddSPHParticle(points[i], rho_ini, pre_ini, sysSPH.GetViscosity());
    }

    // Create container and attach BCE SPH particles
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);

    ground->AddVisualShape(chrono_types::make_shared<ChVisualShapeBox>(
                               ChVector3d(bxDim + 4 * initSpace0, byDim + 0 * initSpace0, 2 * initSpace0)),
                           ChFrame<>(ChVector3d(0, 0, -initSpace0), QUNIT));

    ground->AddVisualShape(
        chrono_types::make_shared<ChVisualShapeBox>(ChVector3d(2 * initSpace0, byDim, bzDim + 4 * initSpace0)),
        ChFrame<>(ChVector3d(+bxDim / 2 + initSpace0, 0, bzDim / 2), QUNIT));

    ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(
                                  cmaterial, ChVector3d(2 * initSpace0, byDim, bzDim + 4 * initSpace0)),
                              ChFrame<>(ChVector3d(+bxDim / 2 + initSpace0, 0, bzDim / 2), QUNIT));

    ground->AddVisualShape(
        chrono_types::make_shared<ChVisualShapeBox>(ChVector3d(2 * initSpace0, byDim, bzDim + 4 * initSpace0)),
        ChFrame<>(ChVector3d(-bxDim / 2 - initSpace0, 0, bzDim / 2), QUNIT));
    ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(
                                  cmaterial, ChVector3d(2 * initSpace0, byDim, bzDim + 4 * initSpace0)),
                              ChFrame<>(ChVector3d(-bxDim / 2 - initSpace0, 0, bzDim / 2), QUNIT));

    sysMBS.AddBody(ground);

    sysSPH.AddBoxContainerBCE(ground,                                         //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(2, 0, -1));

    // Add a box on the left and the right
    auto left_box = chrono_types::make_shared<ChBody>();
    left_box->SetPos(ChVector3d(-bxDim / 2 - bxDim / 2 - 3 * initSpace0, 0, bzDim + initSpace0));
    left_box->SetFixed(true);
    left_box->EnableCollision(true);
    left_box->AddVisualShape(chrono_types::make_shared<ChVisualShapeBox>(ChVector3d(bxDim, byDim, 2 * initSpace0)),
                             ChFrame<>());
    left_box->AddCollisionShape(
        chrono_types::make_shared<ChCollisionShapeBox>(cmaterial, ChVector3d(bxDim, byDim, 2 * initSpace0)),
        ChFrame<>());
    sysMBS.AddBody(left_box);
    sysFSI.AddFsiBody(left_box);
    sysSPH.AddBoxBCE(left_box, ChFrame<>(), ChVector3d(bxDim, byDim, 2 * initSpace0), true);

    auto right_box = chrono_types::make_shared<ChBody>();
    right_box->SetPos(ChVector3d(bxDim / 2 + bxDim / 2 + 3 * initSpace0, 0, bzDim + initSpace0));
    right_box->SetFixed(true);
    right_box->EnableCollision(true);
    right_box->AddVisualShape(chrono_types::make_shared<ChVisualShapeBox>(ChVector3d(bxDim, byDim, 2 * initSpace0)),
                              ChFrame<>());
    right_box->AddCollisionShape(
        chrono_types::make_shared<ChCollisionShapeBox>(cmaterial, ChVector3d(bxDim, byDim, 2 * initSpace0)),
        ChFrame<>());
    sysMBS.AddBody(right_box);
    sysFSI.AddFsiBody(right_box);
    sysSPH.AddBoxBCE(right_box, ChFrame<>(), ChVector3d(bxDim, byDim, 2 * initSpace0), true);

    // First plate
    ChVector3d plate_size1(0.9 * fxDim / 2, 0.7 * fyDim, 4 * initSpace0);
    ChVector3d plate_center1(-bxDim / 4, 0, fzDim + plate_size1.z() * 0.5);
    double plate_density = 400;
    double plate_volume1 = plate_size1.x() * plate_size1.y() * plate_size1.z();
    double plate_mass1 = plate_volume1 * plate_density;
    ChVector3d plate_inertia1(
        (1.0 / 12.0) * plate_mass1 * (plate_size1.y() * plate_size1.y() + plate_size1.z() * plate_size1.z()),  // I_xx
        (1.0 / 12.0) * plate_mass1 * (plate_size1.x() * plate_size1.x() + plate_size1.z() * plate_size1.z()),  // I_yy
        (1.0 / 12.0) * plate_mass1 * (plate_size1.x() * plate_size1.x() + plate_size1.y() * plate_size1.y())   // I_zz
    );
    auto plate_bottom1 = chrono_types::make_shared<ChBody>();
    plate_bottom1->SetPos(plate_center1);
    plate_bottom1->SetFixed(false);
    plate_bottom1->EnableCollision(true);

    auto plate_collision_shape = chrono_types::make_shared<ChCollisionShapeBox>(cmaterial, plate_size1);
    plate_bottom1->AddCollisionShape(plate_collision_shape, ChFrame<>());
    auto plate_visualization_shape = chrono_types::make_shared<ChVisualShapeBox>(plate_size1);
    plate_bottom1->AddVisualShape(plate_visualization_shape, ChFrame<>());
    plate_bottom1->SetMass(plate_mass1);
    plate_bottom1->SetInertiaXX(plate_inertia1);
    sysMBS.AddBody(plate_bottom1);
    sysFSI.AddFsiBody(plate_bottom1);
    sysSPH.AddBoxBCE(plate_bottom1, ChFrame<>(), plate_size1, true);

    // Second plate
    ChVector3d plate_size2(0.9 * fxDim / 2, 0.7 * fyDim, 4 * initSpace0);
    ChVector3d plate_center2(bxDim / 4, 0, fzDim + plate_size2.z() * 0.5);
    double plate_volume2 = plate_size2.x() * plate_size2.y() * plate_size2.z();
    double plate_mass2 = plate_volume2 * plate_density;
    ChVector3d plate_inertia2(
        (1.0 / 12.0) * plate_mass2 * (plate_size2.y() * plate_size2.y() + plate_size2.z() * plate_size2.z()),  // I_xx
        (1.0 / 12.0) * plate_mass2 * (plate_size2.x() * plate_size2.x() + plate_size2.z() * plate_size2.z()),  // I_yy
        (1.0 / 12.0) * plate_mass2 * (plate_size2.x() * plate_size2.x() + plate_size2.y() * plate_size2.y())   // I_zz
    );
    auto plate_bottom2 = chrono_types::make_shared<ChBody>();

    plate_bottom2->SetPos(plate_center2);
    plate_bottom2->SetFixed(false);
    plate_bottom2->EnableCollision(true);

    plate_bottom2->AddCollisionShape(plate_collision_shape, ChFrame<>());
    plate_bottom2->AddVisualShape(plate_visualization_shape, ChFrame<>());
    plate_bottom2->SetMass(plate_mass2);
    plate_bottom2->SetInertiaXX(plate_inertia2);
    sysMBS.AddBody(plate_bottom2);
    sysFSI.AddFsiBody(plate_bottom2);
    sysSPH.AddBoxBCE(plate_bottom2, ChFrame<>(), plate_size2, true);

    ChVector3d veh_init_pos(-bxDim / 2 - bxDim / 3, 0, 1.8);
    auto vehicle = CreateVehicle(sysMBS, sysSPH, sysFSI, ChCoordsys<>(veh_init_pos, QUNIT));

    // Now add a body to the system, a box representing the plate, floating on top of the water

    // Now add a body to the system, a box representing the plate, floating on top of the water
    // auto plate_top = chrono_types::make_shared<ChBody>();
    // plate_center.z() += plate_size.z() * 1.4;
    // plate_top->SetPos(plate_center);
    // plate_top->SetFixed(false);
    // plate_top->EnableCollision(true);
    // plate_top->SetMass(plate_mass);
    // plate_top->SetInertiaXX(plate_inertia);
    // geometry.CreateVisualizationAssets(plate_top, VisualizationType::COLLISION);
    // sysMBS.AddBody(plate_top);
    // sysFSI.AddFsiBody(plate_top);
    // sysSPH.AddBoxBCE(plate_top, ChFrame<>(VNULL, QNULL), plate_size, true);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Output directories
    std::string out_dir = GetChronoOutputPath() + "FSI_Fording_twoBlocks_" + std::to_string(ps_freq);

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    out_dir = out_dir + "/" + sysSPH.GetPhysicsProblemString() + "_" + sysSPH.GetSphMethodTypeString();

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
            std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
            std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
            return 1;
        }
    }
    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
            std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
            return 1;
        }
    }

    // Create driver ...Luning TODO: I actually don't need complicated path following ...
    std::cout << "Create path..." << std::endl;
    auto path = CreatePath(terrain_dir + "/path.txt");
    double target_speed = 1.0;
    ChPathFollowerDriver driver(*vehicle, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(2.0);
    driver.GetSteeringController().SetGains(1.0, 0, 0);
    driver.GetSpeedController().SetGains(0.6, 0.05, 0);
    driver.SetThrottle(0);
    driver.Initialize();

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);

        visFSI->SetTitle("Chrono::FSI Fording");
        visFSI->AddCamera(ChVector3d(bxDim / 2, -10 * byDim, 3.5 * bzDim), ChVector3d(0, 0, 0.4 * bzDim));
        visFSI->SetCameraMoveScale(1.0f);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(chrono_types::make_shared<VelocityColorCallback>(0, 5.0));
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Start the simulation
    DriverInputs driver_inputs = {0, 0, 0};
    double dT = sysFSI.GetStepSizeCFD();
    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    double x_max = bxDim / 2 + bxDim / 3;
    ChTimer timer;
    timer.start();
    while (time < t_end) {
        // Save data of the simulation
        if (output && time >= out_frame / output_fps) {
            // if (output) {
            std::cout << "------- OUTPUT" << std::endl;
            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);

            out_frame++;
        }

        // Ramp up throttle to value requested by the cruise controller
        if (time < 0.4) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        } else {
            driver_inputs.m_throttle = std::min(0.5 * time, 1.0);
            driver_inputs.m_braking = 0;
        }

        if (vehicle->GetPos().x() > x_max) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        }

        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!visFSI->Render())
                break;

            if (snapshots) {
                if (verbose)
                    std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics(dT);
        vehicle->Synchronize(time, driver_inputs);
        sysMBS.DoStepDynamics(dT);
        vehicle->Advance(dT);

        // std::cout << "num contacts: " << sysMBS.GetNumContacts() << std::endl;

        time += dT;
        sim_frame++;
    }
    timer.stop();
    std::cout << "End Time: " << t_end << std::endl;
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
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

std::shared_ptr<WheeledVehicle> CreateVehicle(ChSystemSMC& sys,
                                              ChFluidSystemSPH& sysSPH,
                                              ChFsiSystemSPH& sysFSI,
                                              const ChCoordsys<>& init_pos) {
    std::string vehicle_json = "Polaris/Polaris.json";
    std::string engine_json = "Polaris/Polaris_EngineSimpleMap.json";
    std::string transmission_json = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
    std::string tire_json = "Polaris/Polaris_RigidTire.json";

    // Create and initialize the vehicle
    auto vehicle = chrono_types::make_shared<WheeledVehicle>(&sys, vehicle::GetDataFile(vehicle_json));
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
        }
    }

    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.9f);
    cmaterial->SetRestitution(0.4f);
    std::string mesh_filename = vehicle::GetDataFile("Polaris/meshes/Polaris_tire_collision.obj");

    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(mesh_filename), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight
    auto wheel_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(cmaterial, trimesh, false, false, 0.005);

    // Create wheel BCE markers
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            std::vector<ChVector3d> points;
            wheel->GetSpindle()->AddCollisionShape(wheel_shape);
            wheel->GetSpindle()->EnableCollision(true);
            sysSPH.CreatePoints_Mesh(*trimesh, sysSPH.GetInitialSpacing(), points);
            sysSPH.AddPointsBCE(wheel->GetSpindle(), points, ChFrame<>(), true);
            sysFSI.AddFsiBody(wheel->GetSpindle());
        }
    }
    return vehicle;
}