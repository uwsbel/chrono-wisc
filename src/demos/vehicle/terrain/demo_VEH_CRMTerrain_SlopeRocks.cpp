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
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/core/ChGlobal.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_thirdparty/filesystem/path.h"

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
double terrain_length = 30;
double terrain_width = 3;

// Vehicle specification files
std::string vehicle_json = "Polaris/Polaris.json";
std::string engine_json = "Polaris/Polaris_EngineSimpleMap.json";
std::string transmission_json = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
std::string tire_json = "Polaris/Polaris_RigidTire.json";
////std::string tire_json = "Polaris/Polaris_ANCF4Tire_Lumped.json";

// Suspend vehicle
bool fix_chassis = false;

// Add rocks to the terrain
bool add_rocks = false;

// Save snapshots of the simulation
bool save_snapshots = false;

// ===================================================================================================================

std::shared_ptr<WheeledVehicle> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires);
void CreateFSIWheels(std::shared_ptr<WheeledVehicle> vehicle, CRMTerrain& terrain);
std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);
void AddRocksToTerrain(CRMTerrain& terrain, ChSystem& sys, double slope_angle = 20.0);

// ===================================================================================================================

int main(int argc, char* argv[]) {
    // ----------------
    // Problem settings
    // ----------------

    // Print usage information
    std::cout << "=== Wheeled Vehicle on CRM Terrain Demo ===" << std::endl;
    std::cout << "Usage: " << argv[0] << " [slope_angle] [rocks] [snapshots]" << std::endl;
    std::cout << "  [slope_angle]: 15, 20, 25, 30, or 40 (degrees, default: 20)" << std::endl;
    std::cout << "  [rocks]: 'rocks', '1', 'true', or 'yes' to enable rocks (default: disabled)" << std::endl;
    std::cout << "  [snapshots]: 'snapshots', '1', 'true', or 'yes' to save simulation snapshots (default: disabled)"
              << std::endl;
    std::cout << "Example: " << argv[0] << " 25 rocks snapshots" << std::endl;
    std::cout << "================================================" << std::endl;

    double target_speed = 4.0;
    double tend = 30;
    bool verbose = true;

    // Default slope angle (can be overridden by command line)
    double slope_angle = 25.0;

    // Parse command line arguments
    if (argc > 1) {
        try {
            double input_angle = std::stod(argv[1]);
            // Only allow specific angle values: 15, 20, 25, 30, and 40 degrees
            if (input_angle == 15.0 || input_angle == 20.0 || input_angle == 25.0 || input_angle == 30.0 ||
                input_angle == 40.0) {
                slope_angle = input_angle;
            } else {
                std::cout << "Warning: Slope angle must be one of: 15, 20, 25, 30, or 40 degrees." << std::endl;
                std::cout << "Using default value of 20 degrees." << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "Error parsing slope angle. Using default value of 20 degrees." << std::endl;
        }
    }

    // Check for rocks command line argument (second argument)
    if (argc > 2) {
        std::string rocks_arg = argv[2];
        if (rocks_arg == "rocks" || rocks_arg == "1" || rocks_arg == "true" || rocks_arg == "yes") {
            add_rocks = true;
            std::cout << "Rocks enabled: Vehicle will drive over rocks on the slope." << std::endl;
        } else {
            add_rocks = false;
            std::cout << "Rocks disabled: Vehicle will drive on smooth terrain." << std::endl;
        }
    }

    // Check for snapshots command line argument (third argument)
    if (argc > 3) {
        std::string snapshots_arg = argv[3];
        if (snapshots_arg == "snapshots" || snapshots_arg == "1" || snapshots_arg == "true" || snapshots_arg == "yes") {
            save_snapshots = true;
            std::cout << "Snapshots enabled: Simulation images will be saved." << std::endl;
        } else {
            save_snapshots = false;
            std::cout << "Snapshots disabled: No simulation images will be saved." << std::endl;
        }
    }

    // Print selected angle
    std::cout << "Selected slope angle: " << slope_angle << " degrees" << std::endl;

    // Visualization settings
    bool render = true;                    // use run-time visualization
    double render_fps = 200;               // rendering FPS
    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = false;  // render boundary BCE markers
    bool visualization_rigid_bce = true;   // render wheel BCE markers
    bool chase_cam = true;                 // chase-cam or fixed camera

    // CRM material properties
    double density = 1700;
    double cohesion = 5e3;
    double friction = 0.8;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    // CRM (moving) active box dimension
    double active_box_hdim = 0.4;

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
    terrain.SetActiveDomain(ChVector3d(active_box_hdim));

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
        case PatchType::HEIGHT_MAP: {
            double max_slope_distance = terrain_length / 2;  // This was how the bmp file was mad
            double angle = slope_angle;                      // Use the command line argument value
            double max_height = max_slope_distance * tan(angle * 3.14 / 180);
            std::string height_map_file =
                "terrain/height_maps/slope_" + std::to_string(static_cast<int>(angle)) + ".bmp";
            std::cout << "Height map file: " << height_map_file << std::endl;
            std::cout << "Using slope angle: " << angle << " degrees" << std::endl;
            // Create a patch from a heigh field map image
            terrain.Construct(vehicle::GetDataFile(height_map_file),  // height map image file
                              terrain_length, terrain_width,          // length (X) and width (Y)
                              {0, max_height},                        // height range
                              0.25,                                   // depth
                              true,                                   // uniform depth
                              ChVector3d(terrain_length / 2, 0, 0),   // patch center
                              BoxSide::Z_NEG                          // bottom wall
            );
            // Create straight line path
            path = StraightLinePath(ChVector3d(0, 0, vehicle_init_height),
                                    ChVector3d(terrain_length, 0, vehicle_init_height), 1);
        } break;
        case PatchType::MARKER_DATA:
            // Create a patch using SPH particles and BCE markers from files
            terrain.Construct(vehicle::GetDataFile("terrain/sph/S-lane_RMS/sph_particles.txt"),  // SPH marker locations
                              vehicle::GetDataFile("terrain/sph/S-lane_RMS/bce_markers.txt"),    // BCE marker locations
                              VNULL);
            // Create path from data file
            path = CreatePath("terrain/sph/S-lane_RMS/path.txt");
            break;
    }

    // Add rocks to the terrain if enabled
    if (add_rocks) {
        cout << "Adding rocks to terrain..." << endl;
        AddRocksToTerrain(terrain, *sysMBS, slope_angle);
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
    // Create the path-following driver
    // --------------------------------

    cout << "Create path..." << endl;
    ChPathFollowerDriver driver(*vehicle, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(2.0);
    driver.GetSteeringController().SetGains(1.0, 0, 0);
    driver.GetSpeedController().SetGains(0.6, 0.05, 0);
    driver.Initialize();

    // -----------------------------
    // Create run-time visualization
    // -----------------------------

#ifndef CHRONO_OPENGL
    if (vis_type == ChVisualSystem::Type::OpenGL)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::OpenGL;
#endif
#if !defined(CHRONO_OPENGL) && !defined(CHRONO_VSG)
    render = false;
#endif

    // Create output directory for snapshots if enabled
    std::string out_dir;
    if (save_snapshots) {
        out_dir = GetChronoOutputPath() + "VEH_CRMTerrain_SlopeRocks/";
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        // Create a subdirectory with simulation parameters
        std::string sim_name = "slope" + std::to_string(static_cast<int>(slope_angle));
        if (add_rocks) {
            sim_name += "_rocks";
        }
        out_dir = out_dir + sim_name;
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        // Create snapshots directory
        if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
            std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
            return 1;
        }

        std::cout << "Snapshots will be saved to: " << out_dir + "/snapshots" << std::endl;
    }

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
        visFSI->AddCamera(ChVector3d(0, 8, 1.5), ChVector3d(0, -1, 0));
        visFSI->SetCameraMoveScale(0.2f);
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(false);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        // visFSI->SetSPHColorCallback(
        //     chrono_types::make_shared<ParticleVelocityColorCallback>(ChColor(0.3f, 0.6f, 0.9f), 0, 1));
        visFSI->AttachSystem(sysMBS);
        visFSI->Initialize();
    }

    // ---------------
    // Simulation loop
    // ---------------

    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;

    cout << "Start simulation..." << endl;

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

        // Stop vehicle before reaching end of terrain patch
        if (veh_loc.x() > x_max) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        }

        // Run-time visualization
        if (render && time >= render_frame / render_fps) {
            if (chase_cam) {
                ChVector3d cam_loc = veh_loc + ChVector3d(-6, 6, 1.5);
                ChVector3d cam_point = veh_loc;
                visFSI->UpdateCamera(cam_loc, cam_point);
            }
            if (!visFSI->Render())
                break;

            // Save snapshot if enabled
            if (save_snapshots) {
                if (verbose)
                    std::cout << " -- Saving snapshot " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }

            render_frame++;
        }
        if (!render) {
            std::cout << time << "  " << terrain.GetRtfCFD() << "  " << terrain.GetRtfMBD() << std::endl;
        }

        // Synchronize systems
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle->Synchronize(time, driver_inputs, terrain);

        // Advance system state
        driver.Advance(step_size);
        timer.reset();
        timer.start();
        // (a) Sequential integration of terrain and vehicle systems
        ////terrain.Advance(step_size);
        ////vehicle->Advance(step_size);
        // (b) Concurrent integration (vehicle in main thread)
        ////std::thread th(&CRMTerrain::Advance, &terrain, step_size);
        ////vehicle->Advance(step_size);
        ////th.join();
        // (c) Concurrent integration (terrain in main thread)
        std::thread th(&ChWheeledVehicle::Advance, vehicle.get(), step_size);
        terrain.Advance(step_size);
        th.join();

        // Set correct overall RTF for the FSI problem
        timer.stop();
        double rtf = timer() / step_size;
        sysFSI.SetRtf(rtf);
        time += step_size;
        sim_frame++;
    }

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

void CreateFSIWheels(std::shared_ptr<WheeledVehicle> vehicle, CRMTerrain& terrain) {
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

// Function to add rocks to the terrain
void AddRocksToTerrain(CRMTerrain& terrain, ChSystem& sys, double slope_angle) {
    // Rock mesh files
    std::vector<std::string> rock_meshfiles = {"robot/curiosity/rocks/rock1.obj", "robot/curiosity/rocks/rock2.obj",
                                               "robot/curiosity/rocks/rock3.obj"};

    // Calculate height based on slope angle and terrain configuration
    double slope_angle_rad = slope_angle * CH_DEG_TO_RAD;
    double max_slope_distance = terrain_length / 2;  // This is how the bmp file was made
    double flat_portion_length = terrain_length - max_slope_distance;

    // Rock positions - positioned to ensure right wheel hits first, then left wheel
    // Positions are along the slope at different x positions
    // We need to account for the flat portion before the slope begins
    double x1 = 18.0;  // Position on the slope
    double x2 = 19.0;  // Position on the slope

    // If x >= flat_portion_length, height is (x - flat_portion_length) * tan(angle)
    double z1 = (x1 > flat_portion_length) ? (x1 - flat_portion_length) * tan(slope_angle_rad) : 0.0;
    double z2 = (x2 > flat_portion_length) ? (x2 - flat_portion_length) * tan(slope_angle_rad) : 0.0;

    // Offset to adjust
    double height_offset_1 = 0.8;
    double height_offset_2 = 0.85;
    std::vector<ChVector3d> rock_positions = {
        ChVector3d(x1, 0.5, z1 + height_offset_1),  // Right side rock (first to be hit)
        ChVector3d(x2, -0.5, z2 + height_offset_2)  // Left side rock
    };

    // Rock scales - adjusted for better visibility
    std::vector<double> rock_scales = {0.4, 0.35};

    // Rock density
    double rock_density = 2500;  // kg/m^3

    // Create contact material for rocks
    std::shared_ptr<ChContactMaterial> rock_material = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());

    // Create rocks
    for (size_t i = 0; i < rock_positions.size(); i++) {
        // Select rock mesh (cycle through available meshes)
        std::string mesh_file = rock_meshfiles[i % rock_meshfiles.size()];

        // Load the mesh
        auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(mesh_file), false, true);
        mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(rock_scales[i]));
        mesh->RepairDuplicateVertexes(1e-9);  // Ensure mesh is watertight

        // Compute mass properties
        double mass;
        ChVector3d cog;
        ChMatrix33<> inertia;
        mesh->ComputeMassProperties(true, mass, cog, inertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector3d principal_I;
        ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

        // Create the rock body
        auto rock_body = chrono_types::make_shared<ChBodyAuxRef>();
        rock_body->SetName("rock_" + std::to_string(i));
        rock_body->SetPos(rock_positions[i]);
        rock_body->SetRot(QUNIT);
        rock_body->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));
        rock_body->SetMass(mass * rock_density);
        rock_body->SetInertiaXX(rock_density * principal_I);
        rock_body->SetFixed(true);  // Fix rocks to the ground
        sys.Add(rock_body);

        // Add collision shape
        auto collision_shape =
            chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rock_material, mesh, false, false, 0.005);
        rock_body->AddCollisionShape(collision_shape);
        rock_body->EnableCollision(true);

        // Add visualization
        auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        vis_shape->SetMesh(mesh);
        vis_shape->SetBackfaceCull(true);
        rock_body->AddVisualShape(vis_shape);

        // Add rock to FSI system with BCE markers
        utils::ChBodyGeometry geometry;
        geometry.coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, mesh, VNULL));
        terrain.AddRigidBody(rock_body, geometry, false);

        cout << "  Added rock " << i << " at position " << rock_positions[i] << endl;
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
