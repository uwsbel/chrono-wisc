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

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_models/vehicle/artcar/ARTcar.h"

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
double terrain_length = 7;
double terrain_width = 5;

// Vehicle initial position
double vehicle_init_x = 0;
double vehicle_init_y = 0;
double vehicle_init_z = 0.2;

// Suspend vehicle
bool fix_chassis = false;

// ===================================================================================================================

std::tuple<std::shared_ptr<artcar::ARTcar>, std::shared_ptr<ChBody>, std::shared_ptr<ChLinkMotorRotationAngle>> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires);
void CreateFSIWheels(std::shared_ptr<artcar::ARTcar> vehicle, CRMTerrain& terrain);
void CreateFSIBlade(std::shared_ptr<ChBody> blade, CRMTerrain& terrain);
std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);

// ===================================================================================================================

int main(int argc, char* argv[]) {
    // ---------------- 
    // Problem settings
    // ----------------

    double target_speed = 5.0;
    double tend = 30;
    bool verbose = true;

    // Visualization settings
    bool render = true;                    // use run-time visualization
    double render_fps = 200;               // rendering FPS
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
    double active_box_hdim = 0.4;
    double settling_time = 0;

    // Set SPH spacing
    double spacing = 0.02;

    // --------------
    // Create vehicle
    // --------------

    // TODO: add heading angle to the vehicle initialization
    ChQuaterniond q180_z(0., 0., 0., 1.);

    cout << "Create vehicle..." << endl;
    bool fea_tires;
    auto [vehicle, blade, motor] = CreateVehicle(ChCoordsys<>(ChVector3d(vehicle_init_x, vehicle_init_y, vehicle_init_z), q180_z), fea_tires);
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
            // Create a rectangular terrain patch
            terrain.Construct({terrain_length, terrain_width, 0.25},  // length X width X height
                              ChVector3d(0, 0, 0),   // patch center
                              BoxSide::ALL & ~BoxSide::Z_POS          // all boundaries, except top
            );
            break;
        case PatchType::HEIGHT_MAP:
            // Create a patch from a heigh field map image
            terrain.Construct(vehicle::GetDataFile("terrain/art/padded_image_0.bmp"),   // height map image file
                              terrain_length, terrain_width,                           // length (X) and width (Y)
                              {0, 0.3},                                                // height range
                              0.2,                                                     // depth
                              true,                                                    // uniform depth
                              ChVector3d(0, 0, 0),                                     // patch center
                              BoxSide::Z_NEG                                           // bottom wall
            );
            break;
        case PatchType::MARKER_DATA:
            // Create a patch using SPH particles and BCE markers from files
            terrain.Construct(vehicle::GetDataFile("terrain/sph/S-lane_RMS/sph_particles.txt"),  // SPH marker locations
                              vehicle::GetDataFile("terrain/sph/S-lane_RMS/bce_markers.txt"),    // BCE marker locations
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
    // Create the driver (Path follower)
    // --------------------------------

    // ChDriver driver(vehicle->GetVehicle());
    ChWheeledVehicle& vehicle_ptr = vehicle->GetVehicle();
    std::shared_ptr<ChBezierCurve> path = CreatePath("terrain/art/wpts_data/wpts_0_1.txt");
    cout << "Path created (main)" << endl;
    cout << "Create driver..." << endl;
    ChPathFollowerDriver driver(vehicle_ptr, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(2);
    driver.GetSteeringController().SetGains(0.9, 0, 0);
    driver.GetSpeedController().SetGains(0.6, 0.0, 0);
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
        visFSI->AddCamera(ChVector3d(0, -8, 2.5), ChVector3d(0, -1, 0));
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
            vis_vsg->SetClearColor(ChColor(0.8f, 0.85f, 0.9f)); // Set background to light blue
        }
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
        const auto& veh_loc = vehicle->GetVehicle().GetPos();

        // Set current driver inputs
        auto driver_inputs = driver.GetInputs();
        // cout << "Current inputs: Steering = " << driver_inputs.m_steering << ", Throttle = " << driver_inputs.m_throttle << ", Braking = " << driver_inputs.m_braking << ", Clutch = " << driver_inputs.m_clutch << endl;

        // Ramp up throttle to 1 from 0 for 0.5s
        if (time < 0.5) {
            driver_inputs.m_throttle = time / 0.5;
        } else {
            driver_inputs.m_throttle = 1;
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
            render_frame++;
        }
        if (!render) {
            std::cout << time << "  " << terrain.GetRtfCFD() << "  " << terrain.GetRtfMBD() << std::endl;
        }

        // // test motor function working
        // auto rot = chrono_types::make_shared<ChFunctionConst>(0.5);
        // motor->SetAngleFunction(rot);

        // Synchronize systems
        driver.Synchronize(time);
        terrain.Synchronize(time);

        // Get driver inputs
        vehicle->Synchronize(time, driver_inputs, terrain);

        // Advance system state
        driver.Advance(step_size);
        timer.reset();
        timer.start();

        // (a) Sequential integration of terrain and vehicle systems
        terrain.Advance(step_size);
        vehicle->Advance(step_size);
        // (b) Concurrent integration (vehicle in main thread)
        // std::thread th(&CRMTerrain::Advance, &terrain, step_size);
        // vehicle->Advance(step_size);
        // th.join();
        // (c) Concurrent integration (terrain in main thread)
        // std::thread th(&ChWheeledVehicle::Advance, vehicle->GetVehicle().get(), step_size);
        // terrain.Advance(step_size);
        // th.join();
        
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

std::tuple<std::shared_ptr<artcar::ARTcar>, std::shared_ptr<ChBody>, std::shared_ptr<ChLinkMotorRotationAngle>> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires) {
    fea_tires = false;

    auto vehicle = chrono_types::make_shared<artcar::ARTcar>();
    vehicle->SetContactMethod(ChContactMethod::SMC);
    vehicle->SetChassisFixed(false);
    vehicle->SetInitPosition(init_pos);
    vehicle->SetTireType(TireModelType::RIGID_MESH);
    vehicle->SetMaxMotorVoltageRatio(0.2);
    vehicle->SetStallTorque(1);
    vehicle->SetTireRollingResistance(0.06);
    vehicle->Initialize();
    vehicle->LockAxleDifferential(1, true);
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetTireVisualizationType(VisualizationType::MESH);
    vehicle->SetSuspensionVisualizationType(VisualizationType::MESH);
    vehicle->SetSteeringVisualizationType(VisualizationType::MESH);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    auto contact_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto blade = chrono_types::make_shared<ChBodyEasyMesh>(vehicle::GetDataFile("artcar/blade_final.obj"), 1000, true, false, false);
    auto offsetpos = ChVector3d(0.5, 0, 0.05);
    blade->SetPos(vehicle->GetChassisBody()->TransformPointLocalToParent(offsetpos));
    blade->SetRot(vehicle->GetChassisBody()->GetRot() * Q_ROTATE_Y_TO_X * QuatFromAngleX(-CH_PI_2));
    blade->SetMass(0.1);
    blade->SetFixed(false);
    
    vehicle->GetSystem()->AddBody(blade);

    // create motor and apply it to the blade
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(vehicle->GetChassisBody(), blade, ChFrame<>(blade->GetPos(), vehicle->GetChassisBody()->GetRot()));
    vehicle->GetSystem()->Add(motor);

    return std::make_tuple(vehicle, blade, motor);
}

void CreateFSIWheels(std::shared_ptr<artcar::ARTcar> vehicle, CRMTerrain& terrain) {
    std::string mesh_filename = vehicle::GetDataFile("artcar/tire_collision.obj");
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
    // Create geometry for the blade using its mesh
    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(ChContactMaterialData());
    
    // Use the same mesh file that was used to create the blade
    std::string mesh_filename = vehicle::GetDataFile("artcar/blade_final.obj");
    geometry.coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, mesh_filename, VNULL));
    
    // Add the blade as a rigid body to the FSI system
    size_t num_blade_BCE = terrain.AddRigidBody(blade, geometry, false);
    cout << "Added " << num_blade_BCE << " BCE markers on blade" << endl;
}

std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file) {
    cout << "Create path from " << path_file << "..." << endl;
    
    // Default path (fallback in case of errors)
    std::vector<ChVector3d> default_points = {
        ChVector3d(0, 0, 0.2),
        ChVector3d(10, 0, 0.2)
    };
    
    // Try to open the file
    std::string full_path = vehicle::GetDataFile(path_file);
    std::ifstream ifile(full_path);
    if (!ifile.is_open()) {
        cout << "ERROR: Failed to open path file: " << full_path << endl;
        cout << "Using default straight path" << endl;
        return std::shared_ptr<ChBezierCurve>(new ChBezierCurve(default_points));
    }
    
    // Each file will have exactly 10 points
    std::vector<ChVector3d> points;
    points.reserve(10);
    std::string line;
    
    // Read points with only x and y coordinates, z will always be 0.1
    while (std::getline(ifile, line)) {
        double x, y;
        std::istringstream iss(line);
        if (iss >> x >> y) {
            // Apply offsets to x and y, and set z to fixed value 0.1
            points.push_back(ChVector3d(x - 2.5, y - 1.5, 0.1));
        }
    }
    
    // Verify we have enough points
    if (points.size() < 2) {
        cout << "ERROR: Not enough valid points in path file" << endl;
        return std::shared_ptr<ChBezierCurve>(new ChBezierCurve(default_points));
    }
    
    for (size_t i = 0; i < points.size(); i++) {
        cout << "Point " << i + 1 << ": " << points[i].x() << ", " << points[i].y() << ", " << points[i].z() << endl;
    }
    
    cout << "Successfully read " << points.size() << " path points" << endl;
    ifile.close();
    
    return std::shared_ptr<ChBezierCurve>(new ChBezierCurve(points));
}
