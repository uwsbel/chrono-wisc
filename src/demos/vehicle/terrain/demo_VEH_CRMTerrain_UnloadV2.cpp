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

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkLock.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "demos/SetChronoSolver.h"
#include "demos/vehicle/WheeledVehicleJSON.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire3443B.h"

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
PatchType patch_type = PatchType::RECTANGULAR;

// Terrain dimensions (for RECTANGULAR or HEIGHT_MAP patch type)
double terrain_length = 20;
double terrain_width = 3;

// Vehicle specification files
std::string vehicle_json = "Polaris/Polaris_LTV.json";
std::string engine_json = "Polaris/Polaris_EngineSimpleMap.json";
std::string transmission_json = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
std::string tire_json = "Polaris/Polaris_RigidMeshTire.json";
////std::string tire_json = "Polaris/Polaris_ANCF4Tire_Lumped.json";

// Suspend vehicle
bool fix_chassis = false;

// ===================================================================================================================

std::tuple<std::shared_ptr<WheeledVehicle>,std::shared_ptr<WheeledTrailer>, std::shared_ptr<ChLinkMotorRotationAngle>, std::shared_ptr<ChLinkMotorRotationAngle>, std::shared_ptr<ChBodyEasyBox>, std::shared_ptr<ChBodyEasyBox>> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires);
void CreateFSIWheels(std::shared_ptr<WheeledVehicle> vehicle, std::shared_ptr<WheeledTrailer> trailer, CRMTerrain& terrain);
std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);

// ===================================================================================================================
class ChFunctionTimeShift : public chrono::ChFunction {
    public:
        ChFunctionTimeShift(std::shared_ptr<chrono::ChFunction> inner_func, double time_shift, double max_val)
            : m_inner_func(inner_func), m_time_shift(time_shift), m_max_val(max_val) {}
    
        virtual double GetVal(double x) const override {
            double v = m_inner_func->GetVal(x - m_time_shift);
            return (v > m_max_val) ? m_max_val : v;
        }
    
        virtual ChFunction* Clone() const override {
            return new ChFunctionTimeShift(*this);
        }

        void SetMax(double max_val) {
            m_max_val = max_val;
        }

    private:
        std::shared_ptr<chrono::ChFunction> m_inner_func;
        double m_time_shift;
        double m_max_val;
    };
    
int main(int argc, char* argv[]) {
    // ---------------- 
    // Problem settings
    // ----------------

    double target_speed = 7.0;
    double tend = 30;
    bool verbose = true;

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
    double youngs_modulus = 9e5;
    double poisson_ratio = 0.3;

    // CRM (moving) active box dimension
    double active_box_hdim = 0.4;

    // Set SPH spacing
    double spacing = (patch_type == PatchType::MARKER_DATA) ? 0.02 : 0.04;

    // gravity
    double tilt_angle_deg = 0.0;
    double tilt_radians = (tilt_angle_deg) * CH_DEG_TO_RAD;
    double current_angle = (tilt_radians);
    ChVector3d new_gravity = ChVector3d(0,0,-1.6); //ChVector3d(-std::sin(current_angle),0, -std::cos(current_angle)) * 1.69;
    // --------------
    // --------------

    cout << "Create vehicle..." << endl;
    double vehicle_init_height = 0.4;
    bool fea_tires;
    auto [vehicle,trailer, motor_1, motor_2, rod2, hanging_box] = CreateVehicle(ChCoordsys<>(ChVector3d(4.5, 0, vehicle_init_height), QUNIT), fea_tires);
    vehicle->GetChassis()->SetFixed(fix_chassis);
    auto sysMBS = vehicle->GetSystem();
    sysMBS->SetGravitationalAcceleration(new_gravity);

    // ---------------------------------
    // Set solver and integrator for MBD
    // ---------------------------------

    double step_size = 0;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;

    
    step_size = 1e-3;
    solver_type = ChSolver::Type::SPARSE_LU;
    integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    
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
    terrain.SetGravitationalAcceleration(new_gravity);
    terrain.SetStepSizeCFD(step_size);

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

    terrain.DisableMBD();

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
    // Add vehicle wheels as FSI solids
    
    CreateFSIWheels(vehicle,trailer, terrain);


    // Initialize the terrain system
    terrain.Initialize();

    terrain.SetActiveDomain(ChVector3d(active_box_hdim));
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
        visFSI->AddCamera(ChVector3d(-5, 0, 1.5), ChVector3d(5.0, 0, 0));
        visFSI->SetCameraMoveScale(0.2f);
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(chrono_types::make_shared<ParticleHeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f),
                                                                                           aabb.min.z(), aabb.max.z()));
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
    bool attached_yet = false;
    auto arm_lock = chrono_types::make_shared<ChLinkLockLock>();
    ChTimer timer;

    std::shared_ptr<ChFunctionConst> func_const = chrono_types::make_shared<ChFunctionConst>(-0.2);
    //auto func_ramp = chrono_types::make_shared<ChFunctionRamp>(-0.2, 0.78);
    //auto func_offset = chrono_types::make_shared<ChFunctionSequence>();
    //func_offset->SetOffsetTime(-5.1); // aligns ramp start with t = 5.1
    //func_offset->SetFunction(func_ramp);
    // From -0.2 to 0.78 over 1 second, starting at t = 5.1
    double h = 0.0;
    double final = 0.78;
    double t_start = 5.1;
    double duration = 1.0;
    double w = (final - h) / duration;
    
    auto ramp_func = chrono_types::make_shared<ChFunctionRamp>(h, w);
    auto shifted_func = chrono_types::make_shared<ChFunctionTimeShift>(ramp_func, t_start, final);
    
    double h2 = 0.0;
    double final2 = 1.57;
    double t_start2 = 6;
    double duration2 = 1.0;
    double w2 = (final2 - h2) / duration2;
    
    auto ramp_func_2 = chrono_types::make_shared<ChFunctionRamp>(h2, w2);
    auto shifted_func_2 = chrono_types::make_shared<ChFunctionTimeShift>(ramp_func_2, t_start2, final2);
    
    //motor_2->SetAngleFunction(shifted_func);
    arm_lock->Initialize(rod2, hanging_box, ChFrame<>(hanging_box->GetPos()));
    sysMBS->Add(arm_lock);
    arm_lock->SetDisabled(true);

    while (time < tend) {
        const auto& veh_loc = vehicle->GetPos();
        
        // set offset gravity
        //sysMBS->SetGravitationalAcceleration(new_gravity);
        //terrain.SetGravitationalAcceleration(new_gravity);

        // Set current driver inputs
        auto driver_inputs = driver.GetInputs();
        driver_inputs.m_throttle = 0.0;

        // Ramp up throttle to value requested by the cruise controller
        if (time < 0.1) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        } 
        else if (time < 5.0 && time >= 0.1) {  
            driver_inputs.m_throttle = 1;
            driver_inputs.m_braking = 0;
        } 
        else if (time > 5 && time < 5.1) {
            motor_2->SetAngleFunction(shifted_func);
        }
        else if (time > 5.1 && time < 6) {
            if (!attached_yet) {
                //sysMBS->Add(arm_lock);
                arm_lock->SetDisabled(false);
                attached_yet = true;
            }
            motor_2->SetAngleFunction(shifted_func);
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        }
        else if (time > 6 && time < 7.5) {
            motor_1->SetAngleFunction(shifted_func_2);            
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        }
        else{
            //arm_lock->SetBroken(true);
            arm_lock->SetDisabled(true);
            //arm_lock->GetMask().SetAllDisabled(true);
            //sysMBS->RemoveLink(arm_lock);
            //sysMBS->RemoveBody(hanging_box);
            
            //std::cout << "Disabled lock" << std::endl; 
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        }
        // Run-time visualization TODO: Fix the render frame calculation.
        if (render && time >= render_frame / render_fps) {
            if (chase_cam) {
                ChVector3d cam_loc = veh_loc + ChVector3d(-6, 6, 1.5);
                ChVector3d cam_point = veh_loc;
                visFSI->UpdateCamera(cam_loc, cam_point);
            }
            if (!visFSI->Render())
                break;
            // Write image to file
            std::ostringstream filename;
            filename << "./snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1 << ".png";
            visFSI->GetVisualSystem()->WriteImageToFile(filename.str());

            render_frame++;
        }
        if (!render) {
            std::cout << time << "  " << terrain.GetRtfCFD() << "  " << terrain.GetRtfMBD() << std::endl;
        }

        // Synchronize systems
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle->Synchronize(time, driver_inputs, terrain);
        if(trailer!=nullptr) {
            trailer->Synchronize(time, driver_inputs, terrain);
        }
        // Advance system state
        driver.Advance(step_size);

        timer.reset();
        timer.start();
        // (a) Sequential integration of terrain and vehicle systems
        terrain.Advance(step_size);
        vehicle->Advance(step_size);
        // (b) Concurrent integration (vehicle in main thread)
        ////std::thread th(&CRMTerrain::Advance, &terrain, step_size);
        ////vehicle->Advance(step_size);
        ////th.join();
        // (c) Concurrent integration (terrain in main thread)
        //std::thread th(&ChWheeledVehicle::Advance, vehicle.get(), step_size);
        //terrain.Advance(step_size);
        //th.join();
        //vehicle->GetSystem()->DoStepDynamics(step_size);

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

std::tuple<std::shared_ptr<WheeledVehicle>,std::shared_ptr<WheeledTrailer>, std::shared_ptr<ChLinkMotorRotationAngle>, std::shared_ptr<ChLinkMotorRotationAngle>, std::shared_ptr<ChBodyEasyBox>, std::shared_ptr<ChBodyEasyBox>> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires) {
    fea_tires = false;
    bool use_airless = true;
    bool set_straight_spokes = false;

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
    if(!use_airless) {
        for (auto& axle : vehicle->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
                vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
                if (std::dynamic_pointer_cast<ChDeformableTire>(tire))
                    fea_tires = true;
            }
        }
    } else {
        for (auto& axle : vehicle->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                std::cout <<".... " << std::endl;
                auto tire = chrono_types::make_shared<ANCFAirlessTire3443B>("Airless3443B");
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetRimRadius(0.19);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHeight(0.19);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetWidth(0.25);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetAlpha(0.05);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetOuterRingThickness(0.015);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusSpokes(8e5);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusOuterRing(1e9);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetPoissonsRatio(0.3);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivWidth(3);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivSpokeLength(3);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivOuterRingPerSpoke(3);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetNumberSpokes(16);
                //std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetContactMaterial(contactMat);
                    
                
                if (set_straight_spokes) {
                    std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHubRelativeRotation(0);
                    std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetSpokeCurvatureXPoint(0);
                    std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetSpokeCurvatureZPoint(0);
                }
                vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
                if (std::dynamic_pointer_cast<ChDeformableTire>(tire))
                    fea_tires = true;
            }
        }
    }

    //std::string trailer_file = vehicle::GetDataFile("LRV_Wagon/Polaris.json");
    //auto trailer = chrono_types::make_shared<WheeledTrailer>(vehicle->GetSystem(), trailer_file);
    auto trailer_model = UT_Model();
    auto trailer = chrono_types::make_shared<WheeledTrailer>(vehicle->GetSystem(), vehicle::GetDataFile(trailer_model.TrailerJSON())); 
    //auto trailer = nullptr;

    trailer->Initialize(vehicle->GetChassis());
    trailer->GetChassis()->SetFixed(false);
    trailer->GetChassis()->EnableCollision(true);
    trailer->SetChassisVisualizationType(VisualizationType::MESH);
    trailer->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    trailer->SetWheelVisualizationType(VisualizationType::MESH);
    for (auto& axle : trailer->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
            trailer->InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }


    auto contact_mat = chrono_types::make_shared<ChContactMaterialSMC>();

    // Set the offset position of where to put the rod
    auto offset_spot = vehicle->GetChassisBody()->GetPos() + ChVector3d(-1.2, 0, 0.3);
    
    // set collision shape for trailer
    auto col_trailer = chrono_types::make_shared<ChBodyEasyBox>(1.2, 1.2, 0.1, 1000, true, true, contact_mat);
    // auto col_trailer = chrono_types::make_shared<ChBodyEasyMesh>(vehicle::GetDataFile("LRV_Wagon/trailer_col.obj"), 
    //                                                              1000, 
    //                                                              true, 
    //                                                              true, true, contact_mat);
    ChVector3d col_trailer_pos = trailer->GetChassis()->GetBody()->GetPos() + ChVector3d(0, 0, 0);
    col_trailer->SetPos(col_trailer_pos);
    vehicle->GetSystem()->Add(col_trailer);
    // lock the collision shape to the trailer
    auto lock_trailer = chrono_types::make_shared<ChLinkLockLock>();
    lock_trailer->SetName("lock_trailer");
    lock_trailer->Initialize(trailer->GetChassis()->GetBody(), col_trailer, ChFrame<>(col_trailer_pos));
    vehicle->GetSystem()->Add(lock_trailer);

    // Initialize the first rod
    double rod_length = 1.4;
    double rod_angle = 1.04f;  // 60 degrees
    auto rod_1 = chrono_types::make_shared<ChBodyEasyBox>(rod_length, 0.1, 0.1, 1000, true, false, contact_mat);
    rod_1->SetPos(offset_spot + ChVector3d(-rod_length / 2 * std::cos(rod_angle), 0, rod_length / 2 * std::sin(rod_angle)));
    rod_1->SetRot(QuatFromAngleY(rod_angle));
    rod_1->SetFixed(false);
    vehicle->GetSystem()->Add(rod_1);

    // Initialize the second rod
    auto jointpos_rod12 = offset_spot + ChVector3d(-rod_length * std::cos(rod_angle), 0, rod_length * std::sin(rod_angle));
    auto rod_2 = chrono_types::make_shared<ChBodyEasyBox>(rod_length, 0.1, 0.1, 1000, true, false, contact_mat);
    rod_2->SetPos(offset_spot + ChVector3d(-3.0 * rod_length / 2 * std::cos(rod_angle), 0, rod_length / 2 * std::sin(rod_angle)));
    rod_2->SetRot(QuatFromAngleY(-rod_angle));
    rod_2->SetFixed(false);
    vehicle->GetSystem()->Add(rod_2);

    // Initialize the second body
    auto hanging_box = chrono_types::make_shared<ChBodyEasyBox>(0.4, 0.4, 0.3, 800, true, true, contact_mat);
    auto hanging_box_pos = offset_spot + ChVector3d(-3.0 * rod_length * std::cos(rod_angle) + 0.6, 0, -0.3);
    hanging_box->SetPos(hanging_box_pos);
    hanging_box->SetMass(1.0);
    vehicle->GetSystem()->Add(hanging_box);

    // Create link between vehicle and rod1
    auto motor_1 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor_1->Initialize(vehicle->GetChassisBody(), rod_1, ChFrame<>(offset_spot));
    vehicle->GetSystem()->Add(motor_1);

    // Create link between rod1 and rod2
    auto motor_2 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor_2->Initialize(rod_1, rod_2, ChFrame<>(jointpos_rod12,Q_ROTATE_Y_TO_Z));
    vehicle->GetSystem()->Add(motor_2);

    // // Create lock link
    auto lock = chrono_types::make_shared<ChLinkLockLock>();
    lock->SetName("lock");
    lock->Initialize(rod_2, hanging_box, ChFrame<>(hanging_box_pos));
    vehicle->GetSystem()->Add(lock);

    return std::make_tuple(vehicle, trailer, motor_1, motor_2, rod_2, hanging_box);
}

void CreateFSIWheels(std::shared_ptr<WheeledVehicle> vehicle,std::shared_ptr<WheeledTrailer> trailer, CRMTerrain& terrain) {
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

    // add trailer wheels
    if(trailer != nullptr) {
        for (auto& axle : trailer->GetAxles()) {
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
