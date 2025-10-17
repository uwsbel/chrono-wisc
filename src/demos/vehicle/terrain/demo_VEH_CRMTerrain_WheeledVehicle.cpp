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
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "demos/vehicle/WheeledVehicleJSON.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"

#endif

#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire3443B.h"

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
using namespace chrono::vehicle;

using std::cout;
using std::cerr;
using std::cin;
using std::endl;
bool set_str_spk = true;
// ===================================================================================================================

// CRM terrain patch type
enum class PatchType { RECTANGULAR, MARKER_DATA, HEIGHT_MAP };
PatchType patch_type = PatchType::HEIGHT_MAP;

enum class TireType { ANCF_TOROIDAL, ANCF_AIRLESS };
TireType tire_type = TireType::ANCF_AIRLESS;
 //TireType tire_type = TireType::ANCF_TOROIDAL;

// Terrain dimensions (for RECTANGULAR or HEIGHT_MAP patch type)
double terrain_length = 20;
double terrain_width = 3;

// Vehicle specification files
std::string vehicle_json = "Polaris/Polaris_LTV.json";
std::string engine_json = "Polaris/Polaris_EngineSimpleMap.json";
std::string transmission_json = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
std::string tire_json = "Polaris/Polaris_RigidTire.json";
////std::string tire_json = "Polaris/Polaris_ANCF4Tire_Lumped.json";

// Suspend vehicle
bool fix_chassis = false;

// ===================================================================================================================

std::shared_ptr<WheeledVehicle> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires);
void CreateFSIWheels(std::shared_ptr<WheeledVehicle> vehicle,
                     std::shared_ptr<WheeledTrailer> trailer,
                     CRMTerrain& terrain);
std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);

// ===================================================================================================================

int main(int argc, char* argv[]) {

    if (argc != 4) {
        std::cout << "need to define slope angle, gravity (moon - 0, mars - 1) and friction ceofficient" << std::endl;
        return 0;
    }

    double slope_angle = atof(argv[1]);
    bool use_mars_grav = atoi(argv[2]);
    double friction = atof(argv[3]);


    // ----------------
    // Problem settings
    // ----------------
    double target_speed = 0.5;
    double tend = 30;
    bool verbose = true;

    // Visualization settings
    bool render = true;                    // use run-time visualization
    double render_fps = 10;                // rendering FPS
    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = true;  // render boundary BCE markers
    bool visualization_rigid_bce = true;   // render wheel BCE markers

    double density, cohesion, youngs_modulus, poisson_ratio;

    // CRM material properties
    if (use_mars_grav) {
        density = 1700;
        cohesion = 500;
        youngs_modulus = 1e6;
        poisson_ratio = 0.3;    
    
    } else {
        density = 1700;
        cohesion = 500;
        youngs_modulus = 1e6;
        poisson_ratio = 0.3;    
    }

    // CRM (moving) active box dimension
    double active_box_hdim = 0.4;
    double settling_time = 0;

    // Set SPH spacing
    double spacing = (patch_type == PatchType::MARKER_DATA) ? 0.02 : 0.04;
    bool add_trailer = false;

    auto trailer_model = UT_Model();

    // --------------
    // Create vehicle
    // --------------

    cout << "Create vehicle..." << endl;
    double vehicle_init_height = 0.5;
    bool fea_tires;
    auto vehicle = CreateVehicle(ChCoordsys<>(ChVector3d(3.5, 0, vehicle_init_height), QUNIT), fea_tires);
    std::shared_ptr<WheeledTrailer> trailer;

    if (add_trailer) {
        trailer = chrono_types::make_shared<WheeledTrailer>(vehicle->GetSystem(),
                                                            vehicle::GetDataFile(trailer_model.TrailerJSON()));

        trailer->Initialize(vehicle->GetChassis());

        trailer->SetChassisVisualizationType(VisualizationType::MESH);

        trailer->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);

        trailer->SetWheelVisualizationType(VisualizationType::MESH);

        for (auto& axle : trailer->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                // if (tire_type == TireType::ANCF_AIRLESS) {
                auto tire = chrono_types::make_shared<ANCFAirlessTire3443B>("Airless3443B");
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetRimRadius(0.225);  // Default is 0.225
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHeight(0.225);     // Default is 0.225
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetWidth(0.4);        // Default is 0.4
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetAlpha(0.05);       // Default is 0.05
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetOuterRingThickness(
                    0.015);  // Default is 0.015
                // tire->SetYoungsModulus(y_mod);  // Default is 76e9
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusSpokes(1e6);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusOuterRing(5e9);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetPoissonsRatio(0.3);       // Default is 0.2
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivWidth(3);              // Default is 3
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivSpokeLength(3);        // Default is 3
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivOuterRingPerSpoke(3);  // Default is 3
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetNumberSpokes(16);
                // Options to set for straight spokes
                if (set_str_spk) {
                    std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHubRelativeRotation(0);
                    std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetSpokeCurvatureXPoint(0);
                    std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetSpokeCurvatureZPoint(0);
                }

                trailer->InitializeTire(tire, wheel, VisualizationType::MESH);
                // } else if (tire_type == TireType::ANCF_TOROIDAL) {
                //     auto ancf_tire = chrono_types::make_shared<ANCFToroidalTire>("ANCFtoroidal tire");
                //     ancf_tire->SetRimRadius(0.30375);
                //     ancf_tire->SetHeight(0.15);
                //     ancf_tire->SetThickness(0.015);
                //     ancf_tire->SetDivCircumference(40);
                //     ancf_tire->SetDivWidth(8);
                //     ancf_tire->SetPressure(320e3);
                //     ancf_tire->SetAlpha(0.05);

                //     trailer->InitializeTire(ancf_tire, wheel, VisualizationType::MESH);
                // }
            }
        }
    }

    vehicle->GetChassis()->SetFixed(fix_chassis);
    auto sysMBS = vehicle->GetSystem();

    // ---------------------------------
    // Set solver and integrator for MBD
    // ---------------------------------

    double step_size = 0;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;

    if (fea_tires) {
        step_size = 1e-3;
        solver_type = ChSolver::Type::PARDISO_MKL;
        integrator_type = ChTimestepper::Type::HHT;
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

    if (use_mars_grav) {
        sysMBS->SetGravitationalAcceleration(ChVector3d(0, 0, -3.73));
        std::cout << "USE mars gravity!! " << std::endl;
    } else {
        sysMBS->SetGravitationalAcceleration(ChVector3d(0, 0, -1.62));
        std::cout << "USE moon gravity!! " << std::endl;
    
    }
    // ----------------------
    // Create the CRM terrain
    // ----------------------
    CRMTerrain terrain(*sysMBS, spacing);
    ChFsiSystemSPH& sysFSI = terrain.GetSystemFSI();
    terrain.SetVerbose(verbose);

    if (use_mars_grav) {
        terrain.SetGravitationalAcceleration(ChVector3d(0, 0, -3.73));    
    } else {
        terrain.SetGravitationalAcceleration(ChVector3d(0, 0, -1.62));
    }

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
    mat_props.average_diam = 0.005;
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
    CreateFSIWheels(vehicle, trailer, terrain);
    //terrain.SetActiveDomain(ChVector3d(active_box_hdim));
    //terrain.SetActiveDomainDelay(settling_time);

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

        }

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

    // Initialize the terrain system
    terrain.Initialize();

    std::string out_dir;
    if (use_mars_grav)
        out_dir = GetChronoOutputPath() + "mars_grav_slope_" + argv[1] + "_mu_" + argv[3];
    else 
        out_dir = GetChronoOutputPath() + "moon_grav_slope_" + argv[1] + "_mu_" + argv[3];

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    bool snapshots = true;
    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
            std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
            return 1;
        }
    }
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

    std::string out_file = out_dir + "/results.txt";
    utils::ChWriterCSV csv(" ");

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
        visFSI->SetColorFluidMarkers(ChColor(1, 0.7, 0.7));
        //visFSI->SetSPHColorCallback(chrono_types::make_shared<ParticleHeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f), aabb.min.z(), aabb.max.z()));

        // Wheeled vehicle VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
        visVSG->AttachVehicle(vehicle.get());
        visVSG->AttachPlugin(visFSI);
        visVSG->SetWindowTitle("Wheeled vehicle on CRM deformable terrain");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
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
    int output_frame = 50; // write output every 100 time steps
    bool braking = false;

    cout << "Start simulation..." << endl;

    std::ofstream csv_file(out_dir + "/simulation_data.csv");
    csv_file << "time,x,y,z,roll,pitch,yaw,vx,vy,vz\n";


    ChVector3d vehicle_pos;
    ChQuaternion<> vehicle_rot;
    ChVector3d vehicle_vel;

    while (time < tend) {
        // write outputs every 10 frames 
        if (sim_frame % 10 == 0) {
        
            vehicle_pos = vehicle->GetPos();
            vehicle_rot = vehicle->GetRot();
            vehicle_vel = vehicle->GetChassisBody()->GetPosDt();

            // Vehicle orientation in Euler angles (roll, pitch, yaw)
            ChVector3d euler_angles = vehicle_rot.GetRotVec();
            double roll = euler_angles.x();
            double pitch = euler_angles.y();
            double yaw = euler_angles.z();

            csv_file << std::fixed << std::setprecision(6) << time << "," 
                     << vehicle_pos.x() << "," << vehicle_pos.y() << "," << vehicle_pos.z() << "," << roll << "," << pitch << "," << yaw << ","
                     << vehicle_vel.x() << "," << vehicle_vel.y() << "," << vehicle_vel.z() << "\n";        

            std::cout << time << ", " << vehicle_pos.z() << std::endl;


        }


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
            if (snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                vis->WriteImageToFile(filename.str());
            }
            render_frame++;
        }

        // Synchronize systems
        driver.Synchronize(time);

        terrain.Synchronize(time);        
        if (render) {
            vis->Synchronize(time, driver_inputs);
        }

        vehicle->Synchronize(time, driver_inputs, terrain);
        if (add_trailer)
            trailer->Synchronize(time, driver_inputs, terrain);

        // Advance system state
        driver.Advance(step_size);
        if (render)
            vis->Advance(step_size);
        // Coupled FSI problem (CRM terrain + vehicle)
        terrain.Advance(step_size);
        // th.join();
        // if (add_trailer)
        //     trailer->Advance(step_size);

        csv << time << vehicle->GetPos() << vehicle->GetSpeed() << endl;

        time += step_size;
        sim_frame++;
    }
    csv_file.close();
    //csv.WriteToFile(out_file);

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
            if (tire_type == TireType::ANCF_AIRLESS) {
                auto tire = chrono_types::make_shared<ANCFAirlessTire3443B>("Airless3443B");
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetRimRadius(0.225);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHeight(0.225);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetWidth(0.4);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetAlpha(0.05);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetOuterRingThickness(0.015);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusSpokes(1e6);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusOuterRing(5e9);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetPoissonsRatio(0.3);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivWidth(3);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivSpokeLength(3);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivOuterRingPerSpoke(3);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetNumberSpokes(16);

                if (set_str_spk) {
                    std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHubRelativeRotation(0);
                    std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetSpokeCurvatureXPoint(0);
                    std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetSpokeCurvatureZPoint(0);
                }

                vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
                if (std::dynamic_pointer_cast<ChDeformableTire>(tire))
                    fea_tires = true;
            } else if (tire_type == TireType::ANCF_TOROIDAL) {
                auto ancf_tire = chrono_types::make_shared<ANCFToroidalTire>("ANCFtoroidal tire");
                ancf_tire->SetRimRadius(0.30375);
                ancf_tire->SetHeight(0.15);
                ancf_tire->SetThickness(0.015);
                ancf_tire->SetDivCircumference(40);
                ancf_tire->SetDivWidth(8);
                ancf_tire->SetPressure(0);
                ancf_tire->SetAlpha(0.15);

                vehicle->InitializeTire(ancf_tire, wheel, VisualizationType::MESH);
                if (std::dynamic_pointer_cast<ChDeformableTire>(ancf_tire))
                    fea_tires = true;
            }

            
        }
    }

    return vehicle;
}

void CreateFSIWheels(std::shared_ptr<WheeledVehicle> vehicle,
                     std::shared_ptr<WheeledTrailer> trailer,
                     CRMTerrain& terrain) {
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

    if (trailer) {
        for (auto& axle : trailer->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto mesh = std::dynamic_pointer_cast<ChDeformableTire>(wheel->GetTire())->GetMesh();
                terrain.AddFeaMesh(mesh, false);
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
