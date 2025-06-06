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
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
#endif

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
using namespace chrono::vehicle;

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChFsiVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].y < 0.616; }
};
#endif
// ===================================================================================================================

// CRM terrain patch type
enum class PatchType { RECTANGULAR, MARKER_DATA, HEIGHT_MAP };
PatchType patch_type = PatchType::RECTANGULAR;

// Terrain dimensions (for RECTANGULAR or HEIGHT_MAP patch type)
double terrain_length = 50;  // Changed from 20 to 50
double terrain_width = 3;

// Driver control types
enum class ControlType { SPEED, THROTTLE };

// Vehicle specification files
std::string vehicle_json = "Polaris/Polaris_simplest.json";
std::string engine_json = "Polaris/Polaris_EngineSimpleMap.json";
std::string transmission_json = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
std::string rigid_tire_json = "Polaris/Polaris_RigidMeshTire.json";
std::string deformable_tire_json = "Polaris/Polaris_ANCF4Tire_Lumped.json";
std::string tire_json;  // Will be set based on CLI input

// Soil types
struct SoilProperties {
    double density;
    double cohesion;
    double friction;
};

// Predefined soil types
SoilProperties soft_soil = {1600, 5e2, 0.7};
SoilProperties hard_soil = {1800, 1e3, 0.9};

// Target speed for speed-controlled simulation [m/s]
double target_speed = 5.0;  // Changed from 7.0 to 5.0

// Suspend vehicle
bool fix_chassis = false;

// Flag to indicate if rigid terrain is selected
bool use_rigid_terrain = false;

// ===================================================================================================================

// Custom throttle profile driver (used when ControlType::THROTTLE is selected)
// This driver implements a simple timeline-based throttle control:
// 1. Ramp up throttle during acceleration phase
// 2. Maintain throttle for cruise phase
// 3. Coast with no throttle for coast phase
// 4. Apply braking when near the edge of the terrain
class ChThrottleDriver : public ChDriver {
  public:
    ChThrottleDriver(ChVehicle& vehicle, double terrain_length);
    virtual ~ChThrottleDriver() {}

    virtual void Synchronize(double time) override;
    virtual void Advance(double step) override;

  private:
    const double m_terrain_length;    // Length of terrain (for braking decision)
    const double m_accel_time;        // Duration of acceleration phase [s]
    const double m_cruise_time;       // Duration of cruise phase [s]
    const double m_coast_time;        // Duration of coast phase [s]
    const double m_max_throttle;      // Maximum throttle value
    const double m_braking_distance;  // Distance from end to start braking [m]
};

// ===================================================================================================================

std::shared_ptr<WheeledVehicle> CreateVehicle(const ChCoordsys<>& init_pos,
                                              bool& fea_tires,
                                              double tire_pressure = 100e3);
void CreateFSIWheels(std::shared_ptr<WheeledVehicle> vehicle, std::shared_ptr<CRMTerrain> terrain);
std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);
bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& soil_type,
                     std::string& tire_type,
                     double& tire_pressure,
                     std::string& control_type);

// ===================================================================================================================

int main(int argc, char* argv[]) {
    // ----------------
    // Parse command line arguments
    // ----------------
    std::string soil_type = "hard";              // Default soil type
    std::string tire_type = "deformable";        // Default tire type
    std::string control_type = "speed_control";  // Default control type
    double tire_pressure = 100e3;                // Default tire pressure (100 kPa)

    if (!GetProblemSpecs(argc, argv, soil_type, tire_type, tire_pressure, control_type)) {
        return 1;
    }

    // Set tire JSON based on tire type
    tire_json = (tire_type == "rigid") ? rigid_tire_json : deformable_tire_json;

    // Set soil properties based on soil type or if using rigid terrain
    use_rigid_terrain = (soil_type == "rigid");
    SoilProperties soil_props = (soil_type == "soft") ? soft_soil : hard_soil;

    // Set control type
    ControlType driver_mode = (control_type == "speed_control") ? ControlType::SPEED : ControlType::THROTTLE;

    // Print selected options
    cout << "===== SIMULATION SETTINGS =====" << endl;
    if (use_rigid_terrain) {
        cout << "Soil type: rigid (using RigidTerrain)" << endl;
    } else {
        cout << "Soil type: " << soil_type << " (density=" << soil_props.density << ", cohesion=" << soil_props.cohesion
             << ", friction=" << soil_props.friction << ")" << endl;
    }
    cout << "Tire type: " << tire_type << endl;
    if (tire_type == "deformable") {
        cout << "Tire pressure: " << tire_pressure / 1000.0 << " kPa" << endl;
    }
    cout << "Control type: " << control_type << endl;
    cout << "Terrain size: " << terrain_length << "m x " << terrain_width << "m" << endl;
    if (driver_mode == ControlType::SPEED) {
        cout << "Target speed: " << target_speed << " m/s" << endl;
    }
    cout << "=============================" << endl;

    // ----------------
    // Problem settings
    // ----------------

    double tend = 5;  // Fixed simulation time of 10 seconds
    bool verbose = true;

    // Visualization settings
    bool render = false;                       // use run-time visualization
    double render_fps = 100;                  // rendering FPS
    bool visualization_sph = true;            // render SPH particles
    bool visualization_bndry_bce = false;     // render boundary BCE markers
    bool visualization_rigid_bce = false;     // render wheel BCE markers
    bool visualization_flexible_bce = false;  // render flexible body BCE markers
    bool snapshots = true;                    // enable snapshots at render frequency

#ifdef CHRONO_POSTPROCESS
    bool blender_output = false;  // enable Blender post-processing output
    double blender_fps = 20;     // frames per second for Blender output
#endif

    bool particle_output = false;  // enable SPH particle output
    double particle_fps = 20;     // frames per second for SPH particle output

    // CRM material properties
    double density = soil_props.density;
    double cohesion = soil_props.cohesion;
    double friction = soil_props.friction;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    // CRM (moving) active box dimension
    double active_box_hdim = 0.7;
    double settling_time = 0;

    // Set SPH spacing
    double spacing = (patch_type == PatchType::MARKER_DATA) ? 0.02 : 0.02;

    // --------------
    // Create vehicle
    // --------------

    cout << "Create vehicle..." << endl;
    double vehicle_init_height = 0.25;
    bool fea_tires = false;
    auto vehicle =
        CreateVehicle(ChCoordsys<>(ChVector3d(3.5, 0, vehicle_init_height), QUNIT), fea_tires, tire_pressure);
    vehicle->GetChassis()->SetFixed(fix_chassis);
    auto sysMBS = vehicle->GetSystem();

    // ---------------------------------
    // Set solver and integrator for MBD
    // ---------------------------------

    double step_size = 0;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;

    if (fea_tires) {
        step_size = 5e-4;
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

    // ----------------------
    // Create terrain (either CRM or Rigid)
    // ----------------------

    std::shared_ptr<ChBezierCurve> path;
    std::shared_ptr<RigidTerrain> rigid_terrain;
    std::shared_ptr<CRMTerrain> crm_terrain;
    ChFsiSystemSPH* sysFSI_ptr = nullptr;

    if (use_rigid_terrain) {
        // Create a rigid terrain
        cout << "Create rigid terrain..." << endl;
        rigid_terrain = chrono_types::make_shared<RigidTerrain>(sysMBS);

        // Create the terrain material (same friction as defined for soil)
        auto terrain_mat = chrono_types::make_shared<ChContactMaterialSMC>();
        terrain_mat->SetFriction(soil_props.friction);
        terrain_mat->SetRestitution(0.2f);
        terrain_mat->SetYoungModulus(2e7f);
        terrain_mat->SetPoissonRatio(0.3f);

        // Create a flat terrain patch
        auto patch = rigid_terrain->AddPatch(
            terrain_mat, ChCoordsys<>(ChVector3d(terrain_length / 2, 0, 0.25), QUNIT),  // Same center as CRM terrain
            terrain_length, terrain_width);  // Same dimensions as CRM terrain

        // Set terrain visualization properties
        patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
        patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), terrain_length / 4, terrain_width / 4);

        // Initialize the rigid terrain
        rigid_terrain->Initialize();

        // Create straight line path for speed controller
        path = StraightLinePath(ChVector3d(0, 0, vehicle_init_height),
                                ChVector3d(terrain_length, 0, vehicle_init_height), 1);
    } else {
        // Create the CRM terrain
        cout << "Create CRM terrain..." << endl;
        crm_terrain = chrono_types::make_shared<CRMTerrain>(*sysMBS, spacing);

        crm_terrain->SetVerbose(verbose);
        crm_terrain->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
        crm_terrain->SetStepSizeCFD(step_size);

        // Register the vehicle with the CRM terrain
        crm_terrain->RegisterVehicle(vehicle.get());

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
        crm_terrain->SetElasticSPH(mat_props);

        // Set SPH solver parameters
        ChFsiFluidSystemSPH::SPHParameters sph_params;
        sph_params.sph_method = SPHMethod::WCSPH;
        sph_params.initial_spacing = spacing;
        sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
        sph_params.shifting_xsph_eps = 0.5;
        sph_params.shifting_ppst_push = 3.0;
        sph_params.shifting_ppst_pull = 1.0;
        sph_params.d0_multiplier = 1.2;
        sph_params.kernel_threshold = 0.8;
        sph_params.artificial_viscosity = 0.5;
        sph_params.consistent_gradient_discretization = false;
        sph_params.consistent_laplacian_discretization = false;
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
        sph_params.boundary_type = BoundaryType::ADAMI;
        crm_terrain->SetSPHParameters(sph_params);

        // Set output level from SPH simulation
        crm_terrain->SetOutputLevel(OutputLevel::CRM_FULL);

        // Add vehicle wheels as FSI solids
        CreateFSIWheels(vehicle, crm_terrain);
        crm_terrain->SetActiveDomain(ChVector3d(active_box_hdim));
        crm_terrain->SetActiveDomainDelay(settling_time);

        // Construct the terrain and associated path
        cout << "Construct CRM terrain patch..." << endl;

        // Create a rectangular terrain patch
        crm_terrain->Construct({terrain_length, terrain_width, 0.25},  // length X width X height
                               ChVector3d(terrain_length / 2, 0, 0),   // patch center
                               BoxSide::ALL & ~BoxSide::Z_POS          // all boundaries, except top
        );

        // Create straight line path for speed controller
        path = StraightLinePath(ChVector3d(0, 0, vehicle_init_height),
                                ChVector3d(terrain_length, 0, vehicle_init_height), 1);

       

        // Initialize the terrain system
        crm_terrain->Initialize();

        auto aabb = crm_terrain->GetSPHBoundingBox();
        cout << "  SPH particles:     " << crm_terrain->GetNumSPHParticles() << endl;
        cout << "  Bndry BCE markers: " << crm_terrain->GetNumBoundaryBCEMarkers() << endl;
        cout << "  SPH AABB:          " << aabb.min << "   " << aabb.max << endl;

        // Set maximum vehicle X location (based on CRM patch size)
        double x_max = aabb.max.x() - 5.5;
        sysFSI_ptr = &crm_terrain->GetSystemFSI();
    }

    // --------------------------------
    // Create appropriate driver type
    // --------------------------------

    cout << "Create driver..." << endl;
    std::shared_ptr<ChDriver> driver;

    switch (driver_mode) {
        case ControlType::SPEED: {
            auto path_driver = std::make_shared<ChPathFollowerDriver>(*vehicle, path, "my_path", target_speed);
            path_driver->GetSteeringController().SetLookAheadDistance(2.0);
            path_driver->GetSteeringController().SetGains(1.0, 0, 0);
            path_driver->GetSpeedController().SetGains(0.6, 0.05, 0);
            driver = path_driver;
            break;
        }
        case ControlType::THROTTLE: {
            auto throttle_driver = std::make_shared<ChThrottleDriver>(*vehicle, terrain_length);
            driver = throttle_driver;
            break;
        }
    }

    driver->Initialize();

    // -----------------------------
    // Set up output
    // -----------------------------

    std::stringstream ss;
    ss << "CRM_Vehicle";
    // Add soil type
    ss << "_soil-" << soil_type;
    // Add tire type and pressure if applicable
    ss << "_tire-" << tire_type;
    if (tire_type == "deformable") {
        ss << "-" << std::fixed << std::setprecision(0) << tire_pressure / 1000.0 << "kPa";
    }
    // Add control type
    ss << "_ctrl-" << control_type;
    // Add target speed for speed control
    if (control_type == "speed_control") {
        ss << "-" << std::fixed << std::setprecision(1) << target_speed << "ms";
    }

    std::string out_dir = GetChronoOutputPath() + ss.str() + "/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // Create snapshots directory if needed
    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
            std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
            return 1;
        }
    }

    std::string out_file = out_dir + "/results.txt";
    utils::ChWriterCSV csv(",");  // Use comma as separator

    // Setup CSV header for vehicle state data
    csv << "Time";
    csv << "PosX"
        << "PosY"
        << "PosZ";  // Position components
    csv << "VelX"
        << "VelY"
        << "VelZ";  // Velocity components
    csv << endl;

    // Create the Blender exporter
#ifdef CHRONO_POSTPROCESS
    postprocess::ChBlender blender_exporter(sysMBS);
    if (blender_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/blender"))) {
            std::cerr << "Error creating directory " << out_dir + "/blender" << std::endl;
            return 1;
        }
        blender_exporter.SetBasePath(out_dir + "/blender");
        blender_exporter.SetBlenderUp_is_ChronoZ();
        blender_exporter.SetCamera(ChVector3d(1.5, 8, 1.5), ChVector3d(0, 0, 0), 45);
        blender_exporter.AddAll();
        blender_exporter.ExportScript();
    }
    int blender_frame = 0;
    int particle_frame = 0;
#endif

    if (particle_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
            std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
            std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
            return 1;
        }
    }
    // -----------------------------
    // Create run-time visualization
    // -----------------------------

    std::shared_ptr<ChVehicleVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        if (use_rigid_terrain) {
            // VSG visual system for rigid terrain
            auto visVSG = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            visVSG->AttachVehicle(vehicle.get());
            visVSG->AttachTerrain(rigid_terrain.get());
            visVSG->SetWindowTitle("Wheeled vehicle on Rigid terrain");
            visVSG->SetWindowSize(1920, 1080);
            visVSG->SetWindowPosition(100, 100);
            visVSG->EnableSkyBox();
            visVSG->SetLightIntensity(1.0f);
            visVSG->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            visVSG->SetCameraAngleDeg(40);
            visVSG->SetChaseCamera(VNULL, 6.0, 2.0);
            visVSG->SetChaseCameraPosition(ChVector3d(2, 8, 1.5));

            visVSG->Initialize();
            vis = visVSG;
        } else if (sysFSI_ptr) {
            // Existing VSG visual system with FSI for CRM terrain
            auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(sysFSI_ptr);
            visFSI->EnableFluidMarkers(visualization_sph);
            visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
            visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
            visFSI->EnableFlexBodyMarkers(visualization_flexible_bce);
            visFSI->SetSPHColorCallback(chrono_types::make_shared<ParticlePressureColorCallback>(0, 30000));

            visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

            // Wheeled vehicle VSG visual system (attach visFSI as plugin)
            auto visVSG = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            visVSG->AttachVehicle(vehicle.get());
            visVSG->AttachPlugin(visFSI);
            visVSG->SetWindowTitle("Wheeled vehicle on CRM deformable terrain");
            visVSG->SetWindowSize(1920, 1080);
            visVSG->SetWindowPosition(100, 100);
            visVSG->EnableSkyBox();
            visVSG->SetLightIntensity(1.0f);
            visVSG->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            visVSG->SetCameraAngleDeg(40);
            visVSG->SetChaseCamera(VNULL, 6.0, 2.0);
            visVSG->SetChaseCameraPosition(ChVector3d(2, 8, 1.5));
            visVSG->AddGuiColorbar("Pressure", 0, 30000);

            visVSG->Initialize();
            vis = visVSG;
        }
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
    int data_output_frame = 0;       // Counter for data output
    double data_output_fps = 100.0;  // 100 Hz output rate for vehicle state
    bool braking = false;

    double rigid_terrain_rtf = 0;
    double crm_terrain_cfd_rtf = 0;
    double crm_terrain_mbs_rtf = 0;
    size_t step_counter = 0;

    cout << "Start simulation..." << endl;

    while (time < tend) {
        const auto& veh_loc = vehicle->GetPos();

        // Get the driver inputs based on current time and position
        driver->Synchronize(time);
        auto driver_inputs = driver->GetInputs();

#ifdef CHRONO_VSG
        // Run-time visualization
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            // Save snapshots at render frequency if enabled
            if (snapshots) {
                if (verbose)
                    cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }
        if (!render) {
            if (use_rigid_terrain) {
                // std::cout << time << " " << vehicle->GetSystem()->GetRTF() << std::endl;
                rigid_terrain_rtf += vehicle->GetSystem()->GetRTF();
                step_counter++;
            } else if (crm_terrain) {
                // std::cout << time << "  " << crm_terrain->GetRtfCFD() << "  " << crm_terrain->GetRtfMBD() << std::endl;
                crm_terrain_cfd_rtf += crm_terrain->GetRtfCFD();
                crm_terrain_mbs_rtf += crm_terrain->GetRtfMBD();
                step_counter++;
            }
        }
#endif
        // Output Blender frames
#ifdef CHRONO_POSTPROCESS
        if (blender_output && time >= blender_frame / blender_fps) {
            blender_exporter.ExportData();
            blender_frame++;
        }
#endif

        // Save particle and solid data at specified FPS rate
        if (particle_output && time >= particle_frame / particle_fps) {
            if (verbose)
                cout << " -- Output SPH data at t = " << time << endl;

            if (!use_rigid_terrain) {
                crm_terrain->SaveOutputData(time, out_dir + "/particles", out_dir + "/fsi");
            }

            particle_frame++;
        }

        // Output vehicle state data at 100 FPS
        if (time >= data_output_frame / data_output_fps) {
            // Get vehicle position
            const ChVector3d& pos = vehicle->GetPos();

            // Get vehicle velocity
            const ChVector3d& vel = vehicle->GetSpeed();

            // Write data to CSV file
            csv << time;
            csv << pos.x() << pos.y() << pos.z();
            csv << vel.x() << vel.y() << vel.z();
            csv << endl;

            data_output_frame++;
        }

        // Synchronize systems
        if (render) {
            vis->Synchronize(time, driver_inputs);
        }

        if (use_rigid_terrain && rigid_terrain) {
            // Synchronize rigid terrain
            rigid_terrain->Synchronize(time);
            vehicle->Synchronize(time, driver_inputs, *rigid_terrain);
        } else if (crm_terrain) {
            // Synchronize CRM terrain
            crm_terrain->Synchronize(time);
            vehicle->Synchronize(time, driver_inputs, *crm_terrain);
        }

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        if (render) {
            vis->Advance(step_size);
        }

        if (use_rigid_terrain && rigid_terrain) {
            // Advance rigid terrain
            rigid_terrain->Advance(step_size);
            vehicle->Advance(step_size);
        } else if (crm_terrain) {
            // Advance coupled FSI problem (CRM terrain + vehicle)
            crm_terrain->Advance(step_size);
        }

        time += step_size;
        sim_frame++;
    }

    if (use_rigid_terrain) {
        rigid_terrain_rtf /= step_counter;
    } else if (crm_terrain) {
        crm_terrain_cfd_rtf /= step_counter;
        crm_terrain_mbs_rtf /= step_counter;
    }
    if (use_rigid_terrain) {
        std::cout << "Rigid terrain RTF: " << rigid_terrain_rtf << std::endl;
    } else if (crm_terrain) {
        std::cout << "CRM terrain CFD RTF: " << crm_terrain_cfd_rtf << std::endl;
        std::cout << "CRM terrain MBS RTF: " << crm_terrain_mbs_rtf << std::endl;
    }

    csv.WriteToFile(out_file);

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

std::shared_ptr<WheeledVehicle> CreateVehicle(const ChCoordsys<>& init_pos, bool& fea_tires, double tire_pressure) {
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

            // Set tire pressure if this is a deformable tire
            auto def_tire = std::dynamic_pointer_cast<ChDeformableTire>(tire);
            if (def_tire) {
                def_tire->SetPressure(tire_pressure);
                fea_tires = true;
            }

            vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    return vehicle;
}

void CreateFSIWheels(std::shared_ptr<WheeledVehicle> vehicle, std::shared_ptr<CRMTerrain> terrain) {
    terrain->SetBcePattern2D(BcePatternMesh2D::INWARD);
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
                terrain->AddFeaMesh(mesh, false);
            } else {
                terrain->AddRigidBody(wheel->GetSpindle(), geometry, false);
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

// ===================================================================================================================

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& soil_type,
                     std::string& tire_type,
                     double& tire_pressure,
                     std::string& control_type) {
    ChCLI cli(argv[0], "Wheeled vehicle on terrain demo");

    cli.AddOption<std::string>("Soil", "soil_type", "Soil type (soft/hard/rigid)", "hard");
    cli.AddOption<std::string>("Tire", "tire_type", "Tire type (rigid/deformable)", "deformable");
    cli.AddOption<double>("Tire", "tire_pressure", "Tire pressure in kPa (for deformable tires)", "100");
    cli.AddOption<std::string>("Control", "control_type", "Control type (speed_control/throttle)", "speed_control");

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    // Get parameters
    soil_type = cli.GetAsType<std::string>("soil_type");
    tire_type = cli.GetAsType<std::string>("tire_type");
    control_type = cli.GetAsType<std::string>("control_type");

    // Check for valid soil type
    if (soil_type != "soft" && soil_type != "hard" && soil_type != "rigid") {
        cerr << "Error: soil_type must be either 'soft', 'hard', or 'rigid'" << endl;
        return false;
    }

    // Check for valid tire type
    if (tire_type != "rigid" && tire_type != "deformable") {
        cerr << "Error: tire_type must be either 'rigid' or 'deformable'" << endl;
        return false;
    }

    // Check for valid control type
    if (control_type != "speed_control" && control_type != "throttle") {
        cerr << "Error: control_type must be either 'speed_control' or 'throttle'" << endl;
        return false;
    }

    // Convert tire pressure from kPa to Pa
    tire_pressure = cli.GetAsType<double>("tire_pressure") * 1000.0;

    return true;
}

// Implementation of ChThrottleDriver methods
ChThrottleDriver::ChThrottleDriver(ChVehicle& vehicle, double terrain_length)
    : ChDriver(vehicle),
      m_terrain_length(terrain_length),
      m_accel_time(1.0),       // 2 seconds to ramp up throttle
      m_cruise_time(1.5),      // 3 seconds to maintain throttle
      m_coast_time(1.5),       // 3 seconds of coasting
      m_max_throttle(0.8),     // 70% maximum throttle
      m_braking_distance(6.0)  // Start braking 5m from the edge
{
    // Initialize with zero inputs
    m_throttle = 0;
    m_steering = 0;
    m_braking = 0;
}

void ChThrottleDriver::Synchronize(double time) {
    ChDriver::Synchronize(time);

    // Get current vehicle position
    const ChVector3d& pos = m_vehicle.GetPos();

    // Calculate distance from end of terrain
    double distance_from_end = m_terrain_length - pos.x();

    // Check if we need to brake (near the end of terrain)
    if (distance_from_end < m_braking_distance) {
        m_throttle = 0;
        m_braking = 1.0;
        return;
    }

    // Otherwise, follow the phase-based throttle profile
    if (time < m_accel_time) {
        // Acceleration phase - ramp up throttle
        m_throttle = (time / m_accel_time) * m_max_throttle;
        m_braking = 0;
    } else if (time < m_accel_time + m_cruise_time) {
        // Cruise phase - maintain throttle
        m_throttle = m_max_throttle;
        m_braking = 0;
    } else if (time < m_accel_time + m_cruise_time + m_coast_time) {
        // Coast phase - no throttle, no braking
        m_throttle = 0;
        m_braking = 0;
    } else {
        // Final phase - start braking if we haven't already
        m_throttle = 0;
        m_braking = 0.7;
    }

    // Keep steering at zero (straight line)
    m_steering = 0;
}

void ChThrottleDriver::Advance(double step) {
    // Nothing to do here as we update everything in Synchronize
}
