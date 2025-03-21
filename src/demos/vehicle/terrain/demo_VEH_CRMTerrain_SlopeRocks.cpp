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

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/optix/ChNVDBVolume.h"
#include <execution>

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;
using namespace chrono::sensor;

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
double terrain_length = 20;
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
bool add_rocks = true;

// Save snapshots of the simulation
bool save_snapshots = false;

// Sensor params
// Noise model attached to the sensor
enum NoiseModel {
    CONST_NORMAL,     // Gaussian noise with constant mean and standard deviation
    PIXEL_DEPENDENT,  // Pixel dependent gaussian noise
    NONE              // No noise model
};
NoiseModel noise_model = NONE;

// Camera lens model
// Either PINHOLE or SPHERICAL
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
// Update rate in Hz
float update_rate = 30;
// Image width and height
unsigned int image_width = 1280;
unsigned int image_height = 720;
// 720;
// Camera's horizontal field of view
float fov = (float)CH_PI / 2.;
// Lag (in seconds) between sensing and when data becomes accessible
float lag = 0.0f;
// Exposure (in seconds) of each image
float exposure_time = 0.00f;
int alias_factor = 1;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------
// Save camera images
bool save = true;
// Render camera images
bool vis = true;
// Output directory
const std::string sensor_out_dir = "SENSOR_OUTPUT/SlopeRocks/";

bool use_gi = false;

// VDB info
bool firstInst = true;
std::vector<int> idList;
int prevActiveVoxels = 0;
std::vector<std::shared_ptr<ChBody>> voxelBodyList = {};
std::vector<float> offsetXList = {};
std::vector<float> offsetYList = {};
int activeVoxels = 0;
void createVoxelGrid(std::vector<ChVector3d> points,
                     std::shared_ptr<ChScene> scene,
                     std::shared_ptr<ChVisualMaterial> vis_mat);
int num_meshes = 100;
std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>> regolith_meshes;  // ChVisualShapeTriangleMesh
float slope_angle = 0.f;
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
    double cohesion = 1e3;
    double friction = 0.7;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    // CRM (moving) active box dimension
    double active_box_hdim = 0.4;

    // Set SPH spacing
    double spacing = (patch_type == PatchType::MARKER_DATA) ? 0.02 : 0.02;

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
    sph_params.d0_multiplier = 1.2;
    sph_params.kernel_threshold = 0.8;
    sph_params.artificial_viscosity = 0.5;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    sph_params.boundary_type = BoundaryType::ADAMI;
    sph_params.num_proximity_search_steps = 10;
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

    //
    // SENSOR SIMULATION BEGIN
    //

    // Load regolith meshes
    std::string mesh_name_prefix = "sensor/geometries/regolith/particle_";
    for (int i = 1; i <= num_meshes; i++) {
        auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
            GetChronoDataFile(mesh_name_prefix + std::to_string(i) + ".obj"), false, true);
        mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
        auto trimesh_shape = std::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(mmesh);
        // std::cout << "OK1" << std::endl;
        trimesh_shape->SetName("RegolithMesh" + std::to_string(i));
        trimesh_shape->SetMutable(false);
        regolith_meshes.push_back(trimesh_shape);
    }

    auto regolith_material = chrono_types::make_shared<ChVisualMaterial>();
    regolith_material->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
    regolith_material->SetDiffuseColor({1, 1, 1});  // 0.29f, 0.29f, 0.235f
    regolith_material->SetSpecularColor({1, 1, 1});
    regolith_material->SetUseSpecularWorkflow(true);
    regolith_material->SetRoughness(1.0f);
    regolith_material->SetBSDF((unsigned int)BSDFType::HAPKE);
    regolith_material->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f,
                                          23.4f * (CH_PI / 180));
    regolith_material->SetClassID(30000);
    regolith_material->SetInstanceID(20000);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sysMBS->Add(floor);

    // Create a Sensor manager
    float intensity = 1.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(sysMBS);
    manager->scene->AddPointLight({0, -5, 5}, {intensity, intensity, intensity}, 500);
    manager->scene->SetAmbientLight({.1, .1, .1});
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/starmap_2020_4k.hdr");
    manager->scene->SetBackground(b);
    manager->SetVerbose(false);
    manager->SetRayRecursions(4);
    Integrator integrator = Integrator::LEGACY;
    bool use_denoiser = false;

    // chrono::ChFrame<double> offset_pose1({0, 5, 0}, Q_from_AngAxis(0.2, {0, 0, 1}));  //-1200, -252, 100
    chrono::ChFrame<double> offset_pose1(
        {-1, -3, 1}, QuatFromAngleAxis(CH_PI_2, {0, 0, 1}));  // Q_from_AngAxis(CH_PI_4, {0, 1, 0})  //-1200, -252, 100
    auto cam =
        chrono_types::make_shared<ChCameraSensor>(vehicle->GetChassis()->GetBody(),  // body camera is attached to
                                                  update_rate,                       // update rate in Hz
                                                  offset_pose1,                      // offset pose
                                                  image_width,                       // image width
                                                  image_height,                      // image height
                                                  fov,           // camera's horizontal field of view
                                                  alias_factor,  // super sampling factor
                                                  lens_model,    // lens model type
                                                  use_gi);
    cam->SetIntegrator(integrator);
    cam->SetName("Third Person Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Third Person Camera"));

    if (save)
        cam->PushFilter(
            chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "CRM_DEMO_THIRD_PERSON_VIEW_RealSlope/"));
    manager->AddSensor(cam);

    // chrono::ChFrame<double> offset_pose2({-0.f, -1.7, 0.5}, QuatFromAngleAxis(.2, {-2, 3, 9.75}));
    chrono::ChFrame<double> offset_pose2({-0.3f, -0.6, 0.8f},
                                         QuatFromAngleAxis(0, {0, 0, 1}) * QuatFromAngleAxis(CH_PI_2, {0, 1, 0}));
    //-0.2f, -0.5, 1.f
    auto cam2 =
        chrono_types::make_shared<ChCameraSensor>(vehicle->GetChassis()->GetBody(),  // body camera is attached to
                                                  update_rate,                       // update rate in Hz
                                                  offset_pose2,                      // offset pose
                                                  image_width,                       // image width
                                                  image_height,                      // image height
                                                  fov,           // camera's horizontal field of view
                                                  alias_factor,  // super sampling factor
                                                  lens_model,    // lens model type
                                                  use_denoiser);
    cam2->SetIntegrator(integrator);
    cam2->SetName("FRWheelCam");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);
    if (vis)
        cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "FRWheelCam"));

    if (save)
        cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "FRWheelCam/"));
    manager->AddSensor(cam2);

    chrono::ChFrame<double> rrpose({-2.37f, -0.6, 0.8f},
                                   QuatFromAngleAxis(0, {0, 0, 1}) * QuatFromAngleAxis(CH_PI_2, {0, 1, 0}));
    auto rrcam =
        chrono_types::make_shared<ChCameraSensor>(vehicle->GetChassis()->GetBody(),  // body camera is attached to
                                                  update_rate,                       // update rate in Hz
                                                  rrpose,                            // offset pose
                                                  image_width,                       // image width
                                                  image_height,                      // image height
                                                  fov,           // camera's horizontal field of view
                                                  alias_factor,  // super sampling factor
                                                  lens_model,    // lens model type
                                                  use_denoiser);
    rrcam->SetIntegrator(integrator);
    rrcam->SetName("RRWheelCam");
    rrcam->SetLag(lag);
    rrcam->SetCollectionWindow(exposure_time);
    if (vis)
        rrcam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "RRWheelCam"));

    if (save)
        rrcam->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "RRWheelCam/"));
    manager->AddSensor(rrcam);

    auto seg =
        chrono_types::make_shared<ChSegmentationCamera>(vehicle->GetChassis()->GetBody(),  // body camera is attached to
                                                        update_rate,                       // update rate in Hz
                                                        offset_pose2,                      // offset pose
                                                        image_width,                       // image width
                                                        image_height,                      // image height
                                                        fov,          // camera's horizontal field of view
                                                        lens_model);  // FOV
    seg->SetName("Semantic Segmentation Camera");
    seg->SetLag(lag);
    seg->SetCollectionWindow(exposure_time);

    // Render the semantic mask
    if (vis)
        seg->PushFilter(
            chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Semantic Segmentation"));

    // Save the semantic mask
    if (save)
        seg->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "segmentation/"));

    // manager->AddSensor(seg);

    chrono::ChFrame<double> offset_pose3({-4.f, 0, .6f}, QuatFromAngleAxis(.2, {0, 1, 0}));
    // chrono::ChFrame<double> offset_pose3({0, 0, 5.f}, Q_from_AngAxis(CH_PI_2, {0, 1, 0}));
    auto cam3 =
        chrono_types::make_shared<ChCameraSensor>(vehicle->GetChassis()->GetBody(),  // body camera is attached to
                                                  update_rate,                       // update rate in Hz
                                                  offset_pose3,                      // offset pose
                                                  image_width,                       // image width
                                                  image_height,                      // image height
                                                  fov,           // camera's horizontal field of view
                                                  alias_factor,  // super sampling factor
                                                  lens_model,    // lens model type
                                                  use_denoiser);
    cam3->SetIntegrator(integrator);
    cam3->SetName("Rear Camera");
    cam3->SetLag(lag);
    cam3->SetCollectionWindow(exposure_time);
    if (vis)
        cam3->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Rear Camera"));

    if (save)
        cam3->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Rear_Cam_RealSlope/"));
    manager->AddSensor(cam3);

    chrono::ChFrame<double> offset_pose4({-7.f, 0, 3.f}, QuatFromAngleAxis(.2, {0, 1, 0}));
    // chrono::ChFrame<double> offset_pose4({0, 0, 50.f}, QuatFromAngleAxis(CH_PI_2, {0, 1, 0}));
    auto cam4 =
        chrono_types::make_shared<ChCameraSensor>(vehicle->GetChassis()->GetBody(),  // body camera is attached to
                                                  update_rate,                       // update rate in Hz
                                                  offset_pose4,                      // offset pose
                                                  image_width,                       // image width
                                                  image_height,                      // image height
                                                  fov,           // camera's horizontal field of view
                                                  alias_factor,  // super sampling factor
                                                  lens_model,    // lens model type
                                                  use_denoiser);
    cam4->SetIntegrator(integrator);
    cam4->SetName("Top Camera");
    cam4->SetLag(lag);
    cam4->SetCollectionWindow(exposure_time);
    if (vis)
        cam4->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Top Camera"));

    if (save)
        cam4->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Top_Cam_RealSensor/"));
    manager->AddSensor(cam4);

    // Add NanoVDB particles

    //
    //  SENSOR SIMULATION END
    //

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
    int sensor_render_steps = (unsigned int)round(1 / (update_rate * step_size));
    cout << "Start simulation..." << endl;

    ChTimer timer;
    std::vector<ChVector3d> h_points;
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
                // visFSI->UpdateCamera(cam_loc, cam_point);
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

        // if (sim_frame % sensor_render_steps == 0 && sim_frame > 0) {
        //     // timerNVDB.start();;
        //     h_points = sysFSI.GetFluidSystemSPH().GetParticlePositions();
        //     createVoxelGrid(h_points, manager->scene, regolith_material);
        //     manager->Update();
        // }

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
    double x1 = 15.0;  // Position on the slope
    double x2 = 16.0;  // Position on the slope

    // If x >= flat_portion_length, height is (x - flat_portion_length) * tan(angle)
    double z1 = (x1 > flat_portion_length) ? (x1 - flat_portion_length) * tan(slope_angle_rad) : 0.0;
    double z2 = (x2 > flat_portion_length) ? (x2 - flat_portion_length) * tan(slope_angle_rad) : 0.0;

    // Offset to adjust
    double height_offset_1 = 0.40;
    double height_offset_2 = 0.45;
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

void createVoxelGrid(std::vector<ChVector3d> points,
                     std::shared_ptr<ChScene> scene,
                     std::shared_ptr<ChVisualMaterial> vis_mat) {
    std::cout << "Creating CPU Voxel Grid for " << points.size() << "particles " << std::endl;
    float spacing = 0.02 / 2.f;
    float r = spacing;
    int pointsPerVoxel = 1;
    float voxelSize = spacing;
    std::cout << "VoxelSize=" << voxelSize << std::endl;

    activeVoxels = points.size();

    int numVoxelsToAdd = activeVoxels - prevActiveVoxels;
    int numAdds = 0;
    int numUpdates = 0;

    if (!firstInst) {
        thread_local std::mt19937 generator(std::random_device{}());
        std::uniform_int_distribution<int> distribution(0, num_meshes - 1);
        std::uniform_real_distribution<float> randpos(-0.02f, 0.02f);
        std::uniform_real_distribution<float> randscale(1.f, 2.f);
        int voxelCount = 0;
        for (int i = 0; i < activeVoxels; i++) {
            ChVector3d pos = points[i];  //(points[6 * i], points[6 * i + 1], points[6 * i + 2]);
            if (!idList.empty() && ((voxelCount < prevActiveVoxels) || (voxelCount < idList.size()))) {
                numUpdates++;
                auto voxelBody = voxelBodyList[idList[voxelCount]];
                float offsetX = offsetXList[idList[voxelCount]];
                float offsetY = offsetYList[idList[voxelCount]];
                ChVector3d voxelPos(pos.x() + offsetX, pos.y() + offsetY, pos.z());
                // double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
                // double yRot = voxelPos.y();
                // double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

                // Create a new rotated vector
                // ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
                voxelBody->SetPos(voxelPos);
                // voxelBody->SetRot(QuatFromAngleY(-slope_angle));
                //  voxelBody->SetPos({voxelPos.x(), voxelPos.y(), voxelPos.z()});
            }
            // Create a sphere for each point
            else if (numVoxelsToAdd > 0 && voxelCount >= prevActiveVoxels) {
                // numAdds++;
                // std::shared_ptr<ChBody> voxelBody;
                // if (true) {
                //     // std::cout << "Adding Mesh " << i << std::endl;
                //     int meshIndex = distribution(generator);  // Randomly select a mesh (if needed)
                //     auto trimesh_shape = regolith_meshes[meshIndex];
                //     trimesh_shape->SetScale(randscale(generator));
                //     if (trimesh_shape->GetNumMaterials() == 0) {
                //         trimesh_shape->AddMaterial(vis_mat);
                //     } else {
                //         trimesh_shape->GetMaterials()[0] = vis_mat;
                //     }
                //     voxelBody = chrono_types::make_shared<ChBody>();
                //     voxelBody->AddVisualShape(trimesh_shape);
                // } else {
                //     auto voxelBody = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);
                // }
                // float offsetX = randpos(generator);
                // float offsetY = randpos(generator);
                //// Set the position and other properties of the voxel body
                // ChVector3d voxelPos(pos.x() + offsetX, pos.y() + offsetY, pos.z());
                // double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
                // double yRot = voxelPos.y();
                // double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

                // ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
                // voxelBody->SetPos(voxelPos);
                ////voxelBody->SetRot(QuatFromAngleY(-slope_angle));
                // voxelBody->SetFixed(true);

                // int index = voxelBodyList.size();
                // voxelBodyList.push_back(voxelBody);
                //{
                //     auto shape = voxelBody->GetVisualModel()->GetShapeInstances()[0].first;
                //     if (shape->GetNumMaterials() == 0) {
                //         shape->AddMaterial(vis_mat);
                //     } else {
                //         shape->GetMaterials()[0] = vis_mat;
                //     }
                // }
                // idList.emplace_back(index);
                // offsetXList.emplace_back(offsetX);
                // offsetYList.emplace_back(offsetY);
            }
            voxelCount++;
        }

    } else {
        voxelBodyList.resize(activeVoxels);
        idList.resize(activeVoxels);
        offsetXList.resize(activeVoxels);
        offsetYList.resize(activeVoxels);

        // std::atomic<int> voxelCount(0);  // Thread-safe counter for the voxels

        // Use std::for_each with parallel execution
        std::for_each(std::execution::par, points.begin(), points.begin() + activeVoxels, [&](ChVector3d& point) {
            // for (int i = 0; i < points.size(); i++) {
            //  Calculate the index based on the position in the loop
            int i = &point - &points[0];  // Get the current index

            thread_local std::mt19937 generator(std::random_device{}());
            std::uniform_int_distribution<int> distribution(0, num_meshes - 1);
            std::uniform_real_distribution<float> randpos(-.005f, .005f);
            std::uniform_real_distribution<float> randscale(1.5f, 2.f);
            // Compute voxel position in world space
            ChVector3d pos = point;  // points[i];//(points[6 * i], points[6 * i + 1], points[6 * i + 2]);
            // Create voxelBody if necessary
            if (numVoxelsToAdd > 0 && i >= prevActiveVoxels) {
                std::shared_ptr<ChBody> voxelBody;
                if (true) {
                    // std::cout << "Adding Mesh " << i << std::endl;
                    int meshIndex = distribution(generator);  // Randomly select a mesh (if needed)
                    auto trimesh_shape = regolith_meshes[meshIndex];
                    trimesh_shape->SetScale(randscale(generator));
                    if (trimesh_shape->GetNumMaterials() == 0) {
                        trimesh_shape->AddMaterial(vis_mat);
                    } else {
                        trimesh_shape->GetMaterials()[0] = vis_mat;
                    }
                    voxelBody = chrono_types::make_shared<ChBody>();
                    voxelBody->AddVisualShape(trimesh_shape);

                } else {
                    // Create a sphere voxel
                    voxelBody = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);

                    auto shape = voxelBody->GetVisualModel()->GetShapeInstances()[0].first;
                    if (shape->GetNumMaterials() == 0) {
                        shape->AddMaterial(vis_mat);
                    } else {
                        shape->GetMaterials()[0] = vis_mat;
                    }
                }

                float offsetX = randpos(generator);
                float offsetY = randpos(generator);
                // Set the position and other properties of the voxel body
                ChVector3d voxelPos(pos.x() + offsetX, pos.y() + offsetY, pos.z());
                // double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
                // double yRot = voxelPos.y();
                // double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

                //// Create a new rotated vector
                // ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
                voxelBody->SetPos(voxelPos);
                voxelBody->SetFixed(true);

                // Directly assign the voxelBody and index to the preallocated list positions
                voxelBodyList[i] = voxelBody;
                idList[i] = i;  // Assign index to idList slot
                offsetXList[i] = offsetX;
                offsetYList[i] = offsetY;
            }
        });
        //}
    }
    prevActiveVoxels = activeVoxels;
    std::cout << "Num Voxels: " << voxelBodyList.size() << std::endl;
    scene->SetSprites(voxelBodyList);
    firstInst = false;
}