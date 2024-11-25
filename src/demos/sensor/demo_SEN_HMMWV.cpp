// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Chrono demonstration of the sensor module
// Attach multiple sensors to a hmmwv full vehicle model
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsCreators.h"


#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
// #include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"

// #include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
// #include "chrono_sensor/filters/ChFilterSave.h"
// #include "chrono_sensor/filters/ChFilterSavePtCloud.h"
// #include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::sensor;

// -----------------------------------------------------------------------------
// HMMWV parameters
// -----------------------------------------------------------------------------

// Initial vehicle location and orientation
ChVector3d initLoc(-45, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

enum DriverMode { DEFAULT, RECORD, PLAYBACK };
DriverMode driver_mode = DEFAULT;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, HULLS, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of powertrain models (SHAFTS, SIMPLE)
EngineModelType engine_model = EngineModelType::SHAFTS;
TransmissionModelType transmission_model = TransmissionModelType::AUTOMATIC_SHAFTS;

// Drive type (FWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

SteeringTypeWV steering_type = SteeringTypeWV::PITMAN_ARM;

// Type of tire model (RIGID, RIGID_MESH, TMEASY, FIALA, PAC89, PAC02)
TireModelType tire_model = TireModelType::PAC02;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// -----------------------------------------------------------------------------
// Sensor parameters
// -----------------------------------------------------------------------------

// Update rates of each sensor in Hz
float cam_update_rate = 30;
float lidar_update_rate = 10;

float exposure_time = 0.02f;

int super_samples = 2;

// Image width and height
unsigned int image_width = 1280;
unsigned int image_height = 720;

// Lidar horizontal and vertical samples
unsigned int horizontal_samples = 4500;
unsigned int vertical_samples = 32;

// Camera's horizontal field of view
float cam_fov = 1.408f;

// Lidar's horizontal and vertical fov
float lidar_hfov = (float)(2 * CH_PI);  // 360 degrees
float lidar_vmax = (float)CH_PI / 12;   // 15 degrees up
float lidar_vmin = (float)-CH_PI / 6;   // 30 degrees down
float lidar_max_distance = 100.0f;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// IMU parameters
// -----------------------------------------------------------------------------
// Noise model attached to the sensor
enum IMUNoiseModel {
    NORMAL_DRIFT,  // gaussian drifting noise with noncorrelated equal distributions
    IMU_NONE       // no noise added
};
IMUNoiseModel imu_noise_type = IMU_NONE;

// IMU update rate in Hz
float imu_update_rate = 100.0f;

// IMU lag (in seconds) between sensing and when data becomes accessible
float imu_lag = 0.f;

// IMU collection time (in seconds) of each sample
float imu_collection_time = 0.f;

// Simulation step sizes
double step_size = 2e-3;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "hmmwv";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string sens_dir = out_dir + "/SENSOR_OUTPUT";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// POV-Ray output
bool povray_output = false;

// Save sensor data
bool sensor_save = false;

// Visualize sensor data
bool sensor_vis = true;

// Located in Madison, WI
ChVector3d gps_reference(-89.400, 43.070, 260.0);
// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // --------------
    // Create systems
    // --------------

    // Create the hmmwv vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(contact_method);
    my_hmmwv.SetChassisCollisionType(chassis_collision_type);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetEngineType(engine_model);
    my_hmmwv.SetTransmissionType(transmission_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetSteeringType(steering_type);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.Initialize();

    if (tire_model == TireModelType::RIGID_MESH)
        tire_vis_type = VisualizationType::MESH;

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type);
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type);
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type);
    my_hmmwv.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::PatchType::BOX:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            break;
        case RigidTerrain::PatchType::HEIGHT_MAP:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"), 128,
                                     128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::PatchType::MESH:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }

    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("HMMWV Demo");
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&my_hmmwv.GetVehicle());

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    // Initialize output file for driver inputs
    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::ChWriterCSV driver_csv(" ");

    // Set up vehicle output
    my_hmmwv.GetVehicle().SetChassisOutput(true);
    my_hmmwv.GetVehicle().SetSuspensionOutput(0, true);
    my_hmmwv.GetVehicle().SetSteeringOutput(0, true);
    my_hmmwv.GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    my_hmmwv.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    ChInteractiveDriverIRR driver(*vis);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // If in playback mode, attach the data file to the driver system and
    // force it to playback the driver inputs.
    if (driver_mode == PLAYBACK) {
        driver.SetInputDataFile(driver_file);
        driver.SetInputMode(ChInteractiveDriver::InputMode::DATAFILE);
    }

    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    // output vehicle mass
    std::cout << "VEHICLE MASS: " << my_hmmwv.GetVehicle().GetMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    ////int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    int render_frame = 0;
    double time = 0;

    if (contact_vis) {
        vis->SetSymbolScale(1e-4);
        vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
    }

    // ---------------------------------------------
    // Create a sensor manager and add a point light
    // ---------------------------------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(my_hmmwv.GetSystem());
    manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 5000);
    manager->scene->SetAmbientLight({0, 0, 0});
    manager->scene->SetFogScatteringFromDistance(200.0);

    // manager->scene->SetFogColor({1.0,1.0,1.0});

    // Set environment map
    Background b;
    b.mode = BackgroundMode::GRADIENT;
    b.color_horizon = {0.6f, 0.7f, 0.8f};
    b.color_zenith = {0.4f, 0.5f, 0.6f};
    manager->scene->SetBackground(b);

    std::shared_ptr<ChNoiseModel> acc_noise_model;
    std::shared_ptr<ChNoiseModel> gyro_noise_model;
    std::shared_ptr<ChNoiseModel> mag_noise_model;
    switch (imu_noise_type) {
        case NORMAL_DRIFT:
            // Set the imu noise model to a gaussian model
            acc_noise_model =
                chrono_types::make_shared<ChNoiseNormalDrift>(100.f,                              // double updateRate,
                                                              ChVector3d({0., 0., 0.}),           // double mean,
                                                              ChVector3d({0.001, 0.001, 0.001}),  // double stdev,
                                                              .0001,                              // double bias_drift,
                                                              .1);                                // double tau_drift,
            gyro_noise_model =
                chrono_types::make_shared<ChNoiseNormalDrift>(100.f,                     // float updateRate,
                                                              ChVector3d({0., 0., 0.}),  // float mean,
                                                              ChVector3d({0.0075, 0.0075, 0.0075}),  // float stdev,
                                                              .001,  // double bias_drift,
                                                              .1);   // double tau_drift,
            mag_noise_model =
                chrono_types::make_shared<ChNoiseNormal>(ChVector3d({0., 0., 0.}),            // float mean,
                                                         ChVector3d({0.001, 0.001, 0.001}));  // float stdev,
            break;
        case IMU_NONE:
            // Set the imu noise model to none (does not affect the data)
            acc_noise_model = chrono_types::make_shared<ChNoiseNone>();
            gyro_noise_model = chrono_types::make_shared<ChNoiseNone>();
            mag_noise_model = chrono_types::make_shared<ChNoiseNone>();
            break;
    }

    auto imu_offset_pose = chrono::ChFrame<double>({0, 0, 1.45}, QuatFromAngleAxis(0, {1, 0, 0}));
    auto acc =
        chrono_types::make_shared<ChAccelerometerSensor>(my_hmmwv.GetChassisBody(),  // body to which the IMU is attached
                                                         imu_update_rate,         // update rate
                                                         imu_offset_pose,         // offset pose from body
                                                         acc_noise_model);        // IMU noise model
    acc->SetName("IMU - Accelerometer");
    acc->SetLag(imu_lag);
    acc->SetCollectionWindow(imu_collection_time);
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());  // Add a filter to access the imu data
    manager->AddSensor(acc);                                            // Add the IMU sensor to the sensor manager

    auto gyro =
        chrono_types::make_shared<ChGyroscopeSensor>(my_hmmwv.GetChassisBody(),  // body to which the IMU is attached
                                                     imu_update_rate,                     // update rate
                                                     imu_offset_pose,         // offset pose from body
                                                     gyro_noise_model);       // IMU noise model
    gyro->SetName("IMU - Gyroscope");
    gyro->SetLag(imu_lag);
    gyro->SetCollectionWindow(imu_collection_time);
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());  // Add a filter to access the imu data
    manager->AddSensor(gyro);                                           // Add the IMU sensor to the sensor manager

    auto mag =
        chrono_types::make_shared<ChMagnetometerSensor>(my_hmmwv.GetChassisBody(),  // body to which the IMU is attached
                                                        imu_update_rate,                     // update rate
                                                        imu_offset_pose,         // offset pose from body
                                                        mag_noise_model,
                                                        gps_reference);  // IMU noise model
    mag->SetName("IMU - Magnetometer");
    mag->SetLag(imu_lag);
    mag->SetCollectionWindow(imu_collection_time);
    mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());  // Add a filter to access the imu data
    manager->AddSensor(mag);


    // ---------------
    // Simulate system
    // ---------------
    float orbit_radius = 15.f;
    float orbit_rate = 1;
    chrono::sensor::UserAccelBufferPtr bufferAcc;
    chrono::sensor::UserGyroBufferPtr bufferGyro;
    chrono::sensor::UserMagnetBufferPtr bufferMag;
    // Create a CSV writer to record the IMU data
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    utils::ChWriterCSV imu_csv(" ");
    unsigned int imu_last_launch = 0;

    while (vis->Run()) {
        time = my_hmmwv.GetSystem()->GetChTime();
        if (time < 5){
            driver.SetThrottle(0.4);
            driver.SetSteering(0.0);
        }
        else if (time > 5 && time < 7){
            driver.SetThrottle(0.4);
            driver.SetSteering(0.3);
        }
        else if (time > 7 && time < 9){
            driver.SetThrottle(0.4);
            driver.SetSteering(0.0);
        }
        else if (time > 9 && time < 11){
            driver.SetThrottle(0.4);
            driver.SetSteering(-0.3);
        }
        else{
            driver.SetThrottle(0.0);
            driver.SetSteering(0.0);
            driver.SetBraking(1.0);
        }
        // End simulation
        if (time >= t_end)
            break;

        // Get the most recent imu data
        bufferAcc = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
        bufferGyro = gyro->GetMostRecentBuffer<UserGyroBufferPtr>();
        bufferMag = mag->GetMostRecentBuffer<UserMagnetBufferPtr>();
        if (bufferAcc->Buffer && bufferGyro->Buffer && bufferMag->Buffer &&
            bufferMag->LaunchedCount > imu_last_launch) {
            // Save the imu data to file
            AccelData acc_data = bufferAcc->Buffer[0];
            GyroData gyro_data = bufferGyro->Buffer[0];
            MagnetData mag_data = bufferMag->Buffer[0];

            imu_csv << std::fixed << std::setprecision(6);
            imu_csv << acc_data.X;
            imu_csv << acc_data.Y;
            imu_csv << acc_data.Z;
            imu_csv << gyro_data.Roll;
            imu_csv << gyro_data.Pitch;
            imu_csv << gyro_data.Yaw;
            imu_csv << mag_data.X;
            imu_csv << mag_data.Y;
            imu_csv << mag_data.Z;
            imu_csv << std::endl;
            imu_last_launch = bufferMag->LaunchedCount;
            std::cout<< "IMU DATA: " << acc_data.X << " " << acc_data.Y << " " << acc_data.Z << " " << gyro_data.Roll << " " << gyro_data.Pitch << " " << gyro_data.Yaw << " " << mag_data.X << " " << mag_data.Y << " " << mag_data.Z << std::endl;
        }

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(my_hmmwv.GetSystem(), filename);
            }

            render_frame++;
        }

        // // Debug logging
        // if (debug_output && step_number % debug_steps == 0) {
        //     std::cout << "\n\n============ System Information ============" << std::endl;
        //     std::cout << "Time = " << time << std::endl << std::endl;
        //     my_hmmwv.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
        // }

        // Collect output data from modules (for inter-module communication)
        DriverInputs driver_inputs = driver.GetInputs();

        // Driver output
        if (driver_mode == RECORD) {
            driver_csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking
                       << std::endl;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        double step = step_size;
        driver.Advance(step);
        terrain.Advance(step);
        my_hmmwv.Advance(step);
        vis->Advance(step);

        // Update the sensor manager
        // Will render/save/filter automatically
        manager->Update();

        // Increment frame number
        step_number++;
    }

    if (driver_mode == RECORD) {
        driver_csv.WriteToFile(driver_file);
    }
    std::string imu_file = out_dir + "/hmmwv.csv";
    imu_csv.WriteToFile(imu_file);
    return 0;
}
