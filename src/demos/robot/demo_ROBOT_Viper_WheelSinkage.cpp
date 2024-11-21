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
// Author: Wei Hu, Jason Zhou, Nevindu M. Batagoda
// Chrono::FSI demo to show usage of VIPER rover models on SPH granular terrain
// This demo uses a plug-in VIPER rover model from chrono::models
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/visualization/ChFsiVisualization.h"

#include "chrono_thirdparty/filesystem/path.h"

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

#ifdef USE_SENSOR_NVDB
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/ParticlesToLevelSet.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
#include <execution>
#endif


#include <random>

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::viper;
using namespace chrono::sensor;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::OpenGL;

// Physical properties of terrain particles
double iniSpacing;
double kernelLength;
double density;
double slope_angle;
double total_mass;

// output directories and settings
std::string out_dir = "/root/sbel/outputs/FSI_Viper_WheelSinkage";

// Dimension of the space domain
double bxDim = 6.0;
double byDim = 4.0;
double bzDim = 0.2;

// Rover initial location
ChVector3d init_loc(1.0 - bxDim / 2.0 + .25f, 0, bzDim + 0.4);

// Simulation time and stepsize
double total_time = 20.0;
double dT;

// Save data as csv files to see the results off-line using Paraview
bool output = false;
int out_fps = 10;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = false;
float render_fps = 30;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// Use below mesh file if the wheel type is real VIPER wheel
std::string wheel_obj = "robot/viper/obj/viper_wheel.obj";

// wheel specifics
double wheel_radius = 0.25;
double wheel_wide = 0.2;
double grouser_height = 0.025;
double grouser_wide = 0.005;
int grouser_num = 24;
double wheel_AngVel = 0.8;

// Pointer to store the VIPER instance
std::shared_ptr<Viper> rover;

// Pointer to store the VIPER driver
std::shared_ptr<ViperSpeedDriver> driver;

// std::shared_ptr<ChBodyAuxRef> rock_Body;
int rock_id = -1;
bool add_rocks = false;

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
const std::string sensor_out_dir = "SENSOR_OUTPUT/ViperWheelSinkage";

bool use_gi = false;

// NANO VDB
// nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> nvdb_handle;

std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.2f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChContactMaterial>();
    }
}

// Forward declaration of helper functions
void SaveParaViewFiles(ChSystemFsi& sysFSI, ChSystemNSC& sysMBS, double mTime);
void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI);

// VDB info
bool firstInst = true;
std::vector<int> idList;
int prevActiveVoxels = 0;
std::vector<std::shared_ptr<ChBody>> voxelBodyList = {};
std::vector<float> offsetXList = {};
std::vector<float> offsetYList = {};
int activeVoxels = 0;

#ifdef USE_SENSOR_NVDB
// forward declarations
void createVoxelGrid(std::vector<float> points,
                     ChSystemNSC& sys,
                     std::shared_ptr<ChScene> scene,
                     std::shared_ptr<ChVisualMaterial> vis_mat);  // std::vector<openvdb::Vec3R>&

template <typename ValueType>
class FloatBufferAttrVector {
  public:
    using PosType = ValueType;
    using value_type = openvdb::Vec3d;
    FloatBufferAttrVector(const std::vector<float> data, const openvdb::Index stride = 1)
        : mData(data), mStride(stride) {}

    size_t size() const { return mData.size() / 6; }
    void getPos(size_t n, ValueType& xyz) const {
        xyz = ValueType(mData[6 * n], mData[6 * n + 1], mData[6 * n + 2]);
    }
    // void get(ValueType& value, size_t n) const { value = mData[n]; }
    // void get(ValueType& value, size_t n, openvdb::Index m) const { value = mData[n * mStride + m]; }

  private:
    const const std::vector<float> mData;
    const openvdb::Index mStride;
};  // PointAttributeVector
#else
void createVoxelGrid(std::vector<float> points,
                     ChSystemNSC& sys,
                     std::shared_ptr<ChScene> scene,
                     std::shared_ptr<ChVisualMaterial> vis_mat);
#endif
int num_meshes = 100;
std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>> regolith_meshes;  // ChVisualShapeTriangleMesh 


int main(int argc, char* argv[]) {
    // The path to the Chrono data directory
    // SetChronoDataPath(CHRONO_DATA_DIR);

    // Create a physical system and a corresponding FSI system
    ChSystemNSC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    // Use JSON file to set the FSI parameters

    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Viper_granular_NSC.json");

    total_mass = 440.0;
    slope_angle = 0.0 / 180.0 * CH_PI;
    wheel_AngVel = 0.8;
    out_dir = GetChronoOutputPath() + "FSI_Viper_Slope_WheelSinkage/";
    /*  if (argc == 4) {
          total_mass = std::stod(argv[1]);
          slope_angle = std::stod(argv[2]) / 180.0 * CH_PI;
          wheel_AngVel = std::stod(argv[3]);
          out_dir = out_dir + std::to_string(std::stoi(argv[2])) + "/";
      } else if (argc != 4) {
          std::cout << "usage: ./demo_ROBOT_Viper_RealSlope <total_mass> <slope_angle> <wheel_angVel>" << std::endl;
          return 1;
      }*/

    // Create oputput directories
    if (!filesystem::create_subdirectory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_subdirectory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }
    if (!filesystem::create_subdirectory(filesystem::path(out_dir + "/fsi"))) {
        std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
        return 1;
    }
    if (!filesystem::create_subdirectory(filesystem::path(out_dir + "/rover"))) {
        std::cerr << "Error creating directory " << out_dir + "/rover" << std::endl;
        return 1;
    }

    sysFSI.ReadParametersFromFile(inputJson);

    double gravity_G = sysFSI.GetGravitationalAcceleration().z();  // Is g already set?
    ChVector3d gravity = ChVector3d(gravity_G * sin(slope_angle), 0, gravity_G * cos(slope_angle));
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    // Get the simulation stepsize
    dT = sysFSI.GetStepSize();

    // Get the initial particle spacing
    iniSpacing = sysFSI.GetInitialSpacing();

    // Get the SPH kernel length
    kernelLength = sysFSI.GetKernelLength();

    // // Set the initial particle spacing
    //sysFSI.SetInitialSpacing(iniSpacing);

    // // Set the SPH kernel length
   // sysFSI.SetKernelLength(kernelLength);

    // // Set the terrain density
    //sysFSI.SetDensity(density);

    // // Set the simulation stepsize
    //sysFSI.SetStepSize(dT);

    // Set the simulation domain size
    sysFSI.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetConsistentDerivativeDiscretization(false, false);

    // Setup the solver based on the input value of the prameters
    sysFSI.SetSPHMethod(SPHMethod::WCSPH);

    //// Set wall boundary condition - obsolete
    //sysFSI.SetWallBC(BceVersion::ADAMI);

    //// Set rigid body boundary condition
    //sysFSI.SetRigidBodyBC(BceVersion::ADAMI);

    // Set cohsion of the granular material
    sysFSI.SetCohesionForce(0.0);

  

    // Set the periodic boundary condition
    ChVector3d cMin(-bxDim / 2 * 2, -byDim / 2 - 0.5 * iniSpacing, -bzDim * 10);
    ChVector3d cMax(bxDim / 2 * 2, byDim / 2 + 0.5 * iniSpacing, bzDim * 20);
    sysFSI.SetBoundaries(cMin, cMax);

    // Set simulation data output length
    sysFSI.SetOutputLength(1);

    // Create an initial box for the terrain patch
    chrono::utils::ChGridSampler<> sampler(iniSpacing);
    ChVector3d boxCenter(0, 0, bzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    auto gz = std::abs(gravity.z());
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        double pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + bzDim);
        sysFSI.AddSPHParticle(points[i], sysFSI.GetDensity(), 0, sysFSI.GetViscosity(),
                              ChVector3d(0),         // initial velocity
                              ChVector3d(-pre_ini),  // tauxxyyzz
                              ChVector3d(0)          // tauxyxzyz
        );
    }

    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();


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
    regolith_material->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));
    regolith_material->SetClassID(30000);
    regolith_material->SetInstanceID(20000);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sysMBS.Add(floor);

    // Create a Sensor manager
    float intensity = 1.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sysMBS);
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
        {0, 5, 1}, QuatFromAngleAxis(-CH_PI_2, {0, 0, 1}));  // Q_from_AngAxis(CH_PI_4, {0, 1, 0})  //-1200, -252, 100
    auto cam = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose1,  // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
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

    chrono::ChFrame<double> offset_pose2({-0.f, -1.7, 0.5}, QuatFromAngleAxis(.2, {-2, 3, 9.75}));
    auto cam2 = chrono_types::make_shared<ChCameraSensor>(rover->GetChassis()->GetBody(),  // body camera is attached to
                                                          update_rate,                     // update rate in Hz
                                                          offset_pose2,                    // offset pose
                                                          image_width,                     // image width
                                                          image_height,                    // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // super sampling factor
                                                          lens_model,    // lens model type
                                                          use_denoiser);
    cam2->SetIntegrator(integrator);
    cam2->SetName("Wheel Camera");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);
    if (vis)
        cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Wheel Camera"));

    if (save)
        cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Wheel_Cam_RealSlope/"));
    manager->AddSensor(cam2);


    auto seg = chrono_types::make_shared<ChSegmentationCamera>(rover->GetChassis()->GetBody(),   // body camera is attached to
                                                               update_rate,   // update rate in Hz
                                                               offset_pose2,  // offset pose
                                                               image_width,   // image width
                                                               image_height,  // image height
                                                               fov,           // camera's horizontal field of view
                                                               lens_model);   // FOV
    seg->SetName("Semantic Segmentation Camera");
    seg->SetLag(lag);
    seg->SetCollectionWindow(exposure_time);
    

    // Render the semantic mask
    if (vis)
        seg->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Semantic Segmentation"));

    // Save the semantic mask
    if (save)
        seg->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "segmentation/"));

    manager->AddSensor(seg);

    chrono::ChFrame<double> offset_pose3({-2.f, 0, .5f}, QuatFromAngleAxis(.2, {0, 1, 0}));
    // chrono::ChFrame<double> offset_pose3({0, 0, 5.f}, Q_from_AngAxis(CH_PI_2, {0, 1, 0}));
    auto cam3 = chrono_types::make_shared<ChCameraSensor>(rover->GetChassis()->GetBody(),  // body camera is attached to
                                                          update_rate,                     // update rate in Hz
                                                          offset_pose3,                    // offset pose
                                                          image_width,                     // image width
                                                          image_height,                    // image height
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

    // chrono::ChFrame<double> offset_pose4({-2.f, 0, .5f}, Q_from_AngAxis(.2, {0, 1, 0}));
    chrono::ChFrame<double> offset_pose4({0, 0, 5.f}, QuatFromAngleAxis(CH_PI_2, {0, 1, 0}));
    auto cam4 = chrono_types::make_shared<ChCameraSensor>(rover->GetChassis()->GetBody(),  // body camera is attached to
                                                          update_rate,                     // update rate in Hz
                                                          offset_pose4,                    // offset pose
                                                          image_width,                     // image width
                                                          image_height,                    // image height
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

    // Write position and velocity to file
    std::ofstream ofile;
    if (output)
        ofile.open(out_dir + "./body_position.txt");


    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));
    double time = 0.0;
    int current_step = 0;

    // Get the chassis of the rover
    auto body = sysMBS.GetBodies()[1];

    ChTimer timer;
    ChTimer timerNVDB;

    float addNVDBTime = 0.0f;
    int sensor_render_steps = (unsigned int)round(1 / (update_rate * dT));
    std::vector<float> h_points;
    while (time < total_time) {
        std::cout << current_step << "  time: " << time << "  sim. time: " << timer() << " RTF: " << timer()/time <<  std::endl;
        if (add_rocks) {
            auto rock_body = sysMBS.GetBodies()[rock_id];
            // std::cout << "ID: " << rock_id << "Rock Pos: (" << rock_body->GetPos().x() << ", " << rock_body->GetPos().y()
            // << ", " << rock_body->GetPos().z() << ") "<< std::endl;
            if (rock_body->GetPos().z() < 0.31f)
                rock_body->SetFixed(true);
        }

                    
        if (current_step % sensor_render_steps == 0 && current_step > 0) {
            //timerNVDB.start();
            h_points = sysFSI.GetParticleData();
            #ifdef USE_SENSOR_NVDB
                createVoxelGrid(h_points, sysMBS, manager->scene, regolith_material);
            #else
                createVoxelGrid(h_points, sysMBS, manager->scene, regolith_material);
            #endif
            manager->Update();
        }

        rover->Update();

        std::cout << "  pos: " << body->GetPos() << std::endl;
        std::cout << "  vel: " << body->GetPosDt() << std::endl;
        if (output) {
            ofile << time << "  " << body->GetPos() << "    " << body->GetPosDt() << std::endl;
            if (current_step % output_steps == 0) {
                sysFSI.PrintParticleToFile(out_dir + "/particles");
                sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
                SaveParaViewFiles(sysFSI, sysMBS, time);
            }
        }

        //// Render system
        // if (render && current_step % render_steps == 0) {
        //    if (!fsi_vis.Render())
        //        break;
        //}

        timer.start();
        sysFSI.DoStepDynamics_FSI();
        timer.stop();

        time += dT;
        current_step++;
    }

    if (output)
        ofile.close();

    return 0;
}

#ifdef USE_SENSOR_NVDB
void createVoxelGrid(std::vector<float> points, ChSystemNSC& sys, std::shared_ptr<ChScene> scene, std::shared_ptr<ChVisualMaterial> vis_mat) {
    std::cout << "Creating OpenVDB Voxel Grid for " << points.size()/6 << "particles " << std::endl;
    openvdb::initialize();
    // openvdb::points::PointAttributeVector<openvdb::Vec3R> positionsWrapper(points);
    const FloatBufferAttrVector<openvdb::Vec3R> positionsWrapper(points);
    float spacing = iniSpacing/2.f;
    float r = spacing / 2;
    int pointsPerVoxel = 1;
    // std::vector<float> radius(points.size(), r);
    float voxelSize = spacing;  // openvdb::points::computeVoxelSize(positionsWrapper, pointsPerVoxel);
    // Print the voxel-size to cout
    std::cout << "VoxelSize=" << voxelSize << std::endl;
    // Create a transform using this voxel-size.
    openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(voxelSize);

    openvdb::tools::PointIndexGrid::Ptr pointIndexGrid =
        openvdb::tools::createPointIndexGrid<openvdb::tools::PointIndexGrid>(positionsWrapper, *transform);

    /*  openvdb::points::PointDataGrid::Ptr grid =
          openvdb::points::createPointDataGrid<openvdb:
          :points::NullCodec, openvdb::points::PointDataGrid>(points,*transform);*/

    openvdb::points::PointDataGrid::Ptr grid =
        openvdb::points::createPointDataGrid<openvdb::points::NullCodec, openvdb::points::PointDataGrid>(
            *pointIndexGrid, positionsWrapper, *transform, nullptr);

    // Set the name of the grid
    grid->setName("FSIPointData");
    openvdb::Vec3d minBBox = grid->evalActiveVoxelBoundingBox().min().asVec3d();
    openvdb::Vec3d maxBBox = grid->evalActiveVoxelBoundingBox().max().asVec3d();

    activeVoxels = grid->activeVoxelCount();
    // Print Grid information
    printf("############### VDB POINT GRID INFORMATION ################\n");
    printf("Voxel Size: %f %f %f\n", grid->voxelSize()[0], grid->voxelSize()[1], grid->voxelSize()[2]);
    printf("Grid Class: %d\n", grid->getGridClass());
    printf("Grid Type: %s\n", grid->gridType().c_str());
    printf("Upper Internal Nodes: %d\n", grid->tree().nodeCount()[2]);
    printf("Lower Internal Nodes: %d\n", grid->tree().nodeCount()[1]);
    printf("Leaf Nodes: %d\n", grid->tree().nodeCount()[0]);
    printf("Active Voxels: %d\n", grid->activeVoxelCount());
    printf("Min BBox: %f %f %f\n", minBBox[0], minBBox[1], minBBox[2]);
    printf("Max BBox: %f %f %f\n", maxBBox[0], maxBBox[1], maxBBox[2]);
    printf("############### END #############\n");



    // int voxelCount = 0;
    int numVoxelsToAdd = activeVoxels - prevActiveVoxels;
    int numAdds = 0;
    int numUpdates = 0;

    std::vector<openvdb::Coord> coords;
    for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf) {
        for (auto iter(leaf->cbeginValueOn()); iter; ++iter) {
            const openvdb::Coord coord = iter.getCoord();
            coords.push_back(coord);
        }
    }

    if (!firstInst) {
        thread_local std::mt19937 generator(std::random_device{}());
        std::uniform_int_distribution<int> distribution(0, num_meshes - 1);
        std::uniform_real_distribution<float> randpos(-.005f, .005f);
        std::uniform_real_distribution<float> randscale(1.f, 1.5);
        int voxelCount = 0;
        for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf) {
            for (auto iter(leaf->cbeginValueOn()); iter; ++iter) {
                // const float value = iter.getValue();
                const openvdb::Coord coord = iter.getCoord();
                openvdb::Vec3d voxelPos = grid->indexToWorld(coord);
                if (!idList.empty() && ((voxelCount < prevActiveVoxels) || (voxelCount < idList.size()))) {
                    numUpdates++;
                    auto voxelBody = voxelBodyList[idList[voxelCount]];
                    float offsetX = offsetXList[idList[voxelCount]];
                    float offsetY = offsetYList[idList[voxelCount]];
                    ChVector3d voxelPos(voxelPos.x() + offsetX, voxelPos.y() + offsetY, voxelPos.z());
                    double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
                    double yRot = voxelPos.y();
                    double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

                    // Create a new rotated vector
                    ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
                    voxelBody->SetPos(rotatedVoxelPos);
                    voxelBody->SetRot(QuatFromAngleY(-slope_angle));
                    // voxelBody->SetPos({voxelPos.x(), voxelPos.y(), voxelPos.z()});
                }
                // Create a sphere for each point
                else if (numVoxelsToAdd > 0 && voxelCount >= prevActiveVoxels) {
                    numAdds++;
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
                        auto voxelBody = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);
                    }
                    float offsetX = randpos(generator);
                    float offsetY = randpos(generator);
                    // Set the position and other properties of the voxel body
                    ChVector3d voxelPos(voxelPos.x() + offsetX, voxelPos.y() + offsetY, voxelPos.z());
                    double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
                    double yRot = voxelPos.y();
                    double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

                    ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
                    voxelBody->SetPos(rotatedVoxelPos);
                    voxelBody->SetRot(QuatFromAngleY(-slope_angle));
                    voxelBody->SetFixed(true);

                    int index = voxelBodyList.size();
                    voxelBodyList.push_back(voxelBody);
                    {
                    auto shape = voxelBody->GetVisualModel()->GetShapeInstances()[0].first;
                    if (shape->GetNumMaterials() == 0) {
                        shape->AddMaterial(vis_mat);
                    }
                    else {
                        shape->GetMaterials()[0] = vis_mat;
                    }
                    }
                    idList.emplace_back(index);
                    offsetXList.emplace_back(offsetX);
                    offsetYList.emplace_back(offsetY);
                }
                voxelCount++;
            }
        }

    }
 else {
 voxelBodyList.resize(coords.size());
 idList.resize(coords.size());
 offsetXList.resize(coords.size());
 offsetYList.resize(coords.size());

 // std::atomic<int> voxelCount(0);  // Thread-safe counter for the voxels

 // Use std::for_each with parallel execution
 std::for_each(std::execution::par, coords.begin(), coords.end(), [&](const openvdb::Coord& coord) {
     // Calculate the index based on the position in the loop
     int i = &coord - &coords[0];  // Get the current index

     thread_local std::mt19937 generator(std::random_device{}());
     std::uniform_int_distribution<int> distribution(0, num_meshes - 1);
     std::uniform_real_distribution<float> randpos(-.005f, .005f);
     std::uniform_real_distribution<float> randscale(1.f, 1.5);
     // Compute voxel position in world space
     openvdb::Vec3d voxelPos = grid->indexToWorld(coord);

     // Create voxelBody if necessary
     if (numVoxelsToAdd > 0 && i >= prevActiveVoxels) {
         std::shared_ptr<ChBody> voxelBody;
         if (true) {
             // std::cout << "Adding Mesh " << i << std::endl;
             int meshIndex = distribution(generator);  // Randomly select a mesh (if needed)
             auto trimesh_shape = regolith_meshes[meshIndex];
             trimesh_shape->SetScale(randscale(generator));
             ////std::cout << "OK" << std::endl;
             // auto trimesh_shape = std::make_shared<ChVisualShapeTriangleMesh>();
             // trimesh_shape->SetMesh(mmesh);
             ////std::cout << "OK1" << std::endl;
             // trimesh_shape->SetName("RegolithMesh");
             // trimesh_shape->SetMutable(false);
             if (trimesh_shape->GetNumMaterials() == 0) {
                 trimesh_shape->AddMaterial(vis_mat);
             }
             else {
                 trimesh_shape->GetMaterials()[0] = vis_mat;
             }
             voxelBody = chrono_types::make_shared<ChBody>();
             voxelBody->AddVisualShape(trimesh_shape);

         }
         else {
             // Create a sphere voxel
             voxelBody = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);

             auto shape = voxelBody->GetVisualModel()->GetShapeInstances()[0].first;
             if (shape->GetNumMaterials() == 0) {
                 shape->AddMaterial(vis_mat);
             }
             else {
                 shape->GetMaterials()[0] = vis_mat;
             }
         }

         float offsetX = randpos(generator);
         float offsetY = randpos(generator);
         // Set the position and other properties of the voxel body
         ChVector3d voxelPos(voxelPos.x() + offsetX, voxelPos.y() + offsetY, voxelPos.z());
         double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
         double yRot = voxelPos.y();
         double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

         // Create a new rotated vector
         ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
         voxelBody->SetPos(rotatedVoxelPos);
         voxelBody->SetFixed(true);

         // Directly assign the voxelBody and index to the preallocated list positions
         voxelBodyList[i] = voxelBody;
         idList[i] = i;  // Assign index to idList slot
         offsetXList[i] = offsetX;
         offsetYList[i] = offsetY;
     }
     });
    }
    prevActiveVoxels = coords.size();
    std::wcout << "Num Voxels: " << coords.size() << std::endl;
    scene->SetSprites(voxelBodyList);
    firstInst = false;
}
#else
void createVoxelGrid(std::vector<float> points,
    ChSystemNSC& sys,
    std::shared_ptr<ChScene> scene,
    std::shared_ptr<ChVisualMaterial> vis_mat) {

    std::cout << "Creating CPU Voxel Grid for " << points.size() / 6 << "particles " << std::endl;
    float spacing = iniSpacing / 2.f;
    float r = spacing / 2;
    int pointsPerVoxel = 1;
    float voxelSize = spacing;  
    std::cout << "VoxelSize=" << voxelSize << std::endl;

    activeVoxels = points.size() / 6;
    
    int numVoxelsToAdd = activeVoxels - prevActiveVoxels;
    int numAdds = 0;
    int numUpdates = 0;


    if (!firstInst) {
        thread_local std::mt19937 generator(std::random_device{}());
        std::uniform_int_distribution<int> distribution(0, num_meshes - 1);
        std::uniform_real_distribution<float> randpos(-.005f, .005f);
        std::uniform_real_distribution<float> randscale(1.f, 1.5);
        int voxelCount = 0;
        for (int i = 0; i < activeVoxels; i++) {
                 ChVector3d voxelPos(points[6*i],points[6*i + 1],points[6*i+2]);
                if (!idList.empty() && ((voxelCount < prevActiveVoxels) || (voxelCount < idList.size()))) {
                    numUpdates++;
                    auto voxelBody = voxelBodyList[idList[voxelCount]];
                    float offsetX = offsetXList[idList[voxelCount]];
                    float offsetY = offsetYList[idList[voxelCount]];
                    ChVector3d voxelPos(voxelPos.x() + offsetX, voxelPos.y() + offsetY, voxelPos.z());
                    double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
                    double yRot = voxelPos.y();
                    double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

                    // Create a new rotated vector
                    ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
                    voxelBody->SetPos(rotatedVoxelPos);
                    voxelBody->SetRot(QuatFromAngleY(-slope_angle));
                    // voxelBody->SetPos({voxelPos.x(), voxelPos.y(), voxelPos.z()});
                }
                // Create a sphere for each point
                else if (numVoxelsToAdd > 0 && voxelCount >= prevActiveVoxels) {
                    numAdds++;
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
                        auto voxelBody = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);
                    }
                    float offsetX = randpos(generator);
                    float offsetY = randpos(generator);
                    // Set the position and other properties of the voxel body
                    ChVector3d voxelPos(voxelPos.x() + offsetX, voxelPos.y() + offsetY, voxelPos.z());
                    double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
                    double yRot = voxelPos.y();
                    double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

                    ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
                    voxelBody->SetPos(rotatedVoxelPos);
                    voxelBody->SetRot(QuatFromAngleY(-slope_angle));
                    voxelBody->SetFixed(true);

                    int index = voxelBodyList.size();
                    voxelBodyList.push_back(voxelBody);
                    {
                        auto shape = voxelBody->GetVisualModel()->GetShapeInstances()[0].first;
                        if (shape->GetNumMaterials() == 0) {
                            shape->AddMaterial(vis_mat);
                        } else {
                            shape->GetMaterials()[0] = vis_mat;
                        }
                    }
                    idList.emplace_back(index);
                    offsetXList.emplace_back(offsetX);
                    offsetYList.emplace_back(offsetY);
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
        //std::for_each(std::execution::par, points.begin(), points.begin() + activeVoxels, [&](float& point) {
        for (int i = 0; i < points.size()/6; i++) {
            // Calculate the index based on the position in the loop
            //int i = &point - &points[0];  // Get the current index

            thread_local std::mt19937 generator(std::random_device{}());
            std::uniform_int_distribution<int> distribution(0, num_meshes - 1);
            std::uniform_real_distribution<float> randpos(-.005f, .005f);
            std::uniform_real_distribution<float> randscale(1.f, 1.5);
            // Compute voxel position in world space
            ChVector3d voxelPos(points[6 * i], points[6 * i + 1], points[6 * i + 2]);
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
                ChVector3d voxelPos(voxelPos.x() + offsetX, voxelPos.y() + offsetY, voxelPos.z());
                double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
                double yRot = voxelPos.y();
                double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

                // Create a new rotated vector
                ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
                voxelBody->SetPos(rotatedVoxelPos);
                voxelBody->SetFixed(true);

                // Directly assign the voxelBody and index to the preallocated list positions
                voxelBodyList[i] = voxelBody;
                idList[i] = i;  // Assign index to idList slot
                offsetXList[i] = offsetX;
                offsetYList[i] = offsetY;
            }
        //});
        }
    }
    prevActiveVoxels = activeVoxels;
    std::cout << "Num Voxels: " << voxelBodyList.size() << std::endl;
    scene->SetSprites(voxelBodyList);
    firstInst = false;

}
#endif
//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies and their
// BCE representations are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI) {

    // Create a body for the rigid soil container
    auto box = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 0.02, 1000, false, false);
    box->SetPos(ChVector3d(0, 0, 0));
    box->SetFixed(true);
    sysMBS.Add(box);

    // Get the initial SPH particle spacing
    double initSpace0 = sysFSI.GetInitialSpacing();

    // Fluid-Solid Coupling at the walls via BCE particles
    sysFSI.AddBoxContainerBCE(box,                                        //
                              ChFrame<>(ChVector3d(0, 0, bzDim), QUNIT),  //
                              ChVector3d(bxDim, byDim, 2 * bzDim),        //
                              ChVector3i(2, 0, -1));

    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    rover = chrono_types::make_shared<Viper>(&sysMBS, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    //rover->Initialize(ChFrame<>(init_loc, QUNIT));
    double xRot = init_loc.x() * cos(-slope_angle) + init_loc.z() * sin(-slope_angle);
    double yRot = init_loc.y();
    double zRot = -init_loc.x() * sin(-slope_angle) + init_loc.z() * cos(-slope_angle);

    // Create a new rotated vector
    ChVector3d rotatedRoverInitLoc(xRot, yRot, zRot);
    rover->Initialize(ChFrame<>(rotatedRoverInitLoc, QuatFromAngleY(-slope_angle)));


    // Create the wheel's BCE particles
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    std::vector<ChVector3d> BCE_wheel;
    sysFSI.CreateMeshPoints(*trimesh, initSpace0, BCE_wheel);

    // Add BCE particles and mesh of wheels to the system
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> wheel_body;
        if (i == 0) {
            wheel_body = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            wheel_body = rover->GetWheel(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            wheel_body = rover->GetWheel(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            wheel_body = rover->GetWheel(ViperWheelID::V_RB)->GetBody();
        }

        sysFSI.AddFsiBody(wheel_body);
        if (i == 0 || i == 2) {
            sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, QuatFromAngleZ(CH_PI)), true);
        } else {
            sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, QUNIT), true);
        }
    }
    //// Create a body for the rigid soil container
    //auto box = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 0.02, 1000, false, false);
    //box->SetPos(ChVector3d(0, 0, 0));
    //box->SetFixed(true);
    //sysMBS.Add(box);

    //// Fluid-Solid Coupling at the walls via BCE particles
    //sysFSI.AddBoxContainerBCE(box, ChFrame<>(), ChVector3d(bxDim, byDim, 2 * bzDim), ChVector3i(2, 0, -1));

    //driver = chrono_types::make_shared<ViperSpeedDriver>(0.1, wheel_AngVel);
    //rover = chrono_types::make_shared<Viper>(&sysMBS, wheel_type);
    //rover->SetDriver(driver);
    //rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));

    //rover->Initialize(ChFrame<>(init_loc, QUNIT));

    //// // Create the wheel's BCE particles
    //// auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    //// double scale_ratio = 1.0;
    //// trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    //// trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    //// trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    //// std::vector<ChVector3d> BCE_wheel;
    //// sysFSI.CreateMeshPoints(*trimesh, iniSpacing, BCE_wheel);

    //// Set the rover mass to a user mass
    //for (int i = 0; i < 17; i++) {
    //    double mass_scale = total_mass / 440.0;
    //    auto viper_part = sysMBS.GetBodies()[i + 1];
    //    double part_mass = viper_part->GetMass();
    //    ChVector3d part_inertia = viper_part->GetInertiaXX();
    //    viper_part->SetMass(part_mass * mass_scale);
    //    viper_part->SetInertiaXX(part_inertia * mass_scale);
    //}

    //// Add BCE particles and mesh of wheels to the system
    //for (int i = 0; i < 4; i++) {
    //    std::shared_ptr<ChBodyAuxRef> wheel_body;
    //    if (i == 0) {
    //        wheel_body = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
    //    }
    //    if (i == 1) {
    //        wheel_body = rover->GetWheel(ViperWheelID::V_RF)->GetBody();
    //    }
    //    if (i == 2) {
    //        wheel_body = rover->GetWheel(ViperWheelID::V_LB)->GetBody();
    //    }
    //    if (i == 3) {
    //        wheel_body = rover->GetWheel(ViperWheelID::V_RB)->GetBody();
    //    }

    //    sysFSI.AddFsiBody(wheel_body);
    //    // if (i == 0 || i == 2) {
    //    //     sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, Q_from_AngZ(CH_PI)), true);
    //    // } else {
    //    //     sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, QUNIT), true);
    //    // }
    //    double inner_radius = wheel_radius - grouser_height;
    //    sysFSI.AddWheelBCE_Grouser(wheel_body, ChFrame<>(), inner_radius, wheel_wide - iniSpacing, grouser_height,
    //                               grouser_wide, grouser_num, kernelLength, false);

    //    // wheel_body->GetCollisionModel()->AddCylinder();
    //}

    //{
    //    // Create the chassis of the test rig
    //    auto chassis = chrono_types::make_shared<ChBody>();
    //    chassis->SetMass(10.0);
    //    chassis->SetPos(init_loc);
    //    chassis->EnableCollision(false);
    //    chassis->SetFixed(false);

    //    // Add geometry of the chassis.
    //    // chassis->GetCollisionModel()->Clear();
    //    chrono::utils::AddBoxGeometry(chassis.get(), CustomWheelMaterial(ChContactMethod::NSC),
    //                                  ChVector3d(0.1, 0.1, 0.1), ChVector3d(0, 0, 0), ChQuaternion<>(1, 0, 0, 0),
    //                                  false);
    //    // chassis->GetCollisionModel()->BuildModel();

    //    sysMBS.AddBody(chassis);

    //    // // Create the axle
    //    // auto axle = chrono_types::make_shared<ChBody>();
    //    // axle->SetMass(10.0);
    //    // axle->SetPos(init_loc + ChVector3d(0, 0, 1));
    //    // axle->SetCollide(false);
    //    // axle->SetBodyFixed(false);

    //    // // Add geometry of the axle.
    //    // axle->GetCollisionModel()->ClearModel();
    //    // chrono::utils::AddSphereGeometry(axle.get(), CustomWheelMaterial(ChContactMethod::NSC), 0.5, ChVector3d(0, 0,
    //    // 0)); axle->GetCollisionModel()->BuildModel(); sysMBS.AddBody(axle);

    //    // Connect the chassis to the containing bin (ground) through a translational joint and create a linear
    //    // actuator.
    //    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    //    prismatic1->Initialize(box, chassis, ChFrame<>(chassis->GetPos(), QuatFromAngleY(CH_PI_2)));
    //    prismatic1->SetName("prismatic_chassis_ground");
    //    sysMBS.AddLink(prismatic1);

    //    // auto actuator_fun = chrono_types::make_shared<ChFunction_Ramp>(0.0, wheel_vel);
    //    // actuator->Initialize(box, chassis, false, ChCoordsys<>(chassis->GetPos(), QUNIT),
    //    //                     ChCoordsys<>(chassis->GetPos() + ChVector3d(1, 0, 0), QUNIT));
    //    // actuator->SetName("actuator");
    //    // actuator->SetDistanceOffset(1);
    //    // actuator->SetActuatorFunction(actuator_fun);
    //    // sysMBS.AddLink(actuator);

    //    // Connect the axle to the chassis through a vertical translational joint.
    //    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    //    auto rover_body = rover->GetChassis()->GetBody();
    //    prismatic2->Initialize(chassis, rover_body, ChFrame<>(chassis->GetPos(), QUNIT));
    //    prismatic2->SetName("prismatic_rover_chassis");
    //    sysMBS.AddLink(prismatic2);

    //    // // Connect the rover body to the axle through a engine joint.
    //    // auto lock_link = chrono_types::make_shared<ChLinkLockLock>();
    //    // auto rover_body = rover->GetChassis()->GetBody();
    //    // lock_link->SetName("rover_axle_lock");
    //    // lock_link->Initialize(axle, rover_body, ChCoordsys<>(chassis->GetPos(), QUNIT));
    //    // sysMBS.AddLink(lock_link);
    //    for (auto body : sysMBS.GetBodies()) {
    //        if (body->GetVisualModel()) {
    //            for (auto& shape_instance : body->GetVisualModel()->GetShapeInstances()) {
    //                const auto& shape = shape_instance.first;
    //                shape->SetVisible(true);
    //            }
    //        }
    //    }
    //}


            // Add rock obstacle
    if (add_rocks) {
        auto vis_mat2 = chrono_types::make_shared<ChVisualMaterial>();
        vis_mat2->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
        vis_mat2->SetDiffuseColor({1, 1, 1});
        vis_mat2->SetSpecularColor({1, 1, 1});
        vis_mat2->SetUseSpecularWorkflow(true);
        vis_mat2->SetRoughness(1.0f);
        vis_mat2->SetUseHapke(false);

        std::string rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock3.obj");
        ChVector3d rock_pos = ChVector3d(-.4, -.625, 0.5);

        std::shared_ptr<ChContactMaterial> rockSufaceMaterial =
            ChContactMaterial::DefaultMaterial(sysMBS.GetContactMethod());

        double scale_ratio = .5;
        auto rock_mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_obj_path, false, true);
        rock_mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        rock_mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight
        // compute mass inertia from mesh
        double mmass;
        ChVector3d mcog;
        ChMatrix33<> minertia;
        double mdensity = 8000;  // paramsH->bodyDensity;
        rock_mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector3d principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto rock_Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> rock_rot = QuatFromAngleX(CH_PI / 2);

        rock_Body->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

        rock_Body->SetMass(mmass * mdensity);  // mmass * mdensity
        rock_Body->SetInertiaXX(mdensity * principal_I);

        rock_Body->SetFrameRefToAbs(ChFrame<>(ChVector3d(rock_pos), ChQuaternion<>(rock_rot)));
        rock_Body->SetName("Rocky");
        sysMBS.Add(rock_Body);

        rock_Body->SetFixed(false);

        // rock_Body->GetCollisionModel()->Clear();
        auto rock1_ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rockSufaceMaterial, rock_mmesh,
                                                                                      false, false, 0.005);
        rock_Body->EnableCollision(true);

        std::vector<ChVector3d> BCE_Rock;
        double initSpace0 = sysFSI.GetInitialSpacing();
        sysFSI.CreateMeshPoints(*rock_mmesh, initSpace0, BCE_Rock);
        sysFSI.AddFsiBody(rock_Body);
        sysFSI.AddPointsBCE(rock_Body, BCE_Rock, ChFrame<>(mcog, QUNIT), true);

        auto rock_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        rock_mesh->SetMesh(rock_mmesh);
        rock_mesh->SetBackfaceCull(true);
        {
            if (rock_mesh->GetNumMaterials() == 0) {
                rock_mesh->AddMaterial(vis_mat2);
            } else {
                rock_mesh->GetMaterials()[0] = vis_mat2;
            }
        }
        rock_Body->AddVisualShape(rock_mesh);
        rock_id = rock_Body->GetIndex();
    }
}

//------------------------------------------------------------------
// Function to save the povray files of the MBD
//------------------------------------------------------------------
void SaveParaViewFiles(ChSystemFsi& sysFSI, ChSystemNSC& sysMBS, double mTime) {
    std::string rover_dir = out_dir + "/rover";
    std::string filename;
    static int frame_number = -1;
    frame_number++;

    // save rigid body position and rotation
    for (int i = 1; i < sysMBS.GetBodies().size(); i++) {
        auto body = sysMBS.GetBodies()[i];
        ChFrame<> ref_frame = body->GetFrameRefToAbs();
        ChVector3d pos = ref_frame.GetPos();
        ChQuaternion<> rot = ref_frame.GetRot();
        ChVector3d vel = body->GetPosDt();

        std::string delim = ",";
        filename = rover_dir + "/body_pos_rot_vel" + std::to_string(i) + ".csv";
        std::ofstream file;
        if (sysMBS.GetChTime() > 0)
            file.open(filename, std::fstream::app);
        else {
            file.open(filename);
            file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim << "q1" << delim
                 << "q2" << delim << "q3" << delim << "Vx" << delim << "Vy" << delim << "Vz" << std::endl;
        }

        file << sysMBS.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim << rot.e0()
             << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << delim << vel.x() << delim << vel.y()
             << delim << vel.z() << std::endl;

        file.close();
    }

    std::cout << "-------------------------------------" << std::endl;
    std::cout << " Output frame:  " << frame_number << std::endl;
    std::cout << " Time:          " << mTime << std::endl;
    std::cout << "-------------------------------------" << std::endl;
}
