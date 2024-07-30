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
// Author: Wei Hu, Jason Zhou
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



#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/ParticlesToLevelSet.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>




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
std::string out_dir = "/root/sbel/outputs/FSI_Viper_RealSlope_SlopeAngle_";

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

//std::shared_ptr<ChBodyAuxRef> rock_Body;
int rock_id = -1;

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
const std::string sensor_out_dir = "SENSOR_OUTPUT/";

bool use_gi = false;

// NANO VDB
//nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> nvdb_handle;



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

// forward declarations
void createVoxelGrid(std::vector<openvdb::Vec3R>& points, ChSystemNSC& sys, std::shared_ptr<ChScene> scene);
std::vector<openvdb::Vec3R> floatToVec3R(float* floatBuffer, int npts);

int main(int argc, char* argv[]) {
    // The path to the Chrono data directory
    // SetChronoDataPath(CHRONO_DATA_DIR);

    // Create a physical system and a corresponding FSI system
    ChSystemNSC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    // Use JSON file to set the FSI parameters

    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_ROBOT_Viper_RealSlope.json");

    total_mass = 440.0;
    slope_angle = 0.0 / 180.0 * CH_PI;
    wheel_AngVel = 0.8;
    out_dir = GetChronoOutputPath() + "FSI_Viper_RealSlope/";
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

    double gravity_G = sysFSI.GetGravitationalAcceleration().z(); // Is g already set?
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
    // sysFSI.SetInitialSpacing(iniSpacing);

    // // Set the SPH kernel length
    // sysFSI.SetKernelLength(kernelLength);

    // // Set the terrain density
    // sysFSI.SetDensity(density);

    // // Set the simulation stepsize
    // sysFSI.SetStepSize(dT);

    // Set the simulation domain size
    sysFSI.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetDiscreType(false, false);

    // Set wall boundary condition
    sysFSI.SetWallBC(BceVersion::ADAMI);

    // Set rigid body boundary condition
    sysFSI.SetRigidBodyBC(BceVersion::ADAMI);

    // Set cohsion of the granular material
    sysFSI.SetCohesionForce(0.0);

    // Setup the solver based on the input value of the prameters
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);

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
    float* h_points = sysFSI.GetParticleData();
    int n_pts = sysFSI.GetNumFluidMarkers();
    
    //
    // SENSOR SIMULATION BEGIN
    //
    // auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    // vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
    // vis_mat->SetDiffuseColor({0.71f, 0.54f, 0.26f});  // 0.701f, 0.762f, 0.770f
    // vis_mat->SetSpecularColor({0, 0, 0});
    // vis_mat->SetUseSpecularWorkflow(true);
    // vis_mat->SetRoughness(.5f);
    // vis_mat->SetClassID(30000);
    // vis_mat->SetInstanceID(50000);

    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
    vis_mat->SetDiffuseColor({0.29f, 0.29f, 0.235f});
    vis_mat->SetSpecularColor({1, 1, 1});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(1.0f);
    vis_mat->SetShader(1);
    vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(20000);

    float scal = 10.f;
    //auto vol_bbox = chrono_types::make_shared<ChNVDBVolume>(6.f, 4.f,.2f, 1000,
    //                                                        true);  
    //vol_bbox->SetPos({0, 0, 0});                                   
    //vol_bbox->SetFixed(true);
    //sysMBS.Add(vol_bbox);
    //{
    //    auto shape = vol_bbox->GetVisualModel()->GetShapes()[0].first;
    //    if (shape->GetNumMaterials() == 0) {
    //        shape->AddMaterial(vis_mat);
    //    } else {
    //        shape->GetMaterials()[0] = vis_mat;
    //    }
    //}

    //auto box = chrono_types::make_shared<ChBodyEasyBox>(6,4,0.2, 1000, true, false);
    //box->SetPos({0, 0, 0});
    //box->SetFixed(true);
    //sysMBS.Add(box);
    //{
    //    auto shape = box->GetVisualModel()->GetShapes()[0].first;
    //    if (shape->GetNumMaterials() == 0) {
    //        shape->AddMaterial(vis_mat);
    //    } else {
    //        shape->GetMaterials()[0] = vis_mat;
    //    }
    //}

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
    manager->SetVerbose(false);

    //manager->scene->SetFSIParticles(h_points);
    //manager->scene->SetFSINumFSIParticles(n_pts);


    // chrono::ChFrame<double> offset_pose1({0, 5, 0}, Q_from_AngAxis(0.2, {0, 0, 1}));  //-1200, -252, 100
    chrono::ChFrame<double> offset_pose1({0,5,1}, QuatFromAngleAxis(-CH_PI_2, {0, 0, 1}));  // Q_from_AngAxis(CH_PI_4, {0, 1, 0})  //-1200, -252, 100
    auto cam = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose1,  // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // super sampling factor
                                                         lens_model,    // lens model type
                                                         false, 2.2);
    cam->SetName("Third Person Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Third Person Camera"));

    if (save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "CRM_DEMO_THIRD_PERSON_VIEW_RealSlope/"));
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
                                                          false, 2.2);
    cam2->SetName("Wheel Camera");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);
    if (vis)
        cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Wheel Camera"));

    if (save)
        cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Wheel_Cam_RealSlope/"));
    manager->AddSensor(cam2);

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
                                                          false, 2.2);
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
                                                          false, 2.2);
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
    std::vector<openvdb::Vec3R> vdbBuffer;
    while (time < total_time) {
        std::cout << current_step << "  time: " << time << "  sim. time: " << timer() << " RTF: " << timer()/time <<  std::endl;
        auto rock_body = sysMBS.GetBodies()[rock_id];
        //std::cout << "ID: " << rock_id << "Rock Pos: (" << rock_body->GetPos().x() << ", " << rock_body->GetPos().y() << ", " << rock_body->GetPos().z() << ") "<< std::endl;
        if (rock_body->GetPos().z() < 0.31f)
            rock_body->SetFixed(true);
        
        if (current_step % render_steps == 0) {
           /* timerNVDB.start();
            createNanoVDBGrid(sysMBS, sysFSI, vol_bbox->GetId());
            timerNVDB.stop();
            std::cout << "Adding NVDB points:" << timerNVDB.GetTimeMilliseconds() << "ms" << std::endl;
            timerNVDB.reset();*/
           h_points = sysFSI.GetParticleData();
           n_pts = sysFSI.GetNumFluidMarkers();

           vdbBuffer = floatToVec3R(h_points, n_pts);
           createVoxelGrid(vdbBuffer, sysMBS, manager->scene);
           manager->scene->SetFSIParticles(h_points); // remove these later
           manager->scene->SetFSINumFSIParticles(n_pts);
        }
        manager->Update();

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


std::vector<openvdb::Vec3R> floatToVec3R(float* floatBuffer, int npts) {
    std::vector<openvdb::Vec3R> vec3RBuffer;
    for (size_t i = 0; i < npts; i++) {
        vec3RBuffer.emplace_back(floatBuffer[6 * i], floatBuffer[6 * i + 1], floatBuffer[6 * i + 2]);
    }
    return vec3RBuffer;
}

void createVoxelGrid(std::vector<openvdb::Vec3R>& points, ChSystemNSC& sys, std::shared_ptr<ChScene> scene) {
    std::cout << "Creating OpenVDB Voxel Grid" << std::endl;
    openvdb::initialize();
    openvdb::points::PointAttributeVector<openvdb::Vec3R> positionsWrapper(points);
    float r = 0.005f;
    int pointsPerVoxel = 1;
    std::vector<float> radius(points.size(), r);
    float voxelSize = openvdb::points::computeVoxelSize(positionsWrapper, pointsPerVoxel);
    // Print the voxel-size to cout
    std::cout << "VoxelSize=" << voxelSize << std::endl;
    // Create a transform using this voxel-size.
    openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(voxelSize);

    openvdb::tools::PointIndexGrid::Ptr pointIndexGrid =
        openvdb::tools::createPointIndexGrid<openvdb::tools::PointIndexGrid>(positionsWrapper, *transform);

    openvdb::points::PointDataGrid::Ptr grid =
        openvdb::points::createPointDataGrid<openvdb::points::NullCodec, openvdb::points::PointDataGrid>(points,
                                                                                                         *transform);

    using Codec = openvdb::points::FixedPointCodec</*1-byte=*/false, openvdb::points::UnitRange>;
    openvdb::points::TypedAttributeArray<float, Codec>::registerType();
    openvdb::NamePair radiusAttribute = openvdb::points::TypedAttributeArray<float, Codec>::attributeType();
    openvdb::points::appendAttribute(grid->tree(), "pscale", radiusAttribute);
    // Create a wrapper around the radius vector.
    openvdb::points::PointAttributeVector<float> radiusWrapper(radius);
    // Populate the "pscale" attribute on the points
    openvdb::points::populateAttribute<openvdb::points::PointDataTree, openvdb::tools::PointIndexTree,
                                       openvdb::points::PointAttributeVector<float>>(
        grid->tree(), pointIndexGrid->tree(), "pscale", radiusWrapper);

    // Set the name of the grid
    grid->setName("FSIPointData");
    openvdb::Vec3d minBBox = grid->evalActiveVoxelBoundingBox().min().asVec3d();
    openvdb::Vec3d maxBBox = grid->evalActiveVoxelBoundingBox().max().asVec3d();

    int activeVoxels = grid->activeVoxelCount();
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

    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
    vis_mat->SetDiffuseColor({1, 1, 1});  // 0.29f, 0.29f, 0.235f
    vis_mat->SetSpecularColor({1, 1, 1});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(1.0f);
    vis_mat->SetShader(1);
    vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(20000);

    int voxelCount = 0;
    int numVoxelsToAdd = activeVoxels - prevActiveVoxels;
    int numAdds = 0;
    int numUpdates = 0;

    for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf) {
        for (auto iter(leaf->cbeginValueOn()); iter; ++iter) {
            // const float value = iter.getValue();
            const openvdb::Coord coord = iter.getCoord();
            openvdb::Vec3d voxelPos = grid->indexToWorld(coord);
            // std::wcout << "Voxel Pos: (" << voxelPos[0] << "," << voxelPos[1] << "," << voxelPos[2] << std::endl;
            // Update existing spheres
            /* if (!firstInst && voxelCount >= 500000)
                 std::cout << numVoxelsToAdd << " " << prevActiveVoxels << " " << voxelCount << " " << idList.size()
                           << " " << numAdds << " " << numUpdates << std::endl;*/
            if (!idList.empty() && voxelCount < prevActiveVoxels) {
                /* if (voxelCount >= 500000)
                     std::cout << "Updating sphere: " << idList[voxelCount] << " Total Bodies: "<<
                   sys.GetBodies().size() << std::endl;*/
                numUpdates++;
                // std::cout << voxelCount << "," << idList.size() << std::endl;
                // auto voxelBody = sys.GetBodies()[idList[voxelCount]];
                // auto voxelBody =
                // std::reinterpret_pointer_cast<ChBody>(sys.GetOtherPhysicsItems()[idList[voxelCount]]); auto voxelBody
                // = scene->GetSprite(idList[voxelCount]);
                auto voxelBody = voxelBodyList[idList[voxelCount]];
                voxelBody->SetPos({voxelPos.x(), voxelPos.y(), voxelPos.z()});
            }
            // Create a sphere for each point
            if (numVoxelsToAdd > 0 && voxelCount >= prevActiveVoxels) {
                /* if (!firstInst && voxelCount >= 500000) {
                     std::cout << "Adding Sphere" << std::endl;
                 }*/
                numAdds++;
                //auto voxelBody = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);
                auto voxelBody = chrono_types::make_shared<ChBodyEasyBox>(r, r, r, 1000, true, false);
                voxelBody->SetPos({voxelPos.x(), voxelPos.y(), voxelPos.z()});
                voxelBody->SetFixed(true);
                // sys.Add(voxelBody);
                // int index = voxelBody->GetIndex();
                // int index = sys.GetOtherPhysicsItems().size();
                // int index = scene->GetSprites().size();
                int index = voxelBodyList.size();
                // scene->AddSprite(voxelBody);
                // sys.AddOtherPhysicsItem(voxelBody);
                voxelBodyList.push_back(voxelBody);
                {
                    auto shape = voxelBody->GetVisualModel()->GetShapes()[0].first;
                    if (shape->GetNumMaterials() == 0) {
                        shape->AddMaterial(vis_mat);
                    } else {
                        shape->GetMaterials()[0] = vis_mat;
                    }
                }

                /* if (voxelCount > 500000)
                     std::cout << "BodyList: "<< sys.GetBodies().size() <<  " Adding Sphere " << sphere->GetIndex() <<
                   std::endl;*/
                idList.emplace_back(index);
            }
            voxelCount++;
        }
    }
    prevActiveVoxels = voxelCount;
    std::wcout << "Num Voxels: " << voxelCount << std::endl;
    scene->SetSprites(voxelBodyList);
    firstInst = false;
}

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

    // Fluid-Solid Coupling at the walls via BCE particles
    sysFSI.AddBoxContainerBCE(box, ChFrame<>(), ChVector3d(bxDim, byDim, 2 * bzDim), ChVector3i(2, 0, -1));

    driver = chrono_types::make_shared<ViperSpeedDriver>(0.1, wheel_AngVel);
    rover = chrono_types::make_shared<Viper>(&sysMBS, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    

    rover->Initialize(ChFrame<>(init_loc, QUNIT));

  
    // // Create the wheel's BCE particles
    // auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    // double scale_ratio = 1.0;
    // trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    // trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    // trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    // std::vector<ChVector3d> BCE_wheel;
    // sysFSI.CreateMeshPoints(*trimesh, iniSpacing, BCE_wheel);

    // Set the rover mass to a user mass
    for (int i = 0; i < 17; i++) {
        double mass_scale = total_mass / 440.0;
        auto viper_part = sysMBS.GetBodies()[i + 1];
        double part_mass = viper_part->GetMass();
        ChVector3d part_inertia = viper_part->GetInertiaXX();
        viper_part->SetMass(part_mass * mass_scale);
        viper_part->SetInertiaXX(part_inertia * mass_scale);
        
    }

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
        // if (i == 0 || i == 2) {
        //     sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, Q_from_AngZ(CH_PI)), true);
        // } else {
        //     sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, QUNIT), true);
        // }
        double inner_radius = wheel_radius - grouser_height;
        sysFSI.AddWheelBCE_Grouser(wheel_body, ChFrame<>(), inner_radius, wheel_wide - iniSpacing, grouser_height,
                                   grouser_wide, grouser_num, kernelLength, false);

        //wheel_body->GetCollisionModel()->AddCylinder();
    }

    {
        // Create the chassis of the test rig
        auto chassis = chrono_types::make_shared<ChBody>();
        chassis->SetMass(10.0);
        chassis->SetPos(init_loc);
        chassis->EnableCollision(false);
        chassis->SetFixed(false);

        // Add geometry of the chassis.
        //chassis->GetCollisionModel()->Clear();
        chrono::utils::AddBoxGeometry(chassis.get(), CustomWheelMaterial(ChContactMethod::NSC),
                                      ChVector3d(0.1, 0.1, 0.1), ChVector3d(0, 0, 0), ChQuaternion<>(1,0,0,0), false);
        //chassis->GetCollisionModel()->BuildModel();
        
        sysMBS.AddBody(chassis);

        // // Create the axle
        // auto axle = chrono_types::make_shared<ChBody>();
        // axle->SetMass(10.0);
        // axle->SetPos(init_loc + ChVector3d(0, 0, 1));
        // axle->SetCollide(false);
        // axle->SetBodyFixed(false);

        // // Add geometry of the axle.
        // axle->GetCollisionModel()->ClearModel();
        // chrono::utils::AddSphereGeometry(axle.get(), CustomWheelMaterial(ChContactMethod::NSC), 0.5, ChVector3d(0, 0,
        // 0)); axle->GetCollisionModel()->BuildModel(); sysMBS.AddBody(axle);

        // Connect the chassis to the containing bin (ground) through a translational joint and create a linear
        // actuator.
        auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
        prismatic1->Initialize(box, chassis, ChFrame<>(chassis->GetPos(), QuatFromAngleY(CH_PI_2)));
        prismatic1->SetName("prismatic_chassis_ground");
        sysMBS.AddLink(prismatic1);

        // auto actuator_fun = chrono_types::make_shared<ChFunction_Ramp>(0.0, wheel_vel);
        // actuator->Initialize(box, chassis, false, ChCoordsys<>(chassis->GetPos(), QUNIT),
        //                     ChCoordsys<>(chassis->GetPos() + ChVector3d(1, 0, 0), QUNIT));
        // actuator->SetName("actuator");
        // actuator->SetDistanceOffset(1);
        // actuator->SetActuatorFunction(actuator_fun);
        //sysMBS.AddLink(actuator);

        // Connect the axle to the chassis through a vertical translational joint.
        auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
        auto rover_body = rover->GetChassis()->GetBody();
        prismatic2->Initialize(chassis, rover_body, ChFrame<>(chassis->GetPos(), QUNIT));
        prismatic2->SetName("prismatic_rover_chassis");
        sysMBS.AddLink(prismatic2);

        // // Connect the rover body to the axle through a engine joint.
        // auto lock_link = chrono_types::make_shared<ChLinkLockLock>();
        // auto rover_body = rover->GetChassis()->GetBody();
        // lock_link->SetName("rover_axle_lock");
        // lock_link->Initialize(axle, rover_body, ChCoordsys<>(chassis->GetPos(), QUNIT));
        //sysMBS.AddLink(lock_link);
        for (auto body : sysMBS.GetBodies()) {
            if (body->GetVisualModel()) {
                for (auto& shape_instance : body->GetVisualModel()->GetShapes()) {
                    const auto& shape = shape_instance.first;
                    shape->SetVisible(true);
                }
            }
        }
        // Add rock obstacle
        {
            auto vis_mat2 = chrono_types::make_shared<ChVisualMaterial>();
            vis_mat2->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
            vis_mat2->SetDiffuseColor({1, 1, 1});
            vis_mat2->SetSpecularColor({1, 1, 1});
            vis_mat2->SetUseSpecularWorkflow(true);
            vis_mat2->SetRoughness(1.0f);
            vis_mat2->SetUseHapke(false);

            std::string rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock3.obj");
            ChVector3d rock_pos = ChVector3d(-.4,-.625, 0.5);

             std::shared_ptr<ChContactMaterial> rockSufaceMaterial = ChContactMaterial::DefaultMaterial(sysMBS.GetContactMethod());
            
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


            //rock_Body->GetCollisionModel()->Clear();
            auto rock1_ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rockSufaceMaterial, rock_mmesh, false, false, 0.005);
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
