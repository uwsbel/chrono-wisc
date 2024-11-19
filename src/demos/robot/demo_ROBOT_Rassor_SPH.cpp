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
// Author: Json Zhou, Luning Bakke
// Chrono::FSI demo to show usage of Rassor rover models on CRM granular terrain
// This demo uses a plug-in Rassor rover model from chrono::models
// TODO: 
// rover initialization and driver should be independent from create solid phase
// =============================================================================

#include "chrono_models/robot/rassor/Rassor.h"

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
#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

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


#include <execution>
#include <random>

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::rassor;
using namespace chrono::sensor;


// If true, save as Wavefront OBJ; if false, save as VTK
bool save_obj = false;

// Physical properties of terrain particles
double iniSpacing = 0.004;
double kernelLength = 0.004;
double density = 2600.0;

// Dimension of the space domain
double bxDim = 3.0; // this for real
//double bxDim = 3.0; // use this for debug
double byDim = 1.6;
double bzDim = 0.1;

// Rover initial location
ChVector3d init_loc(-bxDim / 2.0 + 1.0, 0, bzDim + 0.25);

// Simulation time and stepsize
double total_time = 10.0;
double dT = 2.0e-4;

// Save data as csv files to see the results off-line using Paraview
bool output = false;
int out_fps = 10;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 30;

// Pointer to store the Rassor instance
std::shared_ptr<Rassor> rover;
std::shared_ptr<RassorSpeedDriver> driver;

// Define Viper rover wheel type
RassorWheelType wheel_type = RassorWheelType::RealWheel;

// Use below mesh file if the wheel type is real VIPER wheel
std::string wheel_obj = "robot/rassor/obj/rassor_wheel.obj";

// Speeds from Sam
double rover_velocity_array[5] = {0.05, 0.1, 0.15, 0.2, 0.3};
double bucket_omega_array[5]   = {0.7, 1.39, 2.09, 2.78, 4.17};


std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method);
// Forward declaration of helper functions
void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI);
std::vector<ChVector3d> LoadSolidPhaseBCE(std::string filename);
bool CreateSubDirectories(std::string out_dir);


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
int alias_factor = 4;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------
// Save camera images
bool save = true;
// Render camera images
bool vis = true;
// Output directory
const std::string sensor_out_dir = "SENSOR_OUTPUT/RASSOR/";

bool use_gi = false;

// VDB info
bool firstInst = true;
std::vector<int> idList;
int prevActiveVoxels = 0;
std::vector<std::shared_ptr<ChBody>> voxelBodyList = {};
std::vector<float> offsetXList = {};
std::vector<float> offsetYList = {};
int activeVoxels = 0;

// forward declarations
void createVoxelGrid(std::vector<float> points, ChSystemNSC& sys, std::shared_ptr<ChScene> scene); //std::vector<openvdb::Vec3R>&
std::vector<openvdb::Vec3R> floatToVec3R(float* floatBuffer, int npts);


template<typename ValueType>
class FloatBufferAttrVector {
  public:
    using PosType = ValueType;
    using value_type= openvdb::Vec3d;
    FloatBufferAttrVector(const std::vector<float> data, const openvdb::Index stride = 1) : mData(data), mStride(stride) {}

    size_t size() const { return mData.size()/6; }
    void getPos(size_t n, ValueType& xyz) const {
        /*xyz[0] = mData[6 * n];
        xyz[1] = mData[6 * n + 1];
        xyz[2] = mData[6 * n + 2];*/
        xyz = ValueType(mData[6 * n], mData[6 * n + 1], mData[6 * n + 2]);
    }
    //void get(ValueType& value, size_t n) const { value = mData[n]; }
    //void get(ValueType& value, size_t n, openvdb::Index m) const { value = mData[n * mStride + m]; }

  private:
    const const std::vector<float> mData;
    const openvdb::Index mStride;
};  // PointAttributeVector



int num_meshes = 100;
std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>> regolith_meshes;  // ChVisualShapeTriangleMesh 


int main(int argc, char* argv[]) {

    // check number of command line inputs
 /*   if (argc != 4) {
        std::cout << "usage: ./demo_ROBOT_Rassor_SPH <TestID> <artificial_viscosity> <output_folder>" << std::endl;
        return 1;
    }*/

    // get the TestID from the command line
    int TestID = 4;//std::stoi(argv[1]);
    double artificial_viscosity = 0.2;//std::stod(argv[2]);
    std::string out_dir = "RASSOR_OUTPUT"; //std::string(argv[3]);


    double wheel_radius = 0.22;
    double wheel_driver_speed  = rover_velocity_array[TestID] / wheel_radius;
    double bucket_driver_speed = bucket_omega_array[TestID];

    // Output directories and settings
    out_dir = GetChronoOutputPath() + out_dir + "/";

    if (!CreateSubDirectories(out_dir)) { return 1; }

    // Create a physical system and a corresponding FSI system
    ChSystemNSC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);


    // Read JSON file with simulation parameters
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Rassor_granular_NSC.json");
    sysFSI.ReadParametersFromFile(inputJson);

    
    ChVector3d gravity = ChVector3d(0, 0, -9.81);
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    // overwrite artificial viscosity 
    //sysFSI.SetArtificialViscosity(artificial_viscosity, 0.0);

    // Set the initial particle spacing
    sysFSI.SetInitialSpacing(iniSpacing);

    std::cout << "kernelL" << kernelLength << std::endl;

    // Set the SPH kernel length
    sysFSI.SetKernelLength(kernelLength);

    // Set the terrain density
    sysFSI.SetDensity(density);

    // Set the simulation stepsize
    sysFSI.SetStepSize(dT);

    // Set the simulation domain size
    sysFSI.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetConsistentDerivativeDiscretization(false, false);

    // Setup the solver based on the input value of the prameters
    sysFSI.SetSPHMethod(SPHMethod::WCSPH);




    // Set the periodic boundary condition
    double initSpace0 = sysFSI.GetInitialSpacing();
    ChVector3d cMin(-bxDim / 2 * 2, -byDim / 2 - 0.5 * iniSpacing, -bzDim * 10);
    ChVector3d cMax( bxDim / 2 * 2,  byDim / 2 + 0.5 * iniSpacing,   bzDim * 20);
    sysFSI.SetBoundaries(cMin, cMax);

    // Set simulation data output length
    sysFSI.SetOutputLength(0);

    // Create an initial box for the terrain patch
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    ChVector3d boxCenter(0, 0, bzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    std::cout << "Generate SPH particles (" << points.size() << ")" << std::endl;
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
    std::cout << "Generate BCE markers" << std::endl;
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Get the body from the FSI system for visualization
    //std::vector<std::shared_ptr<ChBody>>& FSI_Bodies = sysFSI.GetFsiBodies();


    // Write position and velocity to file
    std::ofstream ofile;
    if (output)
        ofile.open(out_dir + "./body_position.txt");

#if !defined(CHRONO_VSG)
    render = false;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        #ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        #endif

        visFSI->SetTitle("Rassor on CRM terrain");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector3d(0, -3 * byDim, bzDim), ChVector3d(0, 0, 0));
        visFSI->SetCameraMoveScale(1.0f);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(
            chrono_types::make_shared<HeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f), 0, bzDim));
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // setup sensor sim

    // Set up sensor sim
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
    vis_mat->SetDiffuseColor({0.29f, 0.29f, 0.235f});
    vis_mat->SetSpecularColor({1, 1, 1});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(1.0f);
    vis_mat->SetBSDF((unsigned int)BSDFType::VDB);
    vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(20000);


    auto vol_bbox = chrono_types::make_shared<ChNVDBVolume>(400, 400, 200, 1000, true);
    vol_bbox->SetPos({0, 0, 0});
    vol_bbox->SetFixed(true);
    //sysMBS.Add(vol_bbox);
    {
        auto shape = vol_bbox->GetVisualModel()->GetShapeInstances()[0].first;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat);
        } else {
            shape->GetMaterials()[0] = vis_mat;
        }
    }

  /*  auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sysMBS.Add(floor);*/

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

    // Create a Sensor manager
    float intensity = 10.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sysMBS);
    manager->scene->AddPointLight({0, 5, 5}, {intensity, intensity, intensity}, 500);
    manager->scene->SetAmbientLight({0, 0,0});
    manager->scene->SetVoxelSize(sysFSI.GetInitialSpacing());
    manager->scene->SetWindowSize(5);
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/starmap_2020_4k.hdr");
    manager->scene->SetBackground(b);
    manager->SetVerbose(false);
    manager->SetRayRecursions(4);
    Integrator integrator = Integrator::LEGACY;

    chrono::ChFrame<double> offset_pose2({-0, -1.3, 0.2}, QuatFromAngleAxis(CH_PI_2, {0,0,1})); //QuatFromAngleAxis(.2, {-2, 3, 9.75})
    auto cam2 = chrono_types::make_shared<ChCameraSensor>(rover->GetChassis()->GetBody(),  // body camera is attached to
                                                          update_rate,                     // update rate in Hz
                                                          offset_pose2,                    // offset pose
                                                          image_width,                     // image width
                                                          image_height,                    // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // super sampling factor
                                                          lens_model,    // lens model type
                                                          true);
    cam2->SetIntegrator(integrator);
    cam2->SetName("Wheel Camera");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);
    if (vis)
        cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Wheel Camera"));

    if (save)
        cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Wheel_Cam_RealSlope/"));
    manager->AddSensor(cam2);

    
    chrono::ChFrame<double> offset_pose3(
        {0,0,2.f}, QuatFromAngleAxis(CH_PI_2, {0, 1, 0}));  // QuatFromAngleAxis(.2, {-2, 3, 9.75})
    auto top = chrono_types::make_shared<ChCameraSensor>(rover->GetChassis()->GetBody(),  // body camera is attached to
                                                          update_rate,                     // update rate in Hz
                                                          offset_pose3,                    // offset pose
                                                          image_width,                     // image width
                                                          image_height,                    // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // super sampling factor
                                                          lens_model,    // lens model type
                                                          true);
    top->SetIntegrator(integrator);
    top->SetName("Wheel Camera");
    top->SetLag(lag);
    top->SetCollectionWindow(exposure_time);
    if (vis)
        top->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Top Camera"));

    if (save)
        top->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Top_Camera"));
    manager->AddSensor(top);

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));
    double time = 0.0;
    int current_step = 0;

    auto body = sysMBS.GetBodies()[1];

    ChTimer timer;
    ChTimer sensorTimer;
    std::vector<float> h_points;
    int n_pts;

    int sensor_render_steps = (unsigned int)round(1 / (update_rate * dT));
    while (time < total_time) {

        //// RASSOR 2.0, drum spinning counter clock wise
        //driver->SetRazorMotorSpeed((RassorDirID)0, -bucket_driver_speed);
        //driver->SetRazorMotorSpeed((RassorDirID)1,  bucket_driver_speed);

        // RASSOR 1.0, drum spinning  clock wise
        driver->SetRazorMotorSpeed((RassorDirID)0,  bucket_driver_speed);
        driver->SetRazorMotorSpeed((RassorDirID)1, -bucket_driver_speed);


        for (int i = 0; i < 4; i++) {
            driver->SetDriveMotorSpeed((RassorWheelID)i, wheel_driver_speed/3.0);
        }


        

        //if (time <= 3.0) {
        //    for (int i = 0; i < 4; i++) {
        //        driver->SetDriveMotorSpeed((RassorWheelID)i, 0.0);
        //    }

        //    driver->SetRazorMotorSpeed((RassorDirID)0, 3.14);
        //    driver->SetRazorMotorSpeed((RassorDirID)1, -3.14);
        //    driver->SetArmMotorSpeed((RassorDirID)0, 0.05);
        //    driver->SetArmMotorSpeed((RassorDirID)1, -0.05);
       
        //} else if (time > 3.0 && time <= 5.0) {
        //    for (int i = 0; i < 4; i++) {
        //        driver->SetDriveMotorSpeed((RassorWheelID)i, 0.0);
        //    }

        //    driver->SetRazorMotorSpeed((RassorDirID)0, 0.0);
        //    driver->SetRazorMotorSpeed((RassorDirID)1, 0.0);
        //    driver->SetArmMotorSpeed((RassorDirID)0, -0.55);
        //    driver->SetArmMotorSpeed((RassorDirID)1, 0.55);
        //} else if (time > 5.0 && time <= 8.0) {
        //    for (int i = 0; i < 4; i++) {
        //        driver->SetDriveMotorSpeed((RassorWheelID)i, 0.0);
        //    }

        //    driver->SetRazorMotorSpeed((RassorDirID)0, -2.0);
        //    driver->SetRazorMotorSpeed((RassorDirID)1, 2.0);
        //    driver->SetArmMotorSpeed((RassorDirID)0, 0.0);
        //    driver->SetArmMotorSpeed((RassorDirID)1, 0.0);
        //}
// 
        if (time <= 2.0) {
          for (int i = 0; i < 4; i++) {
              driver->SetDriveMotorSpeed((RassorWheelID)i, 2.0);
          }

          driver->SetRazorMotorSpeed((RassorDirID)0, 2.0);
          driver->SetRazorMotorSpeed((RassorDirID)1, 2.0);
          driver->SetArmMotorSpeed((RassorDirID)0, 0.0);
          driver->SetArmMotorSpeed((RassorDirID)1, 0.0);
        } else if (time > 2.0 && time <= 5.0) {
          for (int i = 0; i < 4; i++) {
              driver->SetDriveMotorSpeed((RassorWheelID)i, 0.0);
          }

          driver->SetRazorMotorSpeed((RassorDirID)0, 3.14);
          driver->SetRazorMotorSpeed((RassorDirID)1, -3.14);
          driver->SetArmMotorSpeed((RassorDirID)0, 0.05);
          driver->SetArmMotorSpeed((RassorDirID)1, -0.05);
        } else if (time > 5.0 && time <= 7.0) {
          for (int i = 0; i < 4; i++) {
              driver->SetDriveMotorSpeed((RassorWheelID)i, 0.0);
          }

          driver->SetRazorMotorSpeed((RassorDirID)0, 0.0);
          driver->SetRazorMotorSpeed((RassorDirID)1, 0.0);
          driver->SetArmMotorSpeed((RassorDirID)0, -0.55);
          driver->SetArmMotorSpeed((RassorDirID)1, 0.55);
        } else if (time > 7.0 && time <= 10.0) {
          for (int i = 0; i < 4; i++) {
              driver->SetDriveMotorSpeed((RassorWheelID)i, 0.0);
          }

          driver->SetRazorMotorSpeed((RassorDirID)0, -2.0);
          driver->SetRazorMotorSpeed((RassorDirID)1, 2.0);
          driver->SetArmMotorSpeed((RassorDirID)0, 0.0);
          driver->SetArmMotorSpeed((RassorDirID)1, 0.0);
        }


       

        std::cout << current_step << "  time: " << time << "  sim. time: " << timer() << " Render Time: " << sensorTimer.GetTimeMilliseconds() <<  "ms " << std::endl;
        std::cout << "  pos: " << body->GetPos() << std::endl;
        std::cout << "  vel: " << body->GetPosDt() << std::endl;
        sensorTimer.reset();
        
        rover->Update();

        if (current_step % sensor_render_steps == 0 && current_step > 0) {
            h_points = sysFSI.GetParticleData();
            //n_pts = sysFSI.GetNumFluidMarkers();
            //manager->scene->SetFSIParticles(h_points.data());
            //manager->scene->SetFSINumFSIParticles(h_points.size()/6);
            createVoxelGrid(h_points, sysMBS, manager->scene);

            
            sensorTimer.start();
            manager->Update();
            sensorTimer.stop();
        }

       
    /*    if (output && current_step % output_steps == 0) {
               

            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
            rover->writeMeshFile(out_dir + "/rover", int(current_step/output_steps), true);
        }*/

        //// Render system
        //if (render && current_step % render_steps == 0) {
        // /*   if (!visFSI->Render())
        //        break;*/
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


void createVoxelGrid(std::vector<float> points, ChSystemNSC& sys, std::shared_ptr<ChScene> scene) {
    std::cout << "Creating OpenVDB Voxel Grid" << std::endl;
    openvdb::initialize();
    // openvdb::points::PointAttributeVector<openvdb::Vec3R> positionsWrapper(points);
    const FloatBufferAttrVector<openvdb::Vec3R> positionsWrapper(points);
    float spacing = 0.005f;
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

    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
    vis_mat->SetDiffuseColor({1, 1, 1});  // 0.29f, 0.29f, 0.235f
    vis_mat->SetSpecularColor({1, 1, 1});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(1.0f);
    vis_mat->SetBSDF((unsigned int)BSDFType::HAPKE);
    vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(20000);

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
                    voxelBody->SetPos({voxelPos.x() + offsetX, voxelPos.y() + offsetY, voxelPos.z()});
                    //voxelBody->SetPos({voxelPos.x(), voxelPos.y(), voxelPos.z()});
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
                    voxelBody->SetPos({voxelPos.x() + offsetX, voxelPos.y() + offsetY, voxelPos.z()});
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
                voxelBody->SetPos({voxelPos.x() + offsetX, voxelPos.y() + offsetY, voxelPos.z()});
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

    driver = chrono_types::make_shared<RassorSpeedDriver>(1.0);
    rover = chrono_types::make_shared<Rassor>(&sysMBS, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    rover->Initialize(ChFrame<>(init_loc, QUNIT));


    std::vector<ChVector3d> BCE_wheel;
    BCE_wheel = LoadSolidPhaseBCE(GetChronoDataFile("robot/rassor/bce/rassor_wheel.csv"));
    std::cout << "BCE wheel len:" << BCE_wheel.size() << std::endl;

    // Add BCE particles and mesh of wheels to the system
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> wheel_body;
        if (i == 0) {
            wheel_body = rover->GetWheel(RassorWheelID::RA_LF)->GetBody();
        }
        if (i == 1) {
            wheel_body = rover->GetWheel(RassorWheelID::RA_RF)->GetBody();
        }
        if (i == 2) {
            wheel_body = rover->GetWheel(RassorWheelID::RA_LB)->GetBody();
        }
        if (i == 3) {
            wheel_body = rover->GetWheel(RassorWheelID::RA_RB)->GetBody();
        }

        sysFSI.AddFsiBody(wheel_body);
        if (i == 0 || i == 2) {
            sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, QuatFromAngleZ(CH_PI)), true);
        } else {
            sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, QUNIT), true);
        }
    }

    std::vector<ChVector3d> BCE_razor_back;
    BCE_razor_back = LoadSolidPhaseBCE(GetChronoDataFile("robot/rassor/bce/rassor_drum.csv"));
    std::cout << "BCE Razor len:" << BCE_razor_back.size() << std::endl;

    // now create vector BCE_razor_front, it is the mirrored version of BCE_razor_back, where x value is flipped
    std::vector<ChVector3d> BCE_razor_front;
    for (int i = 0; i < BCE_razor_back.size(); i++) {
        BCE_razor_front.push_back(ChVector3d(-BCE_razor_back[i].x(), BCE_razor_back[i].y(), BCE_razor_back[i].z()));
    }

    // Add BCE particles and mesh of razor to the system
    for (int i = 0; i < 2; i++) {
        std::shared_ptr<ChBodyAuxRef> razor_body;
        if (i == 0) {
            razor_body = rover->GetRazor(RassorDirID::RA_F)->GetBody();
        }
        if (i == 1) {
            razor_body = rover->GetRazor(RassorDirID::RA_B)->GetBody();
        }

        sysFSI.AddFsiBody(razor_body);

        // This is the case for bucket pushing soil away (front drum spin counter clock wise)
        //if (i == 0) {
        //    sysFSI.AddPointsBCE(razor_body, BCE_razor_front, ChFrame<>(VNULL, QUNIT), true);
        //} else {
        //    sysFSI.AddPointsBCE(razor_body, BCE_razor_back, ChFrame<>(VNULL, QUNIT), true);
        //}

        // This is the case for front drum spins clockwise
        if (i == 0) {
            sysFSI.AddPointsBCE(razor_body, BCE_razor_back, ChFrame<>(VNULL, QUNIT), true);
        } else {
            sysFSI.AddPointsBCE(razor_body, BCE_razor_front, ChFrame<>(VNULL, QUNIT), true);
        }



    }
}

std::vector<ChVector3d> LoadSolidPhaseBCE(std::string filename) {
    std::ifstream file(filename);
    std::vector<ChVector3d> points;
    std::string line;

    // Check if the file is open
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return points;
    }

    // Skip the header
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<float> point;
        std::string value;

        while (std::getline(ss, value, ',')) {
            point.push_back(std::stof(value));
        }

        ChVector3d pt_vec;
        pt_vec.x() = point[0];
        pt_vec.y() = point[1];
        pt_vec.z() = point[2];

        points.push_back(pt_vec);
    }

    return points;
}

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

bool CreateSubDirectories(std::string out_dir){
        // Create oputput subdirectories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return false;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return false;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
        std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
        return false;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/rover"))) {
        std::cerr << "Error creating directory " << out_dir + "/rover" << std::endl;
        return false;
    }

    return true;
}