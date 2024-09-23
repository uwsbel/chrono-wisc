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
// Viper rover on CRM terrain (initialized from heightmap image)
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/robot/viper/Viper.h"

#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif


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

#include <omp.h>
#include <execution>
#include <random>


using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::viper;
using namespace chrono::vehicle;
using namespace chrono::sensor;

using std::cout;
using std::cin;
using std::endl;

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;


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
unsigned int image_width = 1920;
unsigned int image_height = 780;
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
const std::string sensor_out_dir = "SENSOR_OUTPUT/ViperCRMTerrain/";

bool use_gi = false;


// VDB info
bool firstInst = true;
std::vector<int> idList;
int prevActiveVoxels = 0;
std::vector<std::shared_ptr<ChBody>> voxelBodyList = {};
std::vector<float> offsetXList = {};
std::vector<float> offsetYList = {};
int activeVoxels = 0;

int num_meshes = 100;
std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>> regolith_meshes;  // ChVisualShapeTriangleMesh 

bool use_regolith_mesh = true;

// forward declarations
void createVoxelGrid(std::vector<float> points, ChSystemNSC& sys, std::shared_ptr<ChScene> scene);


template <typename ValueType>
class FloatBufferAttrVector {
  public:
    using PosType = ValueType;
    using value_type = openvdb::Vec3d;
    FloatBufferAttrVector(const std::vector<float> data, const openvdb::Index stride = 1)
        : mData(data), mStride(stride) {}

    size_t size() const { return mData.size() / 6; }
    void getPos(size_t n, ValueType& xyz) const {
        /*xyz[0] = mData[6 * n];
        xyz[1] = mData[6 * n + 1];
        xyz[2] = mData[6 * n + 2];*/
        xyz = ValueType(mData[6 * n], mData[6 * n + 1], mData[6 * n + 2]);
    }
    // void get(ValueType& value, size_t n) const { value = mData[n]; }
    // void get(ValueType& value, size_t n, openvdb::Index m) const { value = mData[n * mStride + m]; }

  private:
    const const std::vector<float> mData;
    const openvdb::Index mStride;
};  // PointAttributeVector

float spacing = .01f;
int main(int argc, char* argv[]) {
    double density = 1700;
    double cohesion = 5e3;
    double friction = 0.7;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    double tend = 30;
    double step_size = 5e-4;
    ChVector3d active_box_hdim(0.4, 0.3, 0.5);

    bool visualization = false;             // run-time visualization
    double visualizationFPS = 0;           // frames rendered per second (0: every frame)
    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = false;  // render boundary BCE markers
    bool visualization_rigid_bce = false;  // render wheel BCE markers

    bool verbose = true;

    // Create the Chrono system and associated collision system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the CRM terrain system
    CRMTerrain terrain(sys, spacing);
    terrain.SetVerbose(verbose);
    ChSystemFsi& sysFSI = terrain.GetSystemFSI();

    // Set SPH parameters and soil material properties
    const ChVector3d gravity(0, 0, -9.81);
    sysFSI.SetGravitationalAcceleration(gravity);
    sys.SetGravitationalAcceleration(gravity);

    ChSystemFsi::ElasticMaterialProperties mat_props;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.stress = 0;  // default
    mat_props.viscosity_alpha = 0.5;
    mat_props.viscosity_beta = 0.0;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.friction_angle = CH_PI / 10;  // default
    mat_props.dilation_angle = CH_PI / 10;  // default
    mat_props.cohesion_coeff = 0;           // default
    mat_props.kernel_threshold = 0.8;

    sysFSI.SetElasticSPH(mat_props);
    sysFSI.SetDensity(density);
    sysFSI.SetCohesionForce(cohesion);

    sysFSI.SetActiveDomain(ChVector3d(active_box_hdim));
    sysFSI.SetDiscreType(false, false);
    sysFSI.SetWallBC(BceVersion::ADAMI);
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);
    sysFSI.SetStepSize(step_size);

    sysFSI.SetOutputLength(0);

    // Set the simulation domain size
    //// TODO: needed?
    ////sysFSI.SetContainerDim(ChVector3d(4, 2, 0.1));

    cout << "Create terrain..." << endl;
    terrain.Construct(vehicle::GetDataFile("terrain/height_maps/terrain3.bmp"),  // height map image file
                      4, 4,                                                      // length (X) and width (Y)
                      {0, 2.55},                                                 // height range
                      0.3,                                                       // depth
                      3,                                                         // number of BCE layers
                      ChVector3d(0, 0, 0),                                       // patch center
                      0.0,                                                       // patch yaw rotation
                      false                                                      // side walls?
    );

    // Create rover
    cout << "Create rover..." << endl;
    ViperWheelType wheel_type = ViperWheelType::RealWheel;
    std::string wheel_obj = "robot/viper/obj/viper_wheel.obj";
    ChContactMaterialData wheel_mat(0.4f,   // mu
                                    0.2f,   // cr
                                    2e7f,   // Y
                                    0.3f,   // nu
                                    2e5f,   // kn
                                    40.0f,  // gn
                                    2e5f,   // kt
                                    20.0f   // gt
    );
    ChVector3d init_loc(-1, 0.0, 2.4);

    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    auto rover = chrono_types::make_shared<Viper>(&sys, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(wheel_mat.CreateMaterial(sys.GetContactMethod()));
    rover->Initialize(ChFrame<>(init_loc, QUNIT));

    // Create the wheel BCE markers
    cout << "Create wheel BCE markers..." << endl;
    //auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    //double scale_ratio = 1.0;
    //trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    //trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    //trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    //std::vector<ChVector3d> BCE_wheel;
    //sysFSI.CreateMeshPoints(*trimesh, sysFSI.GetInitialSpacing(), BCE_wheel);

    //// Add BCE particles and mesh of wheels to the system
    //for (int i = 0; i < 4; i++) {
    //    auto wheel_body = rover->GetWheels()[i]->GetBody();
    //    auto yaw = (i % 2 == 0) ? QuatFromAngleZ(CH_PI) : QUNIT;
    //    sysFSI.AddFsiBody(wheel_body);
    //    sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, yaw), true);
    //}

    // wheel specifics
    double wheel_radius = 0.25;
    double wheel_wide = 0.2;
    double grouser_height = 0.025;
    double grouser_wide = 0.005;
    int grouser_num = 24;
    double wheel_AngVel = 0.0;

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
        sysFSI.AddWheelBCE_Grouser(wheel_body, ChFrame<>(), inner_radius, wheel_wide - sysFSI.GetInitialSpacing(), grouser_height,
                                   grouser_wide, grouser_num, sysFSI.GetKernelLength(), false);

        // wheel_body->GetCollisionModel()->AddCylinder();
    }

    // Initialize the terrain system
    cout << "Initialize CRM terrain..." << endl;
    terrain.Initialize();

    auto aabb = terrain.GetBoundingBox();
    cout << "  SPH particles:     " << sysFSI.GetNumFluidMarkers() << endl;
    cout << "  Bndry BCE markers: " << sysFSI.GetNumBoundaryMarkers() << endl;
    cout << "  AABB:              " << aabb.min << "   " << aabb.max << endl;

    // Set up sensor sim
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
    vis_mat->SetDiffuseColor({.5,.5,5}); //0.29f, 0.29f, 0.235f
    vis_mat->SetSpecularColor({1, 1, 1});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(1.0f);
    vis_mat->SetAbsorptionCoefficient(0.001f);
    vis_mat->SetScatteringCoefficient(0.01f);
    vis_mat->SetBSDF((unsigned int)BSDFType::VDBVOL);
    vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sys.Add(floor);

    auto box = chrono_types::make_shared<ChBodyEasyBox>(8, 8, 2, 1000, true, false);
    box->SetPos({0, 0, 0});
    box->SetFixed(true);
    //sys.Add(box);
    box->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(vis_mat);

    auto vol_bbox = chrono_types::make_shared<ChNVDBVolume>(8, 8, 2, 1000, true);
    vol_bbox->SetPos({0, 0, 0});
    vol_bbox->SetFixed(true);
    sys.Add(vol_bbox);
    {
        auto shape = vol_bbox->GetVisualModel()->GetShapeInstances()[0].first;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat);
        } else {
            shape->GetMaterials()[0] = vis_mat;
        }
    }

    // Load regolith meshes
    std::string mesh_name_prefix = "sensor/geometries/regolith/particle_";
    for (int i = 1; i <= num_meshes; i++) {
         auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(mesh_name_prefix + std::to_string(i) + ".obj"),
                                                         false, true);
         mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
         auto trimesh_shape = std::make_shared<ChVisualShapeTriangleMesh>();
         trimesh_shape->SetMesh(mmesh);
         // std::cout << "OK1" << std::endl;
         trimesh_shape->SetName("RegolithMesh" + std::to_string(i));
         trimesh_shape->SetMutable(false);
         regolith_meshes.push_back(trimesh_shape);
    }

    // Create a Sensor manager
    float intensity = 1;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({0, -5, 5}, {intensity, intensity, intensity}, 500);
    manager->scene->SetAmbientLight({.1, .1, .1});
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/starmap_2020_4k.hdr");
    manager->scene->SetBackground(b);
    manager->SetVerbose(false);
    manager->scene->SetVoxelSize(spacing);
    manager->scene->SetWindowSize(5);
    manager->SetRayRecursions(4);
    manager->scene->SetThresholdVelocity(.1f);
    manager->scene->SetZThreshold(1.f);
    Integrator integrator = Integrator::LEGACY;
    bool use_denoiser = false;

    chrono::ChFrame<double> offset_pose1({0,4,3}, QuatFromAngleAxis(-CH_PI_2, {0, 0, 1}));  // Q_from_AngAxis(CH_PI_4, {0, 1, 0})  //-1200, -252, 100
    auto cam = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose1,  // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // super sampling factor
                                                         lens_model,    // lens model type
                                                         use_denoiser,
                                                         integrator,
                                                         2.2);
    cam->SetName("Third Person Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Third Person Camera"));

    if (save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "CRM_DEMO_THIRD_PERSON_VIEW_RealSlope/"));
    manager->AddSensor(cam);

    chrono::ChFrame<double> offset_pose2({0.f, -1.7, 0.5}, QuatFromAngleAxis(.2, {-2, 3, 9.75}));
    auto cam2 = chrono_types::make_shared<ChCameraSensor>(rover->GetChassis()->GetBody(),  // body camera is attached to
                                                          update_rate,                     // update rate in Hz
                                                          offset_pose2,                    // offset pose
                                                          image_width,                     // image width
                                                          image_height,                    // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // super sampling factor
                                                          lens_model,    // lens model type
                                                          use_denoiser, 
                                                          integrator,
                                                          2.2);
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
                                                          use_denoiser, 
                                                          integrator,
                                                          2.2);
    cam3->SetName("Rear Camera");
    cam3->SetLag(lag);
    cam3->SetCollectionWindow(exposure_time);
    if (vis)
        cam3->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Rear Camera"));

    if (save)
        cam3->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Rear_Cam_RealSlope/"));
    //manager->AddSensor(cam3);

    // chrono::ChFrame<double> offset_pose4({-2.f, 0, .5f}, Q_from_AngAxis(.2, {0, 1, 0}));
    chrono::ChFrame<double> offset_pose4({2.5f, 0, 1.f}, QuatFromAngleAxis(CH_PI_2, {0, 1, 0}));
    auto cam4 = chrono_types::make_shared<ChCameraSensor>(rover->GetChassis()->GetBody(),  // body camera is attached to
                                                          update_rate,                     // update rate in Hz
                                                          offset_pose4,                    // offset pose
                                                          image_width,                     // image width
                                                          image_height,                    // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // super sampling factor
                                                          lens_model,    // lens model type
                                                          use_denoiser, 
                                                          integrator,
                                                          2.2);
    cam4->SetName("Top Camera");
    cam4->SetLag(lag);
    cam4->SetCollectionWindow(exposure_time);
    if (vis)
        cam4->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Top Camera"));

    if (save)
        cam4->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Top_Cam_RealSensor/"));
    manager->AddSensor(cam4);

    // Create run-time visualization
#ifndef CHRONO_OPENGL
    if (vis_type == ChVisualSystem::Type::OpenGL)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::OpenGL;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (visualization) {
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI, verbose);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI, verbose);
#endif
                break;
            }
        }

        visFSI->SetTitle("Viper rover on CRM deformable terrain");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(init_loc + ChVector3d(0, 6, 0.5), init_loc);
        visFSI->SetCameraMoveScale(0.2f);
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(
            chrono_types::make_shared<HeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f), aabb.min.z(), aabb.max.z()));
        visFSI->AttachSystem(&sys);
        visFSI->Initialize();
    }

    // Start the simulation
    int render_steps = (visualizationFPS > 0) ? (int)std::round((1.0 / visualizationFPS) / step_size) : 1;
    unsigned int sensor_render_steps = (unsigned int)round(1 / (update_rate * step_size));
    double t = 0;
    int frame = 0;

    std::vector<openvdb::Vec3R> vdbBuffer;
    std::vector<float> h_points;
    int n_pts;
    while (t < tend) {
        rover->Update();

        // Run-time visualization
        if (visualization && frame % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }
       /* if (!visualization) {
            std::cout << sysFSI.GetSimTime() << "  " << sysFSI.GetRTF() << std::endl;
        }*/
       
        if(frame % sensor_render_steps == 0) {
            //std::cout << "Adding VDB grid" << std::endl;
            h_points = sysFSI.GetParticleData();
            std::cout << "\n##### Adding " << h_points.size()/6 << " to VDB Grid####\n" << std::endl;
            //n_pts = sysFSI.GetNumFluidMarkers();
            //manager->scene->SetFSIParticles(h_points.data());
            //manager->scene->SetFSINumFSIParticles(h_points.size()/6);
         
            createVoxelGrid(h_points, sys, manager->scene);
        }
      
        manager->Update();
        sysFSI.DoStepDynamics_FSI();
        t += step_size;
        frame++;
    }

    return 0;
}


void createVoxelGrid(std::vector<float> points, ChSystemNSC& sys, std::shared_ptr<ChScene> scene) {
    std::cout << "Creating OpenVDB Voxel Grid" << std::endl;
    openvdb::initialize();
    // openvdb::points::PointAttributeVector<openvdb::Vec3R> positionsWrapper(points);
    const FloatBufferAttrVector<openvdb::Vec3R> positionsWrapper(points);

    float r = spacing/2;
    int pointsPerVoxel = 2;
    // std::vector<float> radius(points.size(), r);
    float voxelSize = spacing ;//openvdb::points::computeVoxelSize(positionsWrapper, pointsPerVoxel);
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
    vis_mat->SetDiffuseColor({1,1,1});  // 0.29f, 0.29f, 0.235f
    vis_mat->SetSpecularColor({1, 1, 1});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(1.0f);
    vis_mat->SetBSDF((unsigned int)BSDFType::HAPKE);
    vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(20000);

    //int voxelCount = 0;
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
                voxelCount++;
            }
        }

        //for (int i = 0; i < coords.size(); i++) {
        //    const openvdb::Coord coord = coords[i];
        //        openvdb::Vec3d voxelPos = grid->indexToWorld(coord);
        //        if (!idList.empty() && ((i < prevActiveVoxels) || (i < idList.size()))) {
        //            numUpdates++;
        //            auto voxelBody = voxelBodyList[idList[i]];
        //            float offsetX = offsetXList[idList[i]];
        //            float offsetY = offsetYList[idList[i]];
        //            voxelBody->SetPos({voxelPos.x() + offsetX, voxelPos.y() + offsetY, voxelPos.z()});
        //        }
        //        //voxelCount++;
        //}    
    
    } else {
        voxelBodyList.resize(coords.size());
        idList.resize(coords.size());
        offsetXList.resize(coords.size());
        offsetYList.resize(coords.size());

        //std::atomic<int> voxelCount(0);  // Thread-safe counter for the voxels

        // Use std::for_each with parallel execution
        std::for_each(std::execution::par, coords.begin(), coords.end(), [&](const openvdb::Coord& coord) {
            // Calculate the index based on the position in the loop
            int i = &coord - &coords[0];  // Get the current index

            thread_local std::mt19937 generator(std::random_device{}());
            std::uniform_int_distribution<int> distribution(0, num_meshes-1);
            std::uniform_real_distribution<float> randpos(-.01f, .01f);
            std::uniform_real_distribution<float> randscale(1.f, 1.5);
            // Compute voxel position in world space
            openvdb::Vec3d voxelPos = grid->indexToWorld(coord);

            // Create voxelBody if necessary
            if (numVoxelsToAdd > 0 && i >= prevActiveVoxels) {
                std::shared_ptr<ChBody> voxelBody;
                if (true) {
                    //std::cout << "Adding Mesh " << i << std::endl;
                    int meshIndex = distribution(generator);  // Randomly select a mesh (if needed)
                    auto trimesh_shape = regolith_meshes[meshIndex];
                    trimesh_shape->SetScale(randscale(generator));
                    ////std::cout << "OK" << std::endl;
                    //auto trimesh_shape = std::make_shared<ChVisualShapeTriangleMesh>();
                    //trimesh_shape->SetMesh(mmesh);
                    ////std::cout << "OK1" << std::endl;
                    //trimesh_shape->SetName("RegolithMesh");
                    //trimesh_shape->SetMutable(false);
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

                float offsetX = 0.f;//randpos(generator);
                float offsetY = 0.f;//randpos(generator);
;                // Set the position and other properties of the voxel body
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
