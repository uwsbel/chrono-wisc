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
const std::string sensor_out_dir = "SENSOR_OUTPUT/ViperCRMTerrain/";

bool use_gi = false;


// VDB info
bool firstInst = true;
std::vector<int> idList;
int prevActiveVoxels = 0;
std::vector<std::shared_ptr<ChBody>> voxelBodyList = {};

// forward declarations
void createVoxelGrid(std::vector<openvdb::Vec3R>& points, ChSystemNSC& sys, std::shared_ptr<ChScene> scene);
std::vector<openvdb::Vec3R> floatToVec3R(float* floatBuffer, int npts);

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
    CRMTerrain terrain(sys, 0.03);
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
    double wheel_AngVel = 0.8;

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
    vis_mat->SetDiffuseColor({0.29f, 0.29f, 0.235f});
    vis_mat->SetSpecularColor({1, 1, 1});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(1.0f);
    vis_mat->SetBSDF(0);
    vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(20000);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sys.Add(floor);

    // Create a Sensor manager
    float intensity = 1.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({0, -5, 5}, {intensity, intensity, intensity}, 500);
    manager->scene->SetAmbientLight({.1, .1, .1});
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/starmap_2020_4k.hdr");
    manager->scene->SetBackground(b);
    manager->SetVerbose(false);
    manager->SetVerbose(false);

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
    float* h_points;
    int n_pts;
    while (t < tend) {
        rover->Update();

        // Run-time visualization
        if (visualization && frame % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }
        if (!visualization) {
            std::cout << sysFSI.GetSimTime() << "  " << sysFSI.GetRTF() << std::endl;
        }
       
        if(frame % sensor_render_steps == 0) {
            //std::cout << "Adding VDB grid" << std::endl;
            h_points = sysFSI.GetParticleData();
            std::cout << "Adding VDB grid" << std::endl;
            n_pts = sysFSI.GetNumFluidMarkers();
           
            vdbBuffer = floatToVec3R(h_points, n_pts);
            createVoxelGrid(vdbBuffer, sys, manager->scene);
        }
      
        manager->Update();
        sysFSI.DoStepDynamics_FSI();
        t += step_size;
        frame++;
    }

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
    float r = 0.01f;
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
    vis_mat->SetBSDF(1);
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
                auto voxelBody = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);
                // auto voxelBody = chrono_types::make_shared<ChBodyEasyBox>(r, r, r, 1000, true, false);
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