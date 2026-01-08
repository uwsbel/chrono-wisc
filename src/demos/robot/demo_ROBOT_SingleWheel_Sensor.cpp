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
// Author: Wei Hu, Radu Serban, Nevindu Batagoda
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChMassProperties.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/core/ChTimer.h"

#include "chrono_fsi/ChSystemFsi.h"

#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/physics/ChBodyEasy.h"
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
using namespace chrono::sensor;

// Physical properties of terrain particles
double iniSpacing = 0.01;
double kernelLength = 0.01;
double density = 1700.0;

// Dimension of the terrain container
double smalldis = 1.0e-9;
double bxDim = 5.0 + smalldis;
double byDim = 0.8 + smalldis;
double bzDim = 0.12 + smalldis;

// Size of the wheel
double wheel_radius = 0.47;
double wheel_slip = 0.0;
double wheel_AngVel = 1.0;
double total_mass = 105.22;
std::string wheel_obj = "vehicle/hmmwv/hmmwv_tire_coarse_closed.obj";

// Initial Position of wheel
ChVector3d wheel_IniPos(-bxDim / 2 + wheel_radius, 0.0, wheel_radius + bzDim + iniSpacing);
ChVector3d wheel_IniVel(0.0, 0.0, 0.0);

// Simulation time and stepsize
double total_time = 5.0;
double dT = 2.5e-4;

// linear actuator and angular actuator
auto actuator = chrono_types::make_shared<ChLinkLockLinActuator>();
auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();

// Save data as csv files to see the results off-line using Paraview
bool output = false;
int output_fps = 20;

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_Single_Wheel_Test/";

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = false;
float render_fps = 100;

// Verbose terminal output
bool verbose_fsi = true;
bool verbose = true;

// Pointer to wheel
std::shared_ptr<ChBody> chassis;


//------------------------------------------------------------------
// Function to save wheel to Paraview VTK files
//------------------------------------------------------------------
void WriteWheelVTK(const std::string& filename, ChTriangleMeshConnected& mesh, const ChFrame<>& frame) {
    std::ofstream outf;
    outf.open(filename);
    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "VTK from simulation" << std::endl;
    outf << "ASCII" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID" << std::endl;
    outf << "POINTS " << mesh.GetCoordsVertices().size() << " "
         << "float" << std::endl;
    for (auto& v : mesh.GetCoordsVertices()) {
        auto w = frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }
    auto nf = mesh.GetIndicesVertexes().size();
    outf << "CELLS " << nf << " " << 4 * nf << std::endl;
    for (auto& f : mesh.GetIndicesVertexes()) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }
    outf << "CELL_TYPES " << nf << std::endl;
    for (int i = 0; i < nf; i++) {
        outf << "5 " << std::endl;
    }
    outf.close();
}


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
const std::string sensor_out_dir = "SENSOR_OUTPUT/SingleWheelTest";

bool use_gi = false;


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
    void getPos(size_t n, ValueType& xyz) const { xyz = ValueType(mData[6 * n], mData[6 * n + 1], mData[6 * n + 2]); }
    // void get(ValueType& value, size_t n) const { value = mData[n]; }
    // void get(ValueType& value, size_t n, openvdb::Index m) const { value = mData[n * mStride + m]; }

  private:
    const const std::vector<float> mData;
    const openvdb::Index mStride;
};  // PointAttributeVector
#else
void createVoxelGrid(std::vector<float> points,
                     std::shared_ptr<ChScene> scene,
                     std::shared_ptr<ChVisualMaterial> vis_mat);
#endif
int num_meshes = 100;
std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>> regolith_meshes;

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    // Common contact material
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.9f);
    cmaterial->SetRestitution(0.4f);
    cmaterial->SetAdhesion(0);

    // Create a container -- always FIRST body in the system
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    chrono::utils::AddBoxContainer(ground, cmaterial,                              //
                                   ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,           //
                                   ChVector3i(0, 0, -1),                           //
                                   false);
    ground->EnableCollision(true);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddBoxContainerBCE(ground,                                     //
                              ChFrame<>(ChVector3d(0, 0, bzDim), QUNIT),  //
                              ChVector3d(bxDim, byDim, 2 * bzDim),        //
                              ChVector3i(2, 0, -1));

    // Create the wheel -- always SECOND body in the system
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    // Compute mass inertia from mesh
    double mmass;
    double mdensity = density;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);
    mcog = ChVector3d(0.0, 0.0, 0.0);

    // Set the abs orientation, position and velocity
    auto wheel = chrono_types::make_shared<ChBodyAuxRef>();
    ChQuaternion<> wheel_Rot = QUNIT;

    // Set the COG coordinates to barycenter, without displacing the REF reference.
    // Make the COG frame a principal frame.
    wheel->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    wheel->SetMass(total_mass * 1.0 / 2.0);
    wheel->SetInertiaXX(mdensity * principal_I);
    wheel->SetPosDt(wheel_IniVel);
    wheel->SetAngVelLocal(ChVector3d(0.0, 0.0, 0.0));  // set an initial anular velocity (rad/s)

    // Set the absolute position of the body:
    wheel->SetFrameRefToAbs(ChFrame<>(ChVector3d(wheel_IniPos), ChQuaternion<>(wheel_Rot)));
    sysMBS.AddBody(wheel);

    wheel->SetFixed(false);
    auto wheel_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(cmaterial, trimesh, false, false, 0.005);
    wheel->AddCollisionShape(wheel_shape);
    wheel->EnableCollision(false);

    auto wheel_vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    wheel_vis_shape->SetMesh(trimesh);
    wheel_vis_shape->SetName("Wheel Mesh");
    wheel_vis_shape->SetMutable(false);
    auto seg_mat = chrono_types::make_shared<ChVisualMaterial>();
    seg_mat->SetClassID(30000);
    seg_mat->SetInstanceID(50000);
    wheel_vis_shape->AddMaterial(seg_mat);
    wheel->AddVisualShape(wheel_vis_shape, ChFrame<>(ChVector3d(0, 0, 0)));

    // Add this body to the FSI system
    std::vector<ChVector3d> BCE_wheel;
    sysFSI.CreateMeshPoints(*trimesh, iniSpacing, BCE_wheel);
    sysFSI.AddPointsBCE(wheel, BCE_wheel, ChFrame<>(), true);
    sysFSI.AddFsiBody(wheel);

    // Create the chassis -- always THIRD body in the system
    chassis = chrono_types::make_shared<ChBody>();
    chassis->SetMass(total_mass * 1.0 / 2.0);
    chassis->SetPos(wheel->GetPos());
    chassis->EnableCollision(false);
    chassis->SetFixed(false);

    // Add geometry of the chassis.
    chrono::utils::AddBoxGeometry(chassis.get(), cmaterial, ChVector3d(0.2, 0.2, 0.2), ChVector3d(0, 0, 0),
                                  ChQuaterniond(1, 0, 0, 0), false);
    sysMBS.AddBody(chassis);

    // Create the axle -- always FOURTH body in the system
    auto axle = chrono_types::make_shared<ChBody>();
    axle->SetMass(total_mass * 1.0 / 2.0);
    axle->SetPos(wheel->GetPos());
    axle->EnableCollision(false);
    axle->SetFixed(false);

    // Add geometry of the axle.
    chrono::utils::AddSphereGeometry(axle.get(), cmaterial, 0.5, ChVector3d(0, 0, 0), ChQuaterniond(1, 0, 0, 0), false);
    sysMBS.AddBody(axle);

    // Connect the chassis to the containing bin (ground) through a translational joint and create a linear actuator.
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, chassis, ChFrame<>(chassis->GetPos(), QuatFromAngleY(CH_PI_2)));
    prismatic1->SetName("prismatic_chassis_ground");
    sysMBS.AddLink(prismatic1);

    double velocity = wheel_AngVel * wheel_radius * (1.0 - wheel_slip);
    auto actuator_fun = chrono_types::make_shared<ChFunctionRamp>(0.0, velocity);

    actuator->Initialize(ground, chassis, false, ChFrame<>(chassis->GetPos(), QUNIT),
                         ChFrame<>(chassis->GetPos() + ChVector3d(1, 0, 0), QUNIT));
    actuator->SetName("actuator");
    actuator->SetDistanceOffset(1);
    actuator->SetActuatorFunction(actuator_fun);
    sysMBS.AddLink(actuator);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(chassis, axle, ChFrame<>(chassis->GetPos(), QUNIT));
    prismatic2->SetName("prismatic_axle_chassis");
    sysMBS.AddLink(prismatic2);

    // Connect the wheel to the axle through a engine joint.
    motor->SetName("engine_wheel_axle");
    motor->Initialize(wheel, axle, ChFrame<>(wheel->GetPos(), chrono::QuatFromAngleX(-CH_PI_2)));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, wheel_AngVel));
    sysMBS.AddLink(motor);
}

// =============================================================================

int main(int argc, char* argv[]) {
    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
        std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
        std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
        return 1;
    }

    // Create the MBS and FSI systems
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    ChVector3d gravity = ChVector3d(0, 0, -9.81);
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    sysFSI.SetVerbose(verbose_fsi);

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_SingleWheelTest.json");
    if (argc == 3) {
        inputJson = std::string(argv[1]);
        wheel_slip = std::stod(argv[2]);
    } else if (argc != 1) {
        std::cout << "usage: ./demo_FSI_SingleWheelTest <json_file> <wheel_slip>" << std::endl;
        std::cout << "or to use default input parameters ./demo_FSI_SingleWheelTest " << std::endl;
        return 1;
    }

    sysFSI.ReadParametersFromFile(inputJson);

    // Set the initial particle spacing
    sysFSI.SetInitialSpacing(iniSpacing);

    // Set the SPH kernel length
    sysFSI.SetKernelLength(kernelLength);

    // Set the terrain density
    sysFSI.SetDensity(density);

    // Set the simulation stepsize
    sysFSI.SetStepSize(dT);

    // Set the terrain container size
    sysFSI.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetConsistentDerivativeDiscretization(false, false);

    // Set cohsion of the granular material
    sysFSI.SetCohesionForce(1.0e2);

    // Setup the SPH method
    sysFSI.SetSPHMethod(SPHMethod::WCSPH);

    // Set up the periodic boundary condition (if not, set relative larger values)
    ChVector3d cMin(-bxDim / 2 * 10, -byDim / 2 - 0.5 * iniSpacing, -bzDim * 10);
    ChVector3d cMax(bxDim / 2 * 10, byDim / 2 + 0.5 * iniSpacing, bzDim * 10);
    sysFSI.SetBoundaries(cMin, cMax);

    // Initialize the SPH particles
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector3d boxCenter(0.0, 0.0, bzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2 - initSpace0, byDim / 2, bzDim / 2 - initSpace0);
    sysFSI.AddBoxSPH(boxCenter, boxHalfDim);

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(sysMBS, sysFSI);

    // Set simulation data output length
    sysFSI.SetOutputLength(0);

    // Construction of the FSI system must be finalized before running
    sysFSI.Initialize();

    auto wheel = sysMBS.GetBodies()[1];

    // Save wheel mesh
    ChTriangleMeshConnected wheel_mesh;
    wheel_mesh.LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    wheel_mesh.RepairDuplicateVertexes(1e-9);

    // Write the information into a txt file
    std::ofstream myFile;
    std::ofstream myDBP_Torque;
    if (output) {
        myFile.open(out_dir + "/results.txt", std::ios::trunc);
        myDBP_Torque.open(out_dir + "/DBP_Torque.txt", std::ios::trunc);
    }

    // Create a run-tme visualizer
#ifdef CHRONO_OPENGL
    ChFsiVisualizationGL fsi_vis(&sysFSI);
    if (render) {
        fsi_vis.SetTitle("Chrono::FSI single wheel demo");
        fsi_vis.AddCamera(ChVector3d(0, -5 * byDim, 5 * bzDim), ChVector3d(0, 0, 0));
        fsi_vis.SetCameraMoveScale(0.05f);
        fsi_vis.EnableBoundaryMarkers(true);
        fsi_vis.Initialize();
    }
#endif


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
    regolith_material->SetBSDF(BSDFType::HAPKE);
    regolith_material->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f,
                                          23.4f * (CH_PI / 180));
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

    // Add sensors

    chrono::ChFrame<double> offset_pose1({0, -1, -0.2}, QuatFromAngleAxis(CH_PI_2, {0,0,1}));
    auto wheelcam = chrono_types::make_shared<ChCameraSensor>(chassis,  // body camera is attached to
                                                          update_rate,                     // update rate in Hz
                                                          offset_pose1,                    // offset pose
                                                          image_width,                     // image width
                                                          image_height,                    // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // super sampling factor
                                                          lens_model,    // lens model type
                                                          use_denoiser);
    wheelcam->SetIntegrator(integrator);
    wheelcam->SetName("Wheel Camera");
    wheelcam->SetLag(lag);
    wheelcam->SetCollectionWindow(exposure_time);
    if (vis)
        wheelcam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Wheel Camera"));

    if (save)
        wheelcam->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "WheelCam/"));
    manager->AddSensor(wheelcam);

    
    auto seg = chrono_types::make_shared<ChSegmentationCamera>(chassis,  // body camera is attached to
                                                        update_rate,                     // update rate in Hz
                                                        offset_pose1,                    // offset pose
                                                        image_width,                     // image width
                                                        image_height,                    // image height
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

    manager->AddSensor(seg);

    chrono::ChFrame<double> offset_pose2({-1, 0, -0.2}, QuatFromAngleAxis(0, {0, 0, 1}));
    auto rear_cam = chrono_types::make_shared<ChCameraSensor>(chassis,       // body camera is attached to
                                                              update_rate,   // update rate in Hz
                                                              offset_pose2,  // offset pose
                                                              image_width,   // image width
                                                              image_height,  // image height
                                                              fov,           // camera's horizontal field of view
                                                              alias_factor,  // super sampling factor
                                                              lens_model,    // lens model type
                                                              use_denoiser);
    rear_cam->SetIntegrator(integrator);
    rear_cam->SetName("Rear Camera");
    rear_cam->SetLag(lag);
    rear_cam->SetCollectionWindow(exposure_time);
    if (vis)
        rear_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Rear Camera"));

    if (save)
        rear_cam->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "RearCam/"));
    manager->AddSensor(rear_cam);

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();
    int sensor_render_steps = (unsigned int)round(1 / (update_rate * dT));
    std::vector<float> h_points;
    while (time < total_time) {
        // Get the infomation of the wheel
        const auto& reaction = actuator->GetReaction2();
        const auto& force = reaction.force;
        const auto& torque = reaction.torque;
        const auto& w_pos = wheel->GetPos();
        const auto& w_vel = wheel->GetPosDt();
        const auto& angvel = wheel->GetAngVelLocal();

        if (verbose) {
            std::cout << "time: " << time << std::endl;
            std::cout << "  wheel position:         " << w_pos << std::endl;
            std::cout << "  wheel linear velocity:  " << w_vel << std::endl;
            std::cout << "  wheel angular velocity: " << angvel << std::endl;
            std::cout << "  drawbar pull:           " << force << std::endl;
            std::cout << "  wheel torque:           " << torque << std::endl;
        }

        if (output) {
            myFile << time << "\t" << w_pos.x() << "\t" << w_pos.y() << "\t" << w_pos.z() << "\t" << w_vel.x() << "\t"
                   << w_vel.y() << "\t" << w_vel.z() << "\t" << angvel.x() << "\t" << angvel.y() << "\t" << angvel.z()
                   << "\t" << force.x() << "\t" << force.y() << "\t" << force.z() << "\t" << torque.x() << "\t"
                   << torque.y() << "\t" << torque.z() << "\n";
            myDBP_Torque << time << "\t" << force.x() << "\t" << torque.z() << "\n";
        }

        if (output && time >= out_frame / output_fps) {
            std::cout << "-------- Output" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
            static int counter = 0;
            std::string filename = out_dir + "/vtk/wheel." + std::to_string(counter++) + ".vtk";
            WriteWheelVTK(filename, wheel_mesh, wheel->GetFrameRefToAbs());
            out_frame++;
        }

        // Render SPH particles
#ifdef CHRONO_OPENGL
        if (render && time >= render_frame / render_fps) {
            if (!fsi_vis.Render())
                break;
            render_frame++;
        }
#endif

                            
        if (sim_frame % sensor_render_steps == 0 && sim_frame > 0) {
            // timerNVDB.start();
            h_points = sysFSI.GetParticleData();
#ifdef USE_SENSOR_NVDB
            createVoxelGrid(h_points, sysMBS, manager->scene, regolith_material);
#else
            createVoxelGrid(h_points, manager->scene, regolith_material);
#endif
            manager->Update();
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();
        time += dT;
        sim_frame++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    if (output) {
        myFile.close();
        myDBP_Torque.close();
    }

    return 0;
}


#ifdef USE_SENSOR_NVDB
void createVoxelGrid(std::vector<float> points,
                     std::shared_ptr<ChScene> scene,
                     std::shared_ptr<ChVisualMaterial> vis_mat) {
    std::cout << "Creating OpenVDB Voxel Grid for " << points.size() / 6 << "particles " << std::endl;
    openvdb::initialize();
    // openvdb::points::PointAttributeVector<openvdb::Vec3R> positionsWrapper(points);
    const FloatBufferAttrVector<openvdb::Vec3R> positionsWrapper(points);
    float spacing = iniSpacing / 2.f;
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

    } else {
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
                     std::shared_ptr<ChScene> scene,
                     std::shared_ptr<ChVisualMaterial> vis_mat) {
    std::cout << "Creating CPU Voxel Grid for " << points.size() / 6 << "particles " << std::endl;
    float spacing = iniSpacing / 2.f;
    float r = spacing;
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
            ChVector3d pos(points[6 * i], points[6 * i + 1], points[6 * i + 2]);
            if (!idList.empty() && ((voxelCount < prevActiveVoxels) || (voxelCount < idList.size()))) {
                numUpdates++;
                auto voxelBody = voxelBodyList[idList[voxelCount]];
                float offsetX = offsetXList[idList[voxelCount]];
                float offsetY = offsetYList[idList[voxelCount]];
                ChVector3d voxelPos(pos.x() + offsetX, pos.y() + offsetY, pos.z());

                voxelBody->SetPos(voxelPos);
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
                ChVector3d voxelPos(pos.x() + offsetX, pos.y() + offsetY, pos.z());

                voxelBody->SetPos(voxelPos);
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
        // std::for_each(std::execution::par, points.begin(), points.begin() + activeVoxels, [&](float& point) {
        for (int i = 0; i < points.size() / 6; i++) {
            // Calculate the index based on the position in the loop
            // int i = &point - &points[0];  // Get the current index

            thread_local std::mt19937 generator(std::random_device{}());
            std::uniform_int_distribution<int> distribution(0, num_meshes - 1);
            std::uniform_real_distribution<float> randpos(-.005f, .005f);
            std::uniform_real_distribution<float> randscale(1.f, 1.5);
            // Compute voxel position in world space
            ChVector3d pos(points[6 * i], points[6 * i + 1], points[6 * i + 2]);
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

                voxelBody->SetPos(voxelPos);
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