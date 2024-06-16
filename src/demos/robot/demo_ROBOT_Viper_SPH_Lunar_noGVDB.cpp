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

#include "chrono_vehicle/terrain/CRMTerrain.h"

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


#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/ParticlesToLevelSet.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::viper;
using namespace chrono::vehicle;
using namespace chrono::sensor;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::OpenGL;

// output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_Viper/";

// Physical properties of terrain particles
double iniSpacing = 0.01;
double kernelLength = 0.01;

std::string terrain_layer_name = GetChronoDataFile("robot/viper/csv/particle_layer_L_0.01.csv");

// if true, save as Wavefront OBJ; if false, save as VTK
bool save_obj = false;  

// Physical properties of terrain particles
double density = 1700.0;

// Dimension of the space domain
double bxDim = 20.0;
double byDim = 20.0;
double bzDim = 0.2;

// Define L shaped terrain for the rover
double rec1_xstart = -0.25 - 0.5;
double rec2_xstart = 2.15 - 0.5;
double rec2_xend = 4.25 - 0.5;

double ystart = -4.35;
double rec1_yend = -2.0;
// double rec2_yend = 9.0;
double rec2_yend = 5;

double z_min = -0.85;
double z_max =  0.25;

// Define boxed particle domain
ChVector3d sph_markers_min(-0.75, -4.35, z_min);
ChVector3d sph_markers_max( 3.75,  rec2_yend, z_max);

// Rover initial location
ChVector3d init_loc(rec1_xstart + 1.0f + 2.3f, -3.2, bzDim + 0.25f);

double wheel_radius = 0.2;
double wheel_wide = 0.15;
double grouser_height = 0.025;
double grouser_wide = 0.005;
int grouser_num = 24;
double wheel_AngVel = 0.8;

// Simulation time and stepsize
double dT = 2.5e-4;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = false;
float render_fps = 30;

bool output = true;
float out_fps = 20;

// Pointer to store the VIPER instance
std::shared_ptr<Viper> rover;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// Use below mesh file if the wheel type is real VIPER wheel
std::string wheel_obj = "robot/viper/obj/viper_wheel.obj";


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
unsigned int image_height = 1080;
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
const std::string sensor_out_dir = "SENSOR_OUTPUT/NoGVDB/";


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

std::vector<ChVector3f> importCSV(const std::string& filename) {
    std::vector<ChVector3f> data;

    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return data; // Return empty vector if file opening fails
    }

    std::string line;
    std::getline(file, line); // skip the first header line
    ChVector3f row;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::getline(ss, cell, ',');
        row.x() = std::stof(cell);
        std::getline(ss, cell, ',');
        row.y() = std::stof(cell);
        std::getline(ss, cell, ',');
        row.z() = std::stof(cell);
        data.push_back(row);
    }

    return data;
}

// customize the wall of the terrain based on the topology 
std::vector<ChVector3f> generateWallBCE(double h // spacing
                                            ){
    std::vector<ChVector3f> bce;

    // add rec A (long patch)
    for (float x = rec2_xstart - 3 * h; x < rec2_xstart; x+= h){
        for (float y = rec1_yend - h;  y < rec2_yend + 4 * h; y+= h){
            
            double z_min_approx = 0;
            // flat part of the patch
            if ( y < 0) {
                for (float z = z_max - 35*h; z < z_max ; z+= h){
                bce.push_back(ChVector3f(x,y,z));
            }
            }

            // downhill part 
            if (y >= 0 && y <= 5){
                z_min_approx = -0.2 * y;
                for (float z = z_min_approx; z < z_min_approx + 30 * h ; z+= h){
                bce.push_back(ChVector3f(x,y,z));
            }

            }



        }
    }


    // add rec E
    for (float x = rec2_xend; x < rec2_xend + 4 * h; x+= h){
        for (float y = ystart - 3 * h;  y < rec2_yend + 4 * h; y+= h){
            
            double z_min_approx = 0;
            
            // flat part of the patch
            if (y < 0) {
                for (float z = z_max - 35*h; z < z_max ; z+= h){
                bce.push_back(ChVector3f(x,y,z));
            }
            }


            // downhill part 
            if (y >= 0 && y <= 5){
                z_min_approx = -0.2 * y;
                for (float z = z_min_approx; z < z_min_approx + 30 * h ; z+= h){
                    bce.push_back(ChVector3f(x,y,z));
                }

            // Thomas TODO: need to add the rest of BCE markers of the downhill part

            }

            if (y > 5) {

            }

        }
    }
    

    // add rec C
    for (float x = rec1_xstart - 3 * h; x < rec1_xstart; x+= h){
        for (float y = ystart - 3 * h;  y < rec1_yend + 4 * h; y+= h){
            for (float z = -0.4; z < z_max; z+= h){

                bce.push_back(ChVector3f(x,y,z));

            }
        }
    }


    // add rec B
    for (float x = rec1_xstart; x < rec2_xstart - 3*h; x += h){
        for (float y = rec1_yend - h; y < rec1_yend + 4*h; y += h){
            for (float z = -0.4; z < z_max; z+= h){

                bce.push_back(ChVector3f(x,y,z));
            }

        }
    }

    ///////////////
    // add rec D//
    ///////////////
    for (float x = rec1_xstart; x < rec2_xend; x += h){
        for (float y = ystart - 3 * h; y < ystart; y += h){
            for (float z = -0.4; z < z_max; z+= h){

                bce.push_back(ChVector3f(x,y,z));
            }

        }
    }



    return bce;

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


// VDB info
bool firstInst = true;
std::vector<int> idList;
int prevActiveVoxels = 0;

// forward declarations
void createVoxelGrid(std::vector<openvdb::Vec3R>& points, ChSystemNSC& sys);
std::vector<openvdb::Vec3R> floatToVec3R(float* floatBuffer, int npts);

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
    if (!filesystem::create_directory(filesystem::path(out_dir + "/rover"))) {
        std::cerr << "Error creating directory " << out_dir + "/rover" << std::endl;
        return 1;
    }

    // Create a physical system and a corresponding FSI system
    ChSystemNSC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    // Create a ground body for the BCE particles
    auto m_ground = chrono_types::make_shared<ChBody>();  // Is this right?
    sysMBS.AddBody(m_ground);
    m_ground->SetFixed(true);

    ChVector3d gravity = ChVector3d(0, 0, -9.81);
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    // Read JSON file with simulation parameters
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Viper_granular_NSC.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc != 1) {
        std::cout << "usage: ./demo_ROBOT_Viper_SPH <json_file>" << std::endl;
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

    // Set the simulation domain size
    sysFSI.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetDiscreType(false, false);

    // Set wall boundary condition
    sysFSI.SetWallBC(BceVersion::ADAMI);

    // Setup the solver based on the input value of the prameters
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);

    // Set the periodic boundary condition
    // ChVector3d cMin(-bxDim / 2 * 2, -byDim / 2 - 0.5 * iniSpacing, -bzDim * 10);
    // ChVector3d cMax(bxDim / 2 * 2, byDim / 2  + 0.5 * iniSpacing, bzDim * 20);
    ChVector3d cMin(-15.0f, -15.0f, -3.0f);
    ChVector3d cMax(15.0f, 15.0f, 3.0f);
    sysFSI.SetBoundaries(cMin, cMax);

    // Set simulation data output length
    sysFSI.SetOutputLength(1);

    std::vector<ChVector3f> top_layer = importCSV(terrain_layer_name);

    // Add SPH particles from the sampler points to the FSI system
    auto gz = std::abs(gravity.z());

    // Define BCE point vector
    std::vector<ChVector3d> bce_markers_container;  // bce marker defining the terrain bed

    // Defining how many extra BCE particles to add
    double bce_const = 0.05;

    int num_markers_top_layer = top_layer.size();

    // Add SPH and BCE particles from the top layer
    int num_layers = 20;
    int num_bce_layers = 3;
    ChVector3f point_loc;
    float pre_ini;
    for (int i = 0; i < num_markers_top_layer; i++){
        point_loc = top_layer.at(i);
        pre_ini = 0; // top layer has zero pressure
        for (int j = 0; j < num_layers; j++){
            point_loc.z() -= iniSpacing;
            pre_ini += sysFSI.GetDensity() * gz * iniSpacing;
            sysFSI.AddSPHParticle(point_loc, sysFSI.GetDensity(), 0, sysFSI.GetViscosity(),
                                  ChVector3d(0),         // initial velocity
                                  ChVector3d(-pre_ini),  // tauxxyyzz
                                  ChVector3d(0)          // tauxyxzyz
            );
        }

        // Add bce markers at the bottom of the terrain 
        for (int j = 0; j < num_bce_layers; j++){
            point_loc.z() -= iniSpacing;
            bce_markers_container.push_back(point_loc);
        }
    }

    // auto read_pts_top = importCSV(GetChronoDataFile("robot/viper/csv/top_layer.csv"));
    // for (int i = 0; i < size_terrain_top; i++) {
    //    ChVector3d point_loc;
    //    point_loc.x() = read_pts_top[i][0];
    //    point_loc.y() = read_pts_top[i][1];
    //    point_loc.z() = read_pts_top[i][2] + 0.2015f;
    //    if ((point_loc.y() > ystart && point_loc.y() < rec1_yend && point_loc.x() > rec1_xstart && point_loc.x() < rec2_xstart) 
    //        || (point_loc.y() > ystart && point_loc.y() < rec2_yend && point_loc.x() >= rec2_xstart && point_loc.x() < rec2_xend)) {
    //        continue;
    //    }
    //    double pre_ini = sysFSI.GetDensity() * gz * (-point_loc.z() + bzDim);
    //    sysFSI.AddSPHParticle(point_loc, sysFSI.GetDensity(), 0, sysFSI.GetViscosity(),
    //               ChVector3d(0),         // initial velocity
    //               ChVector3d(-pre_ini),  // tauxxyyzz
    //               ChVector3d(0)          // tauxyxzyz
    //    );
    // }

    std::vector<ChVector3f> wall_bce = generateWallBCE(iniSpacing);
    for (int i = 0; i < wall_bce.size(); i++){
        bce_markers_container.push_back(wall_bce.at(i));
    }

    // Loading BCE particles
    sysFSI.AddPointsBCE(m_ground, bce_markers_container, ChFrame<>(), false);

      // auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    double wheel_AngVel = 0.8;
    auto driver = chrono_types::make_shared<ViperSpeedDriver>(0.1, wheel_AngVel);
    // auto driver = chrono_types::make_shared<ViperSpeedDriver>(0.1, 0.0f);
    rover = chrono_types::make_shared<Viper>(&sysMBS, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    rover->Initialize(ChFrame<>(init_loc, QuatFromAngleAxis(CH_PI_2, {0,0,1})));

     auto chassis = chrono_types::make_shared<ChBody>();
     chassis->SetMass(10.0);
     chassis->SetPos(init_loc);
     chassis->EnableCollision(false);
     chassis->SetFixed(false);

    //// Add geometry of the chassis.
     chrono::utils::AddBoxGeometry(chassis.get(), CustomWheelMaterial(ChContactMethod::NSC), ChVector3d(0.1, 0.1, 0.1),
                                   ChVector3d(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), false);

     sysMBS.AddBody(chassis);

      // Connect the chassis to the containing bin (ground) through a translational joint and create a linear
     // actuator.
     //auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
     //prismatic1->Initialize(box, chassis, ChFrame<>(chassis->GetPos(), QuatFromAngleY(CH_PI_2)));
     //prismatic1->SetName("prismatic_chassis_ground");
     //sysMBS.AddLink(prismatic1);

     // Connect the axle to the chassis through a vertical translational joint.
     auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
     auto rover_body = rover->GetChassis()->GetBody();
     prismatic2->Initialize(chassis, rover_body, ChFrame<>(chassis->GetPos(), QUNIT));
     prismatic2->SetName("prismatic_rover_chassis");
     sysMBS.AddLink(prismatic2);

    // Create the wheel's BCE particles
    //auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    //double scale_ratio = 1.0;
    //trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    //trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    //trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    //std::vector<ChVector3d> BCE_wheel;
    //sysFSI.CreateMeshPoints(*trimesh, sysFSI.GetInitialSpacing(), BCE_wheel);

    // Add BCE particles and mesh of wheels to the system
    std::shared_ptr<ChBodyAuxRef> fr_wheel;
    std::shared_ptr<ChBodyAuxRef> fl_wheel;
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> wheel_body;
        if (i == 0) {
            wheel_body = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
            fl_wheel = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            wheel_body = rover->GetWheel(ViperWheelID::V_RF)->GetBody();
            fr_wheel = rover->GetWheel(ViperWheelID::V_RF)->GetBody();
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
        double inner_radius = wheel_radius-grouser_height;
        std::cout << "Adding Wheels" << std::endl;
        sysFSI.AddWheelBCE_Grouser(wheel_body, ChFrame<>(), inner_radius, wheel_wide - iniSpacing, 
                                    grouser_height, grouser_wide, grouser_num, kernelLength, false);
    }

    // Complete construction of the FSI system
    sysFSI.Initialize();
    // Get the body from the FSI system for visualization
    std::vector<std::shared_ptr<ChBody>>& FSI_Bodies = sysFSI.GetFsiBodies();
    auto Rover = FSI_Bodies[0];

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sysMBS.Add(floor);

    // Create a Sensor manager
    float intensity = 1.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sysMBS);
    manager->scene->AddPointLight({0, 0, 300}, {intensity, intensity, intensity}, 500);
    manager->scene->SetAmbientLight({.1, .1, .1});
    Background b;
    //b.mode = BackgroundMode::SOLID_COLOR;
    //b.color_zenith = ChVector3f(0.f,0.f,0.f);  //0.1f, 0.2f, 0.4f
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/starmap_2020_4k.hdr");
    manager->scene->SetBackground(b);
    manager->SetVerbose(false);
    

     chrono::ChFrame<double> offset_pose1({-5, 2, 2}, QuatFromAngleAxis(0, {0, 1, 0})); // Q_from_AngAxis(CH_PI_4, {0, 1, 0})  //-1200, -252, 100
    auto cam = chrono_types::make_shared<ChCameraSensor>(rover->GetChassis()->GetBody(),         // body camera is attached to
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
        cam->PushFilter(
            chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "CRM_DEMO_THIRD_PERSON_VIEW_RealSlope/"));
    manager->AddSensor(cam);

        
    chrono::ChFrame<double> offset_pose2({-0.5, -1.7, 0.5}, QuatFromAngleAxis(.2, {-2, 3, 9.75}));
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
        cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Wheel_Cam/"));
    manager->AddSensor(cam2);

    chrono::ChFrame<double> offset_pose3({-2.f, 0, .5f}, QuatFromAngleAxis(.2, {0, 1, 0}));
    // chrono::ChFrame<double> offset_pose3({0, 0, 5.f}, Q_from_AngAxis(CH_C_PI_2, {0, 1, 0}));
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
        cam3->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Rear_Cam/"));
    manager->AddSensor(cam3);

    // chrono::ChFrame<double> offset_pose4({-2.f, 0, .5f}, Q_from_AngAxis(.2, {0, 1, 0}));
    //chrono::ChFrame<double> offset_pose4({0, 0, 5.f}, QuatFromAngleAxis(CH_PI, {0, 1, 0}));
    //auto cam4 = chrono_types::make_shared<ChCameraSensor>(rover->GetChassis()->GetBody(),  // body camera is attached to
    //                                                      update_rate,                     // update rate in Hz
    //                                                      offset_pose4,                    // offset pose
    //                                                      image_width,                     // image width
    //                                                      image_height,                    // image height
    //                                                      fov,           // camera's horizontal field of view
    //                                                      alias_factor,  // super sampling factor
    //                                                      lens_model,    // lens model type
    //                                                      false, 2.2);
    //cam4->SetName("Top Camera");
    //cam4->SetLag(lag);
    //cam4->SetCollectionWindow(exposure_time);
    //if (vis)
    //    cam4->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Top Camera"));

    //if (save)
    //    cam4->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Top_Cam/"));
    //manager->AddSensor(cam4);
     // Create a run-tme visualizer
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
    std::shared_ptr<ChFsiVisualization> fsi_vis;
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
        fsi_vis->SetTitle("Viper on SPH terrain");
        fsi_vis->SetSize(1920, 1080);
        fsi_vis->UpdateCamera(init_loc + ChVector3d(7.5f, 13.0f, 1.9f), init_loc);
        fsi_vis->SetCameraMoveScale(0.2f);
        fsi_vis->EnableBoundaryMarkers(false);
        fsi_vis->EnableRigidBodyMarkers(false);
        fsi_vis->AttachSystem(&sysMBS);
        fsi_vis->Initialize();
    }

    // Start the simulation
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));

    double time = 0.0;
    int current_step = 0;
    double total_time = 40.0;

    auto body = sysMBS.GetBodies()[1];

    double steering = 0;
    double mid = (rec2_xstart + rec2_xend) / 2;
    double max_steering = CH_PI / 3;

    ChTimer timer;

    std::ofstream ofile;
    if (output)
        ofile.open(out_dir + "./body_position.txt");


    float* h_points = sysFSI.GetParticleData();
    int n_pts = sysFSI.GetNumFluidMarkers();
    std::vector<openvdb::Vec3R> vdbBuffer;

    while (time < total_time) {
        std::cout << current_step << "  time: " << time << "  sim. time: " << timer() << "  RTF: " << timer() / time << std::endl;
        
        // 9.01 is too far to the left; might implement automatic turning to stay on track
        //steering = 0;
        //if (time > 1.5 && time < 7.0) {
        //    steering = max_steering * (time - 0.5) / 2;
        //} else if (time >= 7.0) {
        //    // ChVector3d cur_position = body->GetPos();
        //    // double diff = cur_position[0] - mid;
        //    // steering = max_steering * diff / 2;
        //    ChVector3d pos_fr = fr_wheel->GetPos();
        //    ChVector3d pos_fl = fl_wheel->GetPos();
        //    // double diff_r = pos_fr[0] - 0.5 - mid;
        //    if (pos_fl[0] < mid - 0.6f){
        //        steering = -max_steering/15.0f;
        //        std::cout<<"turn right"<<std::endl;
        //    } 
        //    else if (pos_fr[0] > mid + 0.6f) {
        //        steering = max_steering / 15.0f;
        //        std::cout<<"turn left"<<std::endl;
        //    }
        //    // if (pos_fr[0] > mid + 0.475f){
        //    //     steering = max_steering/10.0f;
        //    //     std::cout<<"turn left"<<std::endl;
        //    // }
        //    // steering = max_steering * diff_r / 6;
        //    // std::cout << "Left Wheel x Position: " << pos_fl[0] << std::endl;
        //    // std::cout << "Right Wheel x Position: " << pos_fr[0] << std::endl;
        //}
        //std::cout << steering << std::endl;
        //driver->SetSteering(steering);

              
        if (current_step % render_steps == 0) {
            /* timerNVDB.start();
             createNanoVDBGrid(sysMBS, sysFSI, vol_bbox->GetId());
             timerNVDB.stop();
             std::cout << "Adding NVDB points:" << timerNVDB.GetTimeMilliseconds() << "ms" << std::endl;
             timerNVDB.reset();*/
            h_points = sysFSI.GetParticleData();
            n_pts = sysFSI.GetNumFluidMarkers();

            vdbBuffer = floatToVec3R(h_points, n_pts);
            createVoxelGrid(vdbBuffer, sysMBS);
            manager->scene->SetFSIParticles(h_points);  // remove these later
            manager->scene->SetFSINumFSIParticles(n_pts);
        }

        manager->Update();

        rover->Update();


        if (output) {
            ofile << time << "  " << body->GetPos() << "    " << body->GetPosDt() << std::endl;
            if (current_step % output_steps == 0) {
                sysFSI.PrintParticleToFile(out_dir + "/particles");
                sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
                SaveParaViewFiles(sysFSI, sysMBS, time);
            }
        }
        //std::cout << "  Position: " << body->GetPos() << std::endl;
        //std::cout << "  vel: " << body->GetPosDt() << std::endl;

        // Render system
        if (render && current_step % render_steps == 0) {
            if (!fsi_vis->Render())
                break;
        }

        timer.start();
        sysFSI.DoStepDynamics_FSI();
        timer.stop();

        time += dT;
        current_step++;
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

void createVoxelGrid(std::vector<openvdb::Vec3R>& points, ChSystemNSC& sys) {
    std::cout << "Creating OpenVDB Voxel Grid" << std::endl;
    openvdb::initialize();
    openvdb::points::PointAttributeVector<openvdb::Vec3R> positionsWrapper(points);
    float r = 0.05f;
    int pointsPerVoxel = 10;
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
                auto voxelBody = sys.GetBodies()[idList[voxelCount]];
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
                sys.Add(voxelBody);
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
                idList.emplace_back(voxelBody->GetIndex());
            }
            voxelCount++;
        }
    }
    prevActiveVoxels = voxelCount;
    std::wcout << "Num Voxels: " << voxelCount << std::endl;
    firstInst = false;
}