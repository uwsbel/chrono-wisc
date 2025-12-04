// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bo-Hsun Chen
// =============================================================================
//
// Demo to show VIPER operating on a digitized SCM terrain with wheel sinkage
// estimation and depth map prediction
//
// =============================================================================
#include <iostream>
#include <time.h> 
#include <cstring>
#include <sstream>

#include "chrono_models/robot/viper/Viper.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChMassProperties.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterCameraExposure.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "builtin_interfaces/msg/time.hpp"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::viper;
using namespace chrono::sensor;
using namespace irr;

//// Camera model parameter setting ////
CameraLensModelType lens_model = CameraLensModelType::PINHOLE; // Camera lens model, Either PINHOLE or SPHERICAL
Integrator cam_integrator = Integrator::LEGACY;
float update_rate = 30; // [Hz], Update rate
unsigned int image_width = 1280; // [px], viewer width
unsigned int image_height = 720; // [px], viewer height
unsigned int front_camera_width = 1936; // [px], front camera width
unsigned int front_camera_height = 1216; // [px], front camera height
float viewer_fov = CH_PI / 2.; // viewer's horizontal field of view (hFOV)
float front_camera_fov = (44.0 + 1.0/60) * CH_PI/180.0; // [rad], front camera's horizontal field of view (hFOV)
float sink_observer_fov = (float)CH_PI / 18.; // [rad], wheel sinkage observer's horizontal field of view (hFOV)
float lag = 0.0f; // [sec], lag between sensing and when data becomes accessible
int alias_factor = 2; // Squared root of sample per pixel (spp)

//// Dynamic solver parameter setting ////
double sim_end_time = 40.f; // [sec], simulation end time
// double sim_time_step = 2.5e-4; // [sec], simulation time step
double sim_time_step = 2.5e-3; // debug

//// VIPER model setting ////
ViperWheelType wheel_type = ViperWheelType::CustomizedWheel; // define VIPER rover wheel type (RealWheel, SimpleWheel, CylWheel, or CustomizedWheel)
double wheel_range = 0.25; // [m], global parameter for moving patch size
double dsr_wheel_ang_vel = 2.0; // [rad/sec]

//// terrain setting ////
float terrain_scale_ratio = 1.0;
double mesh_resolution = 0.02; // [m], SCM grid spacing
int nthreads = 40; // number of SCM and collision threads
bool var_params = true; // if true, use provided callback to change soil properties based on location
const std::string decorated_ground_mesh_path = GetChronoDataFile("robot/curiosity/rocks/Terrain06/terrain06_ground_decimate_005.obj");

//// Paths to save results ////
const std::string out_dir = GetChronoOutputPath() + "VIPER_SCM_WheelSinkage/"; // output base folder for saved images

//// ROS parameters ////
double ROS_publish_rate = 15; // [Hz], ROS image_message publishing rate
int pub_buf_size = 10; // size of a publisher buffer

//// Switch setting ////
bool save = false; // Save camera and viewer images
bool vis = false; // visualize camera
bool add_rocks = false; // whether add rocks into the scene
bool enable_bulldozing = true; // enable/disable bulldozing effects
bool debug_terrain = false; // whether just check the terrain and stop running VIPER
bool exposure_correction_switch = true; // whether add exposure correction filter

// Given time stamp in second, return time stamp in ROS format
builtin_interfaces::msg::Time GetROSTimestamp(double time_s) {
    builtin_interfaces::msg::Time timestamp;
    timestamp.sec = static_cast<int32_t>(time_s);
    timestamp.nanosec = static_cast<uint32_t>((time_s - timestamp.sec) * 1e9);
    return timestamp;
}

class RGBA8PublisherNode : public rclcpp::Node {
	public:
		explicit RGBA8PublisherNode(const std::string node_name = "") : rclcpp::Node(node_name) {}		
		
		void AddImgPublisher(std::string topic_name = "rgba8_image") {
			m_publisher_map[topic_name] = this->create_publisher<sensor_msgs::msg::Image>(topic_name, pub_buf_size);
		}

		/// Call this from the main loop to publish the image message
		void PublishImgMsg(std::string topic_name, sensor_msgs::msg::Image &img_msg) {
			m_publisher_map[topic_name]->publish(img_msg);
		}

	private:
		std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> m_publisher_map;
};

// Customized callback for setting location-dependent soil properties.
// Note that the location is given in the SCM reference frame.
class MySoilParams : public vehicle::SCMTerrain::SoilParametersCallback {
  public:
    virtual void Set(const ChVector3d& loc,
                     double& Bekker_Kphi,
                     double& Bekker_Kc,
                     double& Bekker_n,
                     double& Mohr_cohesion,
                     double& Mohr_friction,
                     double& Janosi_shear,
                     double& elastic_K,
                     double& damping_R) override {
        Bekker_Kphi = 0.82e6 / 8;
        Bekker_Kc = 0.14e4 / 8;
        Bekker_n = 1.0;
        Mohr_cohesion = 0.017e4;
        Mohr_friction = 35.0;
        Janosi_shear = 1.78e-2;
        elastic_K = 2e8; // original 2e8
        damping_R = 3e4;
    }
};

// Return customized wheel material parameters
std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.8f;   // coefficient of friction, original 0.4f
    float cr = 0.1f;   // coefficient of restitution
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

int main(int argc, char* argv[]) {
	if (argc < 4) {
		std::cout << "./demo_ROBOT_Viper_SCM_POLAR [Sun Position: 1, 2, 3, 4] [enable Hapke: 0, 1] [exposure time (sec)] [--save] [--viz] [--add_rocks]\n";
		exit(1);
	}
	std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

		// ------------------- //
	// Configuration table //
	// ------------------- //
	// Map of camera poses
	std::unordered_map<char, std::unordered_map<char, ChVector3f>> camera_posi_map{ // [m]
		{'A', {{'L', {3.541f, -0.303f, 1.346f}}, {'R', {3.532f, -0.002f, 1.344f}}}},
		{'B', {{'L', {5.500f, -0.303f, 1.345f}}, {'R', {5.500f, -0.002f, 1.344f}}}},
		{'C', {{'L', {0.316f, -3.540f, 1.346f}}, {'R', {0.611f, -3.479f, 1.344f}}}}
	};

	// Map of Sun poses
	float sun_height = 100 * tanf(8 * CH_PI/180.0); // [m]
	std::unordered_map<int, ChVector3d> sunID_pose_map = {
		{1, ChVector3d( 100.0, 0.0, sun_height)},														// [m], azi =   0 deg
		{2, ChVector3d( 100.0 * cosf(270 * CH_PI/180.0), 100.0 * sinf(270 * CH_PI/180.0), sun_height)},	// [m], azi = 270 deg
		{3, ChVector3d(-100.0, 0.0, sun_height)},														// [m], azi = 180 deg
		{4, ChVector3d( 100.0 * cosf( 90 * CH_PI/180.0), 100.0 * sinf( 90 * CH_PI/180.0), sun_height)},	// [m], azi =  90 deg
		
	};

	// Array of VIPER wheel IDs
	ViperWheelID viper_wheel_IDs[4] = {V_LF, V_RF, V_LB, V_RB};

	//// Assign argument parameters
	for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--save") == 0) {
            save = true;
            std::cout << "Sensor output saved." << std::endl;
        }
		else if (std::strcmp(argv[i], "--viz") == 0) {
            vis = true;
            std::cout << "Sensor output visualized." << std::endl;
        }
		else if (std::strcmp(argv[i], "--add_rocks") == 0) {
            add_rocks = true;
            std::cout << "Rocks added into the scene." << std::endl;
        }
    }

	ChVector3d sun_pose = sunID_pose_map[std::atoi(argv[1])];
	bool enable_hapke = (std::atoi(argv[2]) > 0) ? true : false; // enable Hapke or not?---0(disable hapke)--1(enable hapke)
	float expsr_time = std::atof(argv[3]); // [sec], exposure time
	std::ostringstream expsr_time_str;
	expsr_time_str << std::setfill('0') << std::setw(4) << static_cast<int>(expsr_time * 1000 + 0.5);

	std::string brdf_type = "";
	if (enable_hapke) {
		brdf_type = "hapke";
	}
	else {
		brdf_type = "default";
	}

	// Print arguments
	std::cout << "Sun posi: " << sun_pose << ", BRDF: " << brdf_type << ", exposure time [sec]: " << expsr_time << std::endl;

	//// Create a Chrono::Engine physical system
	ChSystemNSC sys;
	sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81)); // [m/sec^2]
	sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
	sys.SetNumThreads(nthreads, nthreads, 1);

	// ------------------ //
	// Create Viper rover //
	// ------------------ //
	auto driver = chrono_types::make_shared<ViperSpeedDriver>(0.1, dsr_wheel_ang_vel);
	// auto driver = chrono_types::make_shared<ViperDCMotorControl>();
	// for (auto viper_wheel_ID: viper_wheel_IDs) {
	// 	driver->SetMotorNoLoadSpeed(-1.0 * CH_PI, viper_wheel_ID);
	// }
	Viper viper(&sys, wheel_type, "cobra_wheel_v2");
	viper.SetDriver(driver);
	viper.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
	viper.Initialize(ChFrame<>(ChVector3d(5.0, 0., 0.2), QuatFromAngleAxis(CH_PI, {0., 0, 1.}))); // [m, quaternion], good init. posi. for SCM model scaled 1.0

	// Get VIPER wheels and chassis body to set up SCM patches
	auto wheel_LF = viper.GetWheel(ViperWheelID::V_LF)->GetBody();
	auto wheel_RF = viper.GetWheel(ViperWheelID::V_RF)->GetBody();
	auto wheel_LB = viper.GetWheel(ViperWheelID::V_LB)->GetBody();
	auto wheel_RB = viper.GetWheel(ViperWheelID::V_RB)->GetBody();
	auto viper_body = viper.GetChassis()->GetBody();
	
	auto wheel_mat = chrono_types::make_shared<ChVisualMaterial>();
	wheel_mat->SetDiffuseColor({2.0, 2.0, 2.0});
	wheel_mat->SetSpecularColor({2.0, 2.0, 2.0});
	wheel_mat->SetUseSpecularWorkflow(true);
	wheel_mat->SetMetallic(0.f);
	wheel_mat->SetRoughness(1.0f);
	wheel_LB->GetVisualShape(0)->SetMaterial(0, wheel_mat);
	wheel_LB->GetVisualShape(0)->GetMaterial(0)->SetClassID(0);
	wheel_LB->GetVisualShape(0)->GetMaterial(0)->SetInstanceID(65535);

	// ---------------- //
	// Set up the scene //
	// ---------------- //
	// Create lunar regolith visual material
	auto regolith_mat = chrono_types::make_shared<ChVisualMaterial>();
	auto regolith_tex_mat = chrono_types::make_shared<ChVisualMaterial>();
	if (enable_hapke) {
        regolith_mat->SetBSDF((unsigned int)BSDFType::HAPKE);
        regolith_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));
		regolith_tex_mat->SetBSDF((unsigned int)BSDFType::HAPKE);
        regolith_tex_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));
    }
    else {
        // regolith_mat->SetDiffuseColor({0.5738f, 0.5174f, 0.5666f});
        // regolith_mat->SetRoughness(0.0267f);
        // regolith_mat->SetMetallic(0.5399f);
        // regolith_mat->SetUseSpecularWorkflow(false);

		regolith_mat->SetDiffuseColor({0.4 * 1.983, 0.4 * 1.667, 0.4 * 1.410});
		regolith_mat->SetUseSpecularWorkflow(true);
		regolith_mat->SetRoughness(1.0f);
		regolith_mat->SetMetallic(0.f);
		
		regolith_tex_mat->SetKdTexture(GetChronoDataFile("robot/curiosity/rocks/moon_dusted_05_diff_1k.png"));
		regolith_tex_mat->SetRoughnessTexture(GetChronoDataFile("robot/curiosity/rocks/moon_dusted_05_rough_1k.png"));
		regolith_tex_mat->SetNormalMapTexture(GetChronoDataFile("robot/curiosity/rocks/moon_dusted_05_nor_gl_1k.png"));
		regolith_tex_mat->SetDisplacementTexture(GetChronoDataFile("robot/curiosity/rocks/moon_dusted_05_disp_1k.png"));
		regolith_tex_mat->SetMetallic(0.f);
		regolith_tex_mat->SetTextureScale(9.f, 3.f);
    }
	
	regolith_mat->SetClassID(255); // first 4 bits in semantic cam, FF00
	regolith_mat->SetInstanceID(65280); // last 4 bits in semantic cam, 00FF
	regolith_tex_mat->SetClassID(255); // first 4 bits in semantic cam, FF00
    regolith_tex_mat->SetInstanceID(65280); // last 4 bits in semantic cam, 00FF

	////// Set up decorated grounds
	printf("Loading decorated ground into system ...\n");
	// auto decorated_ground_mat = chrono_types::make_shared<ChVisualMaterial>();
	auto decorated_ground_mesh_side = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
	auto decorated_ground_body_left = chrono_types::make_shared<ChBodyAuxRef>();
	auto decorated_ground_body_right = chrono_types::make_shared<ChBodyAuxRef>();
	auto decorated_ground_body_front = chrono_types::make_shared<ChBodyEasyBox>(6.5, 3.8, 0.2, 1000, true, false);
	auto decorated_ground_body_back = chrono_types::make_shared<ChBodyEasyBox>(7.0, 3.8, 0.2, 1000, true, false);

	//// Set up side ground meshes
	// Load mesh from obj file
	auto decorated_ground_mesh_side_loader = ChTriangleMeshConnected::CreateFromWavefrontFile(decorated_ground_mesh_path, true, true);
	decorated_ground_mesh_side_loader->Transform(
		ChVector3d(0, 0, 0),
		ChMatrix33<>(
			{7.0, 1.0, 1.0}, // scale to a different size
			{0., 0., 0.}
		) 
	);
	decorated_ground_mesh_side_loader->RepairDuplicateVertexes(1e-9); // if meshes are not watertight
	decorated_ground_mesh_side->SetMesh(decorated_ground_mesh_side_loader);
	decorated_ground_mesh_side->SetBackfaceCull(true);

	// Set up side decorated-grounds
	decorated_ground_body_left->AddVisualShape(decorated_ground_mesh_side);
	decorated_ground_body_left->GetVisualShape(0)->SetMaterial(0, regolith_mat);
	decorated_ground_body_left->SetFixed(true);
	decorated_ground_body_left->SetPos({0., -3.7, -0.32}); // [m]
	decorated_ground_body_left->SetRot(QuatFromAngleAxis(-0.5 * (CH_PI / 180), {0, 1, 0}));
	sys.Add(decorated_ground_body_left);

	decorated_ground_body_right->AddVisualShape(decorated_ground_mesh_side);
	decorated_ground_body_right->GetVisualShape(0)->SetMaterial(0, regolith_mat);
	decorated_ground_body_right->SetFixed(true);
	decorated_ground_body_right->SetPos({0.,  3.7, -0.39}); // [m]
	decorated_ground_body_right->SetRot(QuatFromAngleAxis(-0.5 * (CH_PI / 180), {0, 1, 0}));
	sys.Add(decorated_ground_body_right);

	//// Set up front ground mesh
	decorated_ground_body_front->SetFixed(true);
	decorated_ground_body_front->SetPos({-9.48, 0.10, -0.505}); // [m]
	decorated_ground_body_front->SetRot(QuatFromAngleAxis(-1.0 * (CH_PI/180), {0, 1, 0}));
	decorated_ground_body_front->GetVisualShape(0)->SetMaterial(0, regolith_mat);
	// {
	// 	auto shape = decorated_ground_body_front->GetVisualModel()->GetShapeInstances()[0].shape;
	// 	if (shape->GetNumMaterials() == 0) {
	// 		shape->AddMaterial(decorated_ground_mat);
	// 	} else {
	// 		shape->GetMaterials()[0] = decorated_ground_mat;
	// 	}
	// }

	sys.Add(decorated_ground_body_front);

	//// Set up back ground mesh
	decorated_ground_body_back->SetPos({9.5, 0, -0.38});
	decorated_ground_body_back->SetFixed(true);
	// {
	// 	auto shape = decorated_ground_body_back->GetVisualModel()->GetShapeInstances()[0].shape;
	// 	if (shape->GetNumMaterials() == 0) {
	// 		shape->AddMaterial(decorated_ground_mat);
	// 	} else {
	// 		shape->GetMaterials()[0] = decorated_ground_mat;
	// 	}
	// }
	decorated_ground_body_back->GetVisualShape(0)->SetMaterial(0, regolith_mat);
	sys.Add(decorated_ground_body_back);

	printf("Finished Loading decorated ground into system\n");

	//// Set up rocks ////
	if (add_rocks == true) {
		std::vector<std::string> terrains = {"04", "01", "11"};
		// std::vector<std::string> terrains = {"04", "01"};
		// std::vector<std::string> terrains = {"11"};
		// Looping through all available terrains
		for (std::string terrainID : terrains) {
			std::string rock_mesh_dir = GetChronoDataFile("robot/curiosity/rocks/Terrain" + terrainID + "/");
			size_t rock_num = 0;
			float terrain_x_offset = 0., terrain_y_offset = 0., terrain_z_offset = 0.;
			float terrain_x_rot = 0.;
			
			if (terrainID == "04") { // Terrain 4
				rock_num = 27;
				terrain_x_offset = 2.45; // [m], for v12
				terrain_y_offset = 0; // [m], for v12
				terrain_z_offset = -0.527; // [m], for v12
				terrain_x_rot = -0.4 * CH_PI/180.0; // [rad]
			}
			else if (terrainID == "01") { // Terrain 01
				rock_num = 15;
				terrain_x_offset = -1.07; // [m], for v12
				terrain_y_offset = 0.; // [m], for v12
				terrain_z_offset = -0.274; // [m], for v12
				terrain_x_rot = -0.1 * CH_PI/180.0; // [rad]
			}
			else if (terrainID == "11") { // Terrain 11
				rock_num = 9;
				terrain_x_offset = -4.22; // [m], for v12
				terrain_y_offset = -0.22; // [m], for v12
				terrain_z_offset = -0.22; // [m], for v12
				terrain_x_rot = 2.5 * CH_PI/180.0; // [rad]
			}
			else {
				std::cout << "unknown terrain ID\n";
				exit(1);
			}
			
			std::vector<std::shared_ptr<ChBodyAuxRef>> rock_bodies(rock_num);
			std::string rock_visual_mesh_path = "";
			std::string rock_contact_mesh_path = "";
			std::shared_ptr<ChContactMaterial> rock_contact_mat = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());
			double mmass;
			ChVector3d mcog;
			ChMatrix33<> minertia;
			double rock_density = 8000;  // [kg/m^3];
			ChMatrix33<> principal_inertia_rot;
			ChVector3d principal_I;

			// ------------------------ //
			// Visible/collidable rocks //
			// ------------------------ //
			for (size_t rock_idx = 0; rock_idx < rock_num; ++rock_idx) {
				//// Set up rock mesh
				// Load mesh from obj file
				if (terrainID == "test" && rock_num > 0) {
					rock_visual_mesh_path = GetChronoDataFile("robot/curiosity/rocks/Terrain" + std::string(argv[3]) + "/terrain" + std::string(argv[3]) + "_rock" + std::to_string(rock_idx+1) + ".obj"); // test
				}
				else {
					rock_visual_mesh_path = rock_mesh_dir + "terrain" + terrainID + "_rock" + std::to_string(rock_idx+1) + ".obj";
					// rock_visual_mesh_path = rock_mesh_dir + "terrain" + terrainID + "_rock" + std::to_string(rock_idx+1) + "_decimate-005.obj"; // debug
				}

				auto rock_visual_mesh_loader = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_visual_mesh_path, false, true);
				rock_visual_mesh_loader->Transform(
					ChVector3d(terrain_x_offset, terrain_y_offset, terrain_z_offset),
					ChMatrix33<>(terrain_scale_ratio) * ChMatrix33<>(QuatFromAngleAxis(0., {1, 0, 0})) // scale to a different size
				);
				rock_visual_mesh_loader->RepairDuplicateVertexes(1e-9); // if meshes are not watertight

				auto rock_visual_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
				rock_visual_shape->SetMesh(rock_visual_mesh_loader);
				rock_visual_shape->SetBackfaceCull(true);
				
				// Set up rocks
				rock_bodies[rock_idx] = chrono_types::make_shared<ChBodyAuxRef>();

				// Set rock appearance
				rock_bodies[rock_idx]->AddVisualShape(rock_visual_shape);
				rock_bodies[rock_idx]->GetVisualShape(0)->SetMaterial(0, regolith_mat);

				// Set up rock cotact shape
				std::string rock_contact_mesh_path = rock_mesh_dir + "terrain" + terrainID + "_rock" + std::to_string(rock_idx+1) + "_decimate-005.obj";
				auto rock_contact_mesh_loader = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_contact_mesh_path, false, true);
				rock_contact_mesh_loader->Transform(
					ChVector3d(terrain_x_offset, terrain_y_offset, terrain_z_offset),
					ChMatrix33<>(terrain_scale_ratio) * ChMatrix33<>(QuatFromAngleAxis(0., {1, 0, 0})) // scale to a different size
				);
				rock_contact_mesh_loader->RepairDuplicateVertexes(1e-9); // if meshes are not watertight

				auto rock_contact_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
					rock_contact_mat, rock_contact_mesh_loader, false, false, 0.005
				);
				rock_bodies[rock_idx]->AddCollisionShape(rock_contact_shape);
				rock_bodies[rock_idx]->EnableCollision(true);

				rock_bodies[rock_idx]->SetFixed(true);
				rock_bodies[rock_idx]->SetPos({0., 0., 0.}); // [m]
				rock_bodies[rock_idx]->SetRot(ChMatrix33<>(terrain_x_rot, {1., 0., 0.}));
				
				sys.Add(rock_bodies[rock_idx]);
				printf("Rock %zu of Terrain %s added to system\n", rock_idx + 1, terrainID.c_str());
			}
		}
	}

	// ---------------- //
	// Creat the ground //
	// ---------------- //
	// std::string ground_mesh_path = GetChronoDataFile("robot/curiosity/rocks/big_ground_scale1_v12.obj"); // for MAES version
	std::string ground_mesh_path = GetChronoDataFile("robot/curiosity/rocks/big_ground_scale1_v15.obj");

	//// Create the deformable SCM ground ////
	vehicle::SCMTerrain ground(&sys);
	ground.SetReferenceFrame(ChCoordsys<>(ChVector3d(0, 0, 0.08)));

	ground.Initialize(ground_mesh_path, mesh_resolution);
	ground.SetMeshWireframe(false);
	{
		auto mesh = ground.GetMesh();
		if(mesh->GetNumMaterials() == 0) mesh->AddMaterial(regolith_tex_mat);
		else mesh->GetMaterials()[0] = regolith_tex_mat;
	}

	//// Set the soil terramechanical parameters
	// Here we use the soil callback defined at the beginning of the code
	if (var_params) { 
		auto my_params = chrono_types::make_shared<MySoilParams>();
		ground.RegisterSoilParametersCallback(my_params);
	}
	// If var_params is set to be false, these default parameters will be used
	else { 
		ground.SetSoilParameters(
			0.2e6,  // Bekker Kphi
			0,      // Bekker Kc
			1.1,    // Bekker n exponent
			0,      // Mohr cohesive limit (Pa)
			30,     // Mohr friction limit (degrees)
			0.01,   // Janosi shear coefficient (m)
			4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
			3e2     // Damping (Pa s/m), proportional to negative vertical speed (optional)
		);
	}

	// Set up bulldozing effect factors
	if (enable_bulldozing) {
		ground.EnableBulldozing(true);  // inflate soil at the border of the rut
		ground.SetBulldozingParameters(
			55,     // angle of friction for erosion of displaced material at the border of the rut
			1,      // displaced material vs downward pressed material.
			5,      // number of erosion refinements per timestep
			6       // number of concentric vertex selections subject to erosion
		);
	}

	// We need to add a moving patch under every wheel on the ground, or we can define a large moving patch at the pos of the rover body
	// for VIPER moving on the ground
	ground.AddActiveDomain(wheel_LF, ChVector3d(0, 0, 0), ChVector3d(0.5, wheel_range, wheel_range));
	ground.AddActiveDomain(wheel_RF, ChVector3d(0, 0, 0), ChVector3d(0.5, wheel_range, wheel_range));
	ground.AddActiveDomain(wheel_LB, ChVector3d(0, 0, 0), ChVector3d(0.5, wheel_range, wheel_range));
	ground.AddActiveDomain(wheel_RB, ChVector3d(0, 0, 0), ChVector3d(0.5, wheel_range, wheel_range));

	// Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
	//ground.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);

	// create a fixed origin cube to attach camera
	auto origin_cube = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
	origin_cube->SetPos({0, 0, 0});
	origin_cube->SetFixed(true);
	sys.Add(origin_cube);

	// --------------------- //
	// Create sensor manager //
	// --------------------- //
	auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
	manager->SetVerbose(false);

	// Add lights
	manager->scene->AddPointLight(sun_pose, {1.0, 1.0, 1.0}, 2000.0f);
	manager->scene->SetAmbientLight({.01, .01, .01});

	// Set up background
	Background bgd;
	// bgd.mode = BackgroundMode::ENVIRONMENT_MAP;
	// bgd.env_tex = GetChronoDataFile("sensor/textures/starmap_2020_4k_darkened.hdr");
	
	bgd.mode = BackgroundMode::SOLID_COLOR;
	bgd.color_zenith = {0, 0, 0};
	
	manager->scene->SetBackground(bgd);

	// ------------------------- //
	// Creat cameras and viewers //
	// ------------------------- //
	std::vector<float> expsr_a_arr(3, 0.); // exposure correction coefficients of a-term
	std::vector<float> expsr_b_arr(3, 0.); // exposure correction coefficients of b-term
	expsr_a_arr = {0., 0., 1.0}; // simple Default model
	if (enable_hapke) {
		expsr_b_arr = {0.698932456, -0.862405122, -7.530271991}; // simple Hapke model, avg of Terrains 01, 11, and 04
	}
	else {
		expsr_b_arr = {0.698938172, -0.918033419, -7.751329574}; // simple Default model, avg of Terrains 01, 11, and 04
	}

	// Set up left bird-eye-view camera
	chrono::ChFrame<double> cam_birdview_left_pose({0, -5.5, 2.0}, QuatFromAngleAxis(CH_PI/2, {0, 0, 1}));
	// chrono::ChFrame<double> cam_birdview_left_pose({-5.0, -7.0, 2.0}, QuatFromAngleAxis(CH_PI/2, {0, 0, 1})); // calibrating
	cam_birdview_left_pose.SetRot(cam_birdview_left_pose.GetRot() * QuatFromAngleAxis(20. * CH_PI/180, {0, 1, 0}));
	auto cam_birdview_left = chrono_types::make_shared<ChCameraSensor>(
		// viper.GetChassis()->GetBody(),		// body camera is attached to
		origin_cube,						// body that camera is attached to
		update_rate,						// update rate in Hz
		cam_birdview_left_pose,				// offset pose
		image_width,                    // image width
		image_height,                   // image height
		viewer_fov,                     // camera's horizontal field of view
		alias_factor,                   // supersample factor for antialiasing
		lens_model,                     // lens model
		false,							// use global illumination or not
		cam_integrator					// integrator
	);
	cam_birdview_left->SetName("Left Bird Viewer");
	cam_birdview_left->SetLag(lag);
	cam_birdview_left->SetCollectionWindow(0.0f); // would cause dynamic blur effect
	if (exposure_correction_switch == true) {
		std::cout << expsr_a_arr[0] << std::endl;
		cam_birdview_left->PushFilter(chrono_types::make_shared<ChFilterCameraExposureCorrect>(
			expsr_a_arr[0], expsr_a_arr[1], expsr_a_arr[2],
			expsr_b_arr[0], expsr_b_arr[1], expsr_b_arr[2], expsr_time)
		);
	}
	if (vis) {
		cam_birdview_left->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
			image_width, image_height, "Left bird view"
		));
	}
	if (save) {
		cam_birdview_left->PushFilter(chrono_types::make_shared<ChFilterSave>(
			out_dir + "CamBirdViewLeft_" + argv[1] + "_" + brdf_type + "_" + expsr_time_str.str() + "/"
		));
	}
	manager->AddSensor(cam_birdview_left);

	// Set up right bird-eye-view camera
	chrono::ChFrame<double> cam_birdview_right_pose({0, 5.5, 2.0}, QuatFromAngleAxis(-CH_PI/2, {0, 0, 1}));
	// chrono::ChFrame<double> cam_birdview_right_pose({-5.0, 8.0, 2.0}, QuatFromAngleAxis(-CH_PI/2, {0, 0, 1})); // calibrating
	cam_birdview_right_pose.SetRot(cam_birdview_right_pose.GetRot() * QuatFromAngleAxis(20. * CH_PI/180, {0, 1, 0}));
	auto cam_birdview_right = chrono_types::make_shared<ChCameraSensor>(
		// viper.GetChassis()->GetBody(),  // body camera is attached to
		origin_cube,                    // body that camera is attached to
		update_rate,                    // update rate in Hz
		cam_birdview_right_pose,         // offset pose
		image_width,                    // image width
		image_height,                   // image height
		viewer_fov,                     // camera's horizontal field of view
		alias_factor,                   // supersample factor for antialiasing
		lens_model,                     // lens model
		false,							// use global illumination or not
		cam_integrator					// integrator
	);
	cam_birdview_right->SetName("Right Bird Viewer");
	cam_birdview_right->SetLag(lag);
	cam_birdview_right->SetCollectionWindow(0.0f); // would cause dynamic blur effect
	if (exposure_correction_switch == true) {
		std::cout << expsr_a_arr[0] << std::endl;
		cam_birdview_right->PushFilter(chrono_types::make_shared<ChFilterCameraExposureCorrect>(
			expsr_a_arr[0], expsr_a_arr[1], expsr_a_arr[2],
			expsr_b_arr[0], expsr_b_arr[1], expsr_b_arr[2], expsr_time)
		);
	}
	if (vis) {
		cam_birdview_right->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
			image_width, image_height, "Right bird view"
		));
	}
	if (save) {
		cam_birdview_right->PushFilter(chrono_types::make_shared<ChFilterSave>(
			out_dir + "CamBirdViewRight_" + argv[1] + "_" + brdf_type + "_" + expsr_time_str.str() + "/"
		));
	}
	manager->AddSensor(cam_birdview_right);

	/*
	chrono::ChFrame<double> wheel_poses[4] = {
		chrono::ChFrame<double>({-1.0, -1.5, 0.5}, QuatFromAngleAxis(.2, {-2, 3,  9.75})), 
		chrono::ChFrame<double>({ 1.0, -1.5, 0.5}, QuatFromAngleAxis(.2, {-2, 3,  9.75})),
		chrono::ChFrame<double>({ 1.0,  1.5, 0.5}, QuatFromAngleAxis(.2, { 2, 3, -9.75})),
		chrono::ChFrame<double>({-1.0,  1.5, 0.5}, QuatFromAngleAxis(.2, { 2, 3, -9.75}))
	};

	std::vector<std::string> wheel_cam_names = {"RightBack", "RightFront", "LeftFront", "LeftBack"};

	auto rf_wheel = chrono::ChFrame<double>({1.0, 1.5, 0.5}, QuatFromAngleAxis(.2, {2, 3, -9.75}));
	for (int i = 0; i < 4; i++) {
		auto wheel_cam = chrono_types::make_shared<ChCameraSensor>(
			viper.GetChassis()->GetBody(),	// body camera is attached to
			update_rate,					// update rate in Hz
			wheel_poses[i],					// offset pose
			image_width,					// image width
			image_height,					// image height
			viewer_fov,						// camera's horizontal field of view
			alias_factor,					// supersample factor for antialiasing
			lens_model,						// lens model
			false,							// use global illumination or not
			cam_integrator      			// integrator
		);
		wheel_cam->SetName("Wheel Camera " + wheel_cam_names[i]);
		wheel_cam->SetLag(lag);
		wheel_cam->SetCollectionWindow(0.f); // would cause dynamic blur effect
		if (exposure_correction_switch == true) {
			wheel_cam->PushFilter(chrono_types::make_shared<ChFilterCameraExposureCorrect>(
				expsr_a_arr[0], expsr_a_arr[1], expsr_a_arr[2],
				expsr_b_arr[0], expsr_b_arr[1], expsr_b_arr[2], expsr_time + 0.100f
			));
		}
		if (vis)
			wheel_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
				image_width, image_height, "Wheel Camera " + wheel_cam_names[i]
			));
		if (save) {
			wheel_cam->PushFilter(chrono_types::make_shared<ChFilterSave>(
				out_dir + "WheelCam_" + wheel_cam_names[i] + "_" + argv[1] + "_" + brdf_type + "_" + expsr_time_str.str() + "/"
			));
		}
		manager->AddSensor(wheel_cam);
	}
	*/
	
	// Add Left-Back wheel sinkage observer
	ChVector3f LB_wheel_offset({-0.6418, 0.6098 + 0.22, 0});
    float cam_radius = 5.f; // [m]
    float cam_elevat = 7.f * CH_PI/180; // [rad]
    chrono::ChFrame<double> offset_sink_observer(
        {LB_wheel_offset.x(), LB_wheel_offset.y() + cam_radius * cosf(cam_elevat), LB_wheel_offset.z() + cam_radius * sinf(cam_elevat)},
        QuatFromAngleAxis(-CH_PI_2, {0, 0, 1}) * QuatFromAngleAxis(cam_elevat, {0, 1, 0})
    );
	auto wheel_cam_LB = chrono_types::make_shared<ChCameraSensor>(
		viper.GetChassis()->GetBody(),	// body camera is attached to
		// origin_cube,						// body that camera is attached to
		update_rate,					// update rate in Hz
		offset_sink_observer,			// offset pose
		// cam_birdview_left_pose,				// offset pose
		image_width,					// image width
		image_height,					// image height
		sink_observer_fov,				// camera's horizontal field of view
		// viewer_fov,                     // camera's horizontal field of view
		alias_factor,					// supersample factor for antialiasing
		lens_model,						// lens model
		false,							// use global illumination or not
		cam_integrator      			// integrator
	);
	wheel_cam_LB->SetName("wheel_cam_left_back");
	wheel_cam_LB->SetLag(lag);
	wheel_cam_LB->SetCollectionWindow(0.f); // would cause dynamic blur effect
	if (exposure_correction_switch == true) {
		wheel_cam_LB->PushFilter(chrono_types::make_shared<ChFilterCameraExposureCorrect>(
			expsr_a_arr[0], expsr_a_arr[1], expsr_a_arr[2],
			expsr_b_arr[0], expsr_b_arr[1], expsr_b_arr[2], expsr_time * 2.0f
		));
	}
	wheel_cam_LB->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
	if (vis) {
		wheel_cam_LB->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Wheel Camera Left-Back"));
	}
	if (save) {
		wheel_cam_LB->PushFilter(chrono_types::make_shared<ChFilterSave>(
			out_dir + "WheelCam_LeftBack_" + argv[1] + "_" + brdf_type + "_" + expsr_time_str.str() + "/"
		));
	}
	manager->AddSensor(wheel_cam_LB);

	// Add Left-Back wheel sinkage semantic segmenter
	auto wheel_segment_LB = chrono_types::make_shared<ChSegmentationCamera>(
		viper.GetChassis()->GetBody(),	// body camera is attached to
		update_rate,					// update rate in Hz
		offset_sink_observer,			// offset pose
		image_width,					// image width
		image_height,					// image height
		sink_observer_fov,				// camera's horizontal field of view
		lens_model						// lens model
	);
	wheel_segment_LB->SetName("wheel_segment_left_back");
	wheel_segment_LB->SetLag(lag);
	wheel_segment_LB->SetCollectionWindow(0.f); // would cause dynamic blur effect
	wheel_segment_LB->PushFilter(chrono_types::make_shared<ChFilterSemanticAccess>());
	if (vis) {
		wheel_segment_LB->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Wheel Segment Left-Back"));
	}
	if (save) {
		wheel_segment_LB->PushFilter(chrono_types::make_shared<ChFilterSave>(
			out_dir + "WheelSegment_LeftBack_" + argv[1] + "_" + brdf_type + "_" + expsr_time_str.str() + "/"
		));
	}
	manager->AddSensor(wheel_segment_LB);

	// Add front-end camera mounted on the Viper
	char front_cam_posi_idx = 'A';
	char front_cam_LR_idx = 'R';
	ChVector3f front_cam_posi = camera_posi_map[front_cam_posi_idx][front_cam_LR_idx];
	front_cam_posi.Mul(front_cam_posi, terrain_scale_ratio);
	front_cam_posi.x() += -3.3;
	front_cam_posi.z() += 0.30;
	front_cam_posi.y() += 0.50;
	chrono::ChFrame<double> front_cam_pose(
		front_cam_posi,
		QuatFromAngleAxis(30 * (CH_PI / 180), {0, 1, 0})
	);
	// chrono::ChFrame<double> offset_pose6( // debug
	//     ChVector3d({1.76913 + front_cam_posi.x(), -0.503446 + front_cam_posi.y(), 0.5 + front_cam_posi.z()}),
	//     QuatFromAngleAxis(CH_PI, {0, 0, 1}) * QuatFromAngleAxis(20 * (CH_PI / 180), {0, 1, 0}) 
	// );
	auto front_end_cam = chrono_types::make_shared<ChCameraSensor>(
		viper.GetChassis()->GetBody(), // body that camera is attached to
		// origin_cube,            // body that camera is attached to, debug
		update_rate,            // update rate in Hz
		front_cam_pose,         // offset pose
		// offset_pose6,           // offset pose, debug
		front_camera_width,     // image width
		front_camera_height,    // image height
		front_camera_fov,       // camera's horizontal field of view
		alias_factor,           // supersample factor for antialiasing
		lens_model,             // lens model
		false,					// use global illumination or not
		cam_integrator			// integrator
	);
	front_end_cam->SetName("Front-end Camera");

	front_end_cam->SetLag(lag);
	// cam->SetUpdateRate(1000.0f / float(exposure_time)); // [Hz]

	front_end_cam->SetCollectionWindow(0.0f); // [sec]
	if (exposure_correction_switch == true) {
		front_end_cam->PushFilter(chrono_types::make_shared<ChFilterCameraExposureCorrect>(
			expsr_a_arr[0], expsr_a_arr[1], expsr_a_arr[2],
			expsr_b_arr[0], expsr_b_arr[1], expsr_b_arr[2], expsr_time + 0.300f
		));
	}
	if (vis) {
		front_end_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
			front_camera_width, front_camera_height, "Front-end Camera"
		));
	}
	if (save) {
		front_end_cam->PushFilter(chrono_types::make_shared<ChFilterSave>(
			out_dir + "front_end_cam_" + argv[1] + "_" + brdf_type + "_" + expsr_time_str.str() + "/"
		));
	}
	manager->AddSensor(front_end_cam);

	// --------- //
	// ROS setup //
	// --------- //
	// Initialize image message format
	sensor_msgs::msg::Image sink_observer_msg;
	sink_observer_msg.header.frame_id	= wheel_cam_LB->GetName();						// camera's name
	sink_observer_msg.width				= wheel_cam_LB->GetWidth();
	sink_observer_msg.height			= wheel_cam_LB->GetHeight();
	sink_observer_msg.encoding			= "rgba8";										// RGBA, 8 bits per channel
	sink_observer_msg.is_bigendian		= false;
	sink_observer_msg.step				= sizeof(PixelRGBA8) * sink_observer_msg.width;	// 4 bytes per pixel (R, G, B, A)
	sink_observer_msg.data.resize(sink_observer_msg.step * sink_observer_msg.height);

	sensor_msgs::msg::Image sink_segment_msg;
	sink_segment_msg.header.frame_id	= wheel_segment_LB->GetName();						// camera's name
	sink_segment_msg.width				= wheel_segment_LB->GetWidth();
	sink_segment_msg.height				= wheel_segment_LB->GetHeight();
	sink_segment_msg.encoding			= "rgba8";										// RGBA, 8 bits per channel
	sink_segment_msg.is_bigendian		= false;
	sink_segment_msg.step				= sizeof(PixelRGBA8) * sink_segment_msg.width;	// 4 bytes per pixel (R, G, B, A)
	sink_segment_msg.data.resize(sink_segment_msg.step * sink_segment_msg.height);

	// Initialize ROS
	rclcpp::init(argc, argv);

	// Create the ROS node, passing in the initialized image
	auto node = std::make_shared<RGBA8PublisherNode>("chrono_node");
	node->AddImgPublisher(wheel_cam_LB->GetName());
	node->AddImgPublisher(wheel_segment_LB->GetName());

	// Explicit executor
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node);

	// // Create the Irrlicht visualization sys
	// auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();Speed
	// vis->AddCamera(ChVector3d(2.0, 0.0, 1.4), ChVector3d(0, 0, wheel_range));
	// vis->AddTypicalLights();
	// vis->AddLightWithShadow(ChVector3d(-5.0, -0.5, 8.0), ChVector3d(-1, 0, 0), 100, 1, 35, 85, 512,
	//                         ChColor(0.8f, 0.8f, 0.8f));
	// vis->EnableShadows();
	// std::chrono::high_resolution_clock::time_point tm1 = std::chrono::high_resolution_clock::now();

	// --------------- //
	// Simulation loop //
	// --------------- //

	ChTimer wall_timer; // [sec], wall time in real world
	float sim_time = 0.0; // [sec], simulation time in Chrono
	float cout_timer = 0.1; // [sec], time to print out something
	float orbit_radius = 100.0f;
	float orbit_rate = 0.5f;
	double max_steering = CH_PI / 3;
	double steering = 0;
	double publish_frame_time = (ROS_publish_rate == 0) ? 0 : (1 / ROS_publish_rate); // NOTE: If update_rate == 0, tick is called each time
	double time_elapsed_since_last_publish = 0;
	while (sim_time < sim_end_time) {
	// while (vis->Run()) {   

	//     vis->BeginScene();
	//     vis->GetActiveCamera()->setTarget(core::vector3dfCH(Body_1->GetPos()));
	//     vis->Render();
	//     tools::drawColorbar(vis.get(), 0, 20000, "Pressure yield [Pa]", 1180);
	//     vis->EndScene();# Make ssh dir
		
	sim_time = (float)sys.GetChTime();
		// std::cout << "Sim time: " << sim_time << " RTF: " << timer() / sim_time << std::endl; 
		if (sim_time > cout_timer) {
			// time_t timenow = time(NULL);
			auto timenow = std::chrono::system_clock::to_time_t(std::chrono::high_resolution_clock::now());
			char* time_str = ctime(&timenow);
			time_str[strlen(time_str) - 1] = '\0'; // remove the '\n' character
			std::cout << "[" << time_str << "] " << "Sim. time: " << sim_time << " [sec], RTF: " << wall_timer() / sim_time << std::endl;
			// std::cout << viper.GetChassis()->GetBody()->GetPos() << std::endl; // debug
			cout_timer += 0.1;
		}

		// std::chrono::high_resolution_clock::time_point tm1 = std::chrono::high_resolution_clock::now();
		
		//// steering policy ////
		// P-control
		double y_offset = viper_body->GetPos().y();
		steering = 1.0 * max_steering * std::max(-1.0, std::min(y_offset, 1.0));

		driver->SetSteering(steering); // debug, stop VIPER
		
		// Update image message data and publish out
		sink_observer_msg.header.stamp = GetROSTimestamp(sim_time);
		sink_segment_msg.header.stamp = GetROSTimestamp(sim_time);
		// img_msg.data.assign(rgba8_image, rgba8_image + img_msg.step * img_msg.height); // debug

		UserRGBA8BufferPtr rgba8_buffer_ptr = wheel_cam_LB->GetMostRecentBuffer<UserRGBA8BufferPtr>(); // start of PixelRGBA8[]
		UserSemanticBufferPtr segment_buffer_ptr = wheel_segment_LB->GetMostRecentBuffer<UserSemanticBufferPtr>(); // start of PixelSemantic[]
		if (rgba8_buffer_ptr && rgba8_buffer_ptr->Buffer) {
			const uint8_t* rgba8_ptr = reinterpret_cast<const uint8_t*>(reinterpret_cast<const PixelRGBA8*>(rgba8_buffer_ptr->Buffer.get()));
			sink_observer_msg.data.assign(rgba8_ptr, rgba8_ptr + sink_observer_msg.step * sink_observer_msg.height);
		}
		if (segment_buffer_ptr && segment_buffer_ptr->Buffer) {
			const uint8_t* segment_ptr = reinterpret_cast<const uint8_t*>(reinterpret_cast<const PixelSemantic*>(segment_buffer_ptr->Buffer.get()));
			sink_segment_msg.data.assign(segment_ptr, segment_ptr + sink_segment_msg.step * sink_segment_msg.height);
		}
    	
		time_elapsed_since_last_publish += sim_time_step;
		if (time_elapsed_since_last_publish > publish_frame_time) {
			node->PublishImgMsg(wheel_cam_LB->GetName(), sink_observer_msg);
			node->PublishImgMsg(wheel_segment_LB->GetName(), sink_segment_msg);
			time_elapsed_since_last_publish -= publish_frame_time;
		}
		
		//// Update states
		if (!rclcpp::ok()) break;
		wall_timer.start();
		if (debug_terrain == false) {
			sys.DoStepDynamics(2.5e-4);
			// sys.DoStepDynamics(2.5e-3); // debug
		}
		wall_timer.stop();
		manager->Update();
		viper.Update(); // debug, stop VIPER
		

		// std::chrono::high_resolution_clock::time_point tm2 = std::chrono::high_resolution_clock::now();
		// std::chrono::duration<double> wall_time_m = std::chrono::duration_cast<std::chrono::duration<double>>(tm2 - tm1);
		// std::cout << "wall time: " << wall_time_m.count() << "s.\n";

		////terrain.PrintStepStatistics(std::cout);
	}
	rclcpp::shutdown();

	return 0;
}
