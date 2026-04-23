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
// Demo to show VIPER operated in a digital twin of a real terrain built in the lab
//
// =============================================================================
#include <iostream>
// #include <ctime>
#include <time.h> 
#include <cstring>
// #include <sstream>

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChVisualShapeModelFile.h"

#include "chrono/geometry/ChTriangleMeshConnected.h"


#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

// #include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using namespace chrono;
using namespace chrono::irrlicht;
// using namespace chrono::geometry;
// using namespace chrono::viper;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::sensor;
// using namespace irr;


// ---- General camera parameters ---- //
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
float update_rate = 30;		// [Hz], update rate
float cam_lag = 0.f;		// [sec], lag time between Camera sensing and when data becomes accessible
int alias_factor = 2;		// number of rays per pixel
unsigned int cam_h = 1536;	// [px]
unsigned int cam_w = 2048;	// [px]
float cam_hFOV = 1.12781;	// [rad]

unsigned int viewer_h = 1080;			// [px], viewer height
unsigned int viewer_w = 1920;			// [px], viewer width
float viewer_hFOV = (float)CH_PI / 2.;	// [rad], viewer's horizontal field of view (hFOV)

// ---- Dynamics solver parameters ---- //
float sim_end_time = 24.5f;		// [sec], simulation end time
// float dynamics_step = 2e-4;		// [sec], simulation step size
float dynamics_step = 2.5e-3;	// [sec], simulation step size

// ---- Scene setting ---- //
double terrain_scale = 24.0;
// ChVector3d village_offset({0.00, -0.015, -0.095});
ChVector3d village_offset({0., 0., -0.003}); // [m]
// ChVector3d village_offset({0., 0., 0.}); // [m]
ChVector3d init_loc(0.30 + village_offset.x(), 0.42 + village_offset.y(), 0.07 + village_offset.z()); // [m], initial location of HMMWV
ChColor ground_albedo(0.55, 0.35, 0.20); // [R,G,B], albedo of the ground

// ---- Paths ---- //
const std::string out_dir			= "HMMWV_MystVillage/"; // output folder for saved images
const std::string env_light_dir		= "/home/bohsun/UW_Madison/Research/DiffPhysCam_Data/NovelViewSynthesis_Data/SimExperiment/envmaps/";
const std::string village_mesh_dir	= "/home/bohsun/UW_Madison/Research/DiffPhysCam_Data/NovelViewSynthesis_Data/SimExperiment/mesh/";

// ---- Switch setting ---- //
bool save = false; // Save camera and viewer images
bool vis = false; // visualize camera
bool use_diffuse = false;  // whether Camera consinders diffuse reflection
bool use_denoiser = false; // whether Camera 1 uses the OptiX denoiser
bool debug_terrain = false; // whether just check the terrain and stop running rover
bool tune_cam_focal_length = false; // whether tuning the camera focal length

/// @brief Change all diffuse colors of the visual materials on a body to
/// @param body 
/// @param color 
void SetAllDiffuseColorsOnBody(std::shared_ptr<chrono::ChBody> body, float amp_rate) {
    if (!body)
        return;

    auto vis_model = body->GetVisualModel();
    if (!vis_model)
        return;

    for (size_t is = 0; is < vis_model->GetNumShapes(); ++is) {
        auto shape = vis_model->GetShape((unsigned int)is);
        if (!shape)
            continue;

        size_t num_mats = shape->GetNumMaterials();
        for (size_t im = 0; im < num_mats; ++im) {
            auto mat = shape->GetMaterial(im);
            if (!mat) {
                mat = chrono_types::make_shared<chrono::ChVisualMaterial>();
                shape->SetMaterial(im, mat);
            }

            ChColor albedo = mat->GetDiffuseColor();
            mat->SetDiffuseColor(
                {amp_rate * albedo.R, amp_rate * albedo.G, amp_rate * albedo.B}
            );
        }
    }
}


int main(int argc, char* argv[]) {
	if (argc < 3) {
		std::cout << "./demo_SEN_HMMWV_SCM_MystVillage <number of samples per pixel> <Sun_angle> <env_light_scale> --use_diffuse --use_denoiser --save --vis\n";
		exit(1);
	}

	std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
	
	alias_factor = (int)sqrt(std::atof(argv[1]));
	std::string sun_angle = argv[2];
	float env_light_scale = std::atof(argv[3]);
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--use_diffuse") == 0) {
			use_diffuse = true;
			std::cout << "\nCamera considers diffuse reflection\n";
		}
		else if (std::strcmp(argv[i], "--use_denoiser") == 0) {
			use_denoiser = true;
			std::cout << "\nCamera uses OptiX denoiser\n";
		}
		else if (std::strcmp(argv[i], "--save") == 0) {
			save = true;
			std::cout << "\nSave images\n";
		}
		else if (std::strcmp(argv[i], "--vis") == 0) {
			vis = true;
			std::cout << "\nVisualize the camera\n";
		}
	}

	// Define environment light
	std::string env_light_path = env_light_dir + "envmap_" + sun_angle + "_045_nvDiffRec_exp.exr";


	/*
	//// Define different postion of the sun ////
	float sun_height = 1000 * tanf(45 * CH_PI/180.0); // [m]

	auto pos1 = ChVector3d( 1000.0, 0.0, sun_height); // [m], azi = 0 deg
	auto pos2 = ChVector3d( 1000.0 * cosf(45 * CH_PI/180.0), 1000.0 * sinf(45 * CH_PI/180.0), sun_height); // [m], azi = 315 deg
	auto pos3 = ChVector3d( 1000.0 * cosf(135 * CH_PI/180.0), 1000.0 * sinf(135 * CH_PI/180.0), sun_height); // [m], azi = 225 deg
	auto pos4 = ChVector3d(-1000.0, 0.0, sun_height); // [m], azi = 180 deg
	auto sun_pose = ChVector3d{0.0, 0.0, 0.0};
	auto sun_color = ChColor(0., 0., 0.);
	auto ambient_color = ChVector3d(0., 0., 0.);
	switch (std::atoi(argv[1])) {
		case 1:
			sun_pose = pos1;
			// sun_color = ChColor({4.88 * 255/255.0, 4.88 * 228/255.0, 4.88 * 206/255.0});
			sun_color = ChColor({4.88, 4.88, 4.88});
			ambient_color.Set(0.476, 0.476, 0.476);
			break;
		case 2:
			sun_pose = pos2;
			// sun_color = ChColor({4.88 * 255/255.0, 4.88 * 228/255.0, 4.88 * 206/255.0});
			sun_color = ChColor({4.88, 4.88, 4.88});
			ambient_color.Set(0.476, 0.476, 0.476);
			break;
		case 3:
			sun_pose = pos3;
			// sun_color = ChColor({1.22 * 255/255.0, 1.22 * 228/255.0, 1.22 * 206/255.0});
			sun_color = ChColor({1.22, 1.22, 1.22});
			ambient_color.Set(0.119, 0.119, 0.119);
			break;
		case 4:
			sun_pose = pos4;
			// sun_color = ChColor({1.22 * 255/255.0, 1.22 * 228/255.0, 1.22 * 206/255.0});
			sun_color = ChColor({1.22, 1.22, 1.22});
			ambient_color.Set(0.119, 0.119, 0.119);
			break;
		default:
			std::cout << "unknown Sun position ID ...";
			exit(1);
	}
	std::cout << "Sun posi: " << sun_pose << std::endl;
	*/

	// --------------------- //
	// Create vehicle system //
	// --------------------- //
	HMMWV_Full my_hmmwv;
	my_hmmwv.SetContactMethod(ChContactMethod::NSC);
	my_hmmwv.SetChassisCollisionType(CollisionType::NONE);
	my_hmmwv.SetChassisFixed(false);
	my_hmmwv.SetInitPosition(ChCoordsys<>(
		{init_loc.x() * terrain_scale, init_loc.y() * terrain_scale, init_loc.z() * terrain_scale}, // [m]
		QuatFromAngleAxis(-CH_PI/2, {0., 0, 1.}) // [quaternion]
	));
	my_hmmwv.SetEngineType(EngineModelType::SIMPLE);
	my_hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
	my_hmmwv.SetDriveType(DrivelineTypeWV::RWD);
	my_hmmwv.SetSteeringType(SteeringTypeWV::RACK_PINION);
	my_hmmwv.SetTireType(TireModelType::RIGID);
	my_hmmwv.SetTireStepSize(dynamics_step);
	my_hmmwv.Initialize();

	my_hmmwv.SetChassisVisualizationType(VisualizationType::MESH);
	my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
	my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
	my_hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
	my_hmmwv.SetTireVisualizationType( VisualizationType::MESH);
	
	SetAllDiffuseColorsOnBody(my_hmmwv.GetChassisBody(), 0.20);

	std::vector<ChVector3d> via_points = {
		terrain_scale * init_loc,
		terrain_scale * ChVector3d( 0.25, -0.02, 0.),
		terrain_scale * ChVector3d(-0.04, -0.11, 0.),
		terrain_scale * ChVector3d(-0.26, -0.04, 0.),
		// terrain_scale * ChVector3d(-0.52,  0.20, 0.),
		terrain_scale * ChVector3d(-0.35,  0.30, 0.),
	};
	std::vector<ChVector3d> in_CVs = {
		terrain_scale * ChVector3d( 0.27,  0.51, 0.),
		terrain_scale * ChVector3d( 0.33,  0.08, 0.),
		terrain_scale * ChVector3d( 0.05, -0.02, 0.),
		terrain_scale * ChVector3d(-0.20, -0.11, 0.),
		// terrain_scale * ChVector3d(-0.40,  0.20, 0.),
		terrain_scale * ChVector3d(-0.47,  0.18, 0.),
	};
	std::vector<ChVector3d> out_CVs = {
		terrain_scale * ChVector3d( 0.34,  0.29, 0.),
		terrain_scale * ChVector3d( 0.17, -0.12, 0.),
		terrain_scale * ChVector3d(-0.11, -0.18, 0.),
		terrain_scale * ChVector3d(-0.39,  0.11, 0.),
		// terrain_scale * ChVector3d(-0.68,  0.20, 0.),
		terrain_scale * ChVector3d(-0.27,  0.38, 0.),
	};

	auto path = chrono_types::make_shared<ChBezierCurve>(via_points, in_CVs, out_CVs, false);

	ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), path, "my_path", 2.0);
	driver.GetSteeringController().SetLookAheadDistance(4);
	driver.GetSteeringController().SetGains(2.4, 0, 0);
	driver.GetSpeedController().SetGains(1.2, 0, 0);
	driver.Initialize();

	// --------------------------------- //
	// Nonresponsive and visible objects //
	// --------------------------------- //
	// Create object body
	auto village_body = chrono_types::make_shared<ChBodyAuxRef>();
	
	// Set up object mesh
	auto village_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
	auto village_mesh_loader = ChTriangleMeshConnected::CreateFromWavefrontFile(village_mesh_dir + "mesh.obj", true, true);
	village_mesh_loader->Transform({0., 0., 0.}, ChMatrix33<>(terrain_scale)); // scale to a different size
	
	village_mesh_loader->RepairDuplicateVertexes(1e-9); // if meshes are not watertight
	village_mesh->SetMesh(village_mesh_loader);
	village_mesh->SetBackfaceCull(true);
	
	// Set up object appearance
	auto village_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
	// village_vis_mat->SetBSDF(BSDFType::PRINCIPLED);
	village_vis_mat->SetKdTexture(village_mesh_dir + "albedo.exr");
	village_vis_mat->SetUseSpecularWorkflow(false);
	village_vis_mat->SetRoughnessTexture(village_mesh_dir + "rough.exr"); 
	village_vis_mat->SetMetallicTexture(village_mesh_dir + "metal.exr");
	village_vis_mat->SetNormalMapTexture(village_mesh_dir + "normal.exr");

	village_body->AddVisualShape(village_mesh);
	village_body->GetVisualShape(0)->SetMaterial(0, village_vis_mat);
	
	// Register object to system
	village_body->SetFixed(true);
	village_body->SetPos(terrain_scale * village_offset); // [m]
	village_body->SetRot(QuatFromAngleAxis(CH_PI/2, {1., 0, 0.}));
	my_hmmwv.GetSystem()->Add(village_body);

	// Create a fixed origin cube to attach camera
	auto origin_cube = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
	origin_cube->SetPos({0., 0., 0.});
	origin_cube->SetFixed(true);
	my_hmmwv.GetSystem()->Add(origin_cube);

	// Create a moving cube to attach camera
	auto moving_cube = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
	moving_cube->SetPos(init_loc);
	moving_cube->SetRot(QuatFromAngleAxis(-CH_PI/2, {0., 0, 1.}));
	moving_cube->SetFixed(false);
	my_hmmwv.GetSystem()->Add(moving_cube);

	// ------------ //
	// Rigid ground //
	// ------------ //
	RigidTerrain ground(my_hmmwv.GetSystem());
	auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
	patch_mat->SetFriction(0.9f);
	patch_mat->SetRestitution(0.01f);

	auto patch = ground.AddPatch(
		patch_mat,
		ChCoordsys<>(ChVector3d(0, 0., 0), QUNIT),
		village_mesh_dir + "experiment_ground.obj"                          
	);
	patch->SetColor(ground_albedo);
	// patch->SetTexture(ground_mesh_dir + "albedo.png", 1.0, 1.0);

	ground.Initialize();
	

	// -------------------- //
	// Sensor manager setup //
	// -------------------- //

	auto manager = chrono_types::make_shared<ChSensorManager>(my_hmmwv.GetSystem());
	manager->SetRayRecursions(6);
	manager->SetVerbose(false);
	manager->scene->SetAmbientLight({0.f, 0.f, 0.f});

	// Set up background
	Background b;
	b.mode = BackgroundMode::ENVIRONMENT_MAP;
	b.env_tex = env_light_path;

	manager->scene->SetBackground(b);
	manager->Update();
	int light_idx = manager->scene->AddEnvironmentLight(b.env_tex, env_light_scale);

	// ------------------------- //
	// Cameras and viewers setup //
	// ------------------------- //    
	
	// ---- Set up left bird-view camera ---- //
	chrono::ChFrame<double> bird_viewer_pose(
		{terrain_scale, 0, terrain_scale},
		QuatFromAngleAxis(CH_PI, {0, 0, 1}) * QuatFromAngleAxis(45 * (CH_PI/180), {0, 1, 0})
	);
	auto bird_viewer = chrono_types::make_shared<ChCameraSensor>(
		// viper.GetChassis()->GetBody(),  // body camera is attached to
		origin_cube,		// body that camera is attached to
		update_rate,		// update rate in Hz
		bird_viewer_pose,	// offset pose
		viewer_w,			// image width
		viewer_h,			// image height
		viewer_hFOV,		// camera's horizontal field of view
		alias_factor,		// supersample factor for antialiasing
		lens_model,			// lens model
		use_diffuse,		// whether consider diffuse reflection. If false, then only considers specular reflection
		use_denoiser,		// whether use OptiX denoiser for diffuse reflection or area lights
		Integrator::PATH,	// integrator algorithm for rendering
		2.2f,				// gamma correction
		false				// whether to use fog 
	);
	bird_viewer->SetName("Bird Viewer");
	bird_viewer->SetLag(cam_lag);
	bird_viewer->SetCollectionWindow(0.0f); // would cause dynamic blur effect
	
	// ---- Mount front-end camera on rover ---- //
	// chrono::ChFrame<double> front_cam_pose({-2.4, -0.8, 5.0}, QuatFromAngleAxis(15 * (CH_PI / 180), {0, 1, 0}));
	float degree = 20;
	chrono::ChFrame<double> front_cam_pose(
		{-terrain_scale * cosf(degree * CH_PI/180) , 0., terrain_scale * sinf(degree * CH_PI/180)},
		QuatFromAngleAxis(degree * (CH_PI / 180), {0, 1, 0})
	);
	
	auto front_cam = chrono_types::make_shared<ChCameraSensor>(
		moving_cube,		// body camera is attached to
		update_rate,		// update rate in Hz
		front_cam_pose,		// offset pose
		cam_w,				// image width
		cam_h,				// image height
		cam_hFOV,			// camera's horizontal field of view (hFOV)
		alias_factor,		// squared root of samples per pixel
		lens_model,			// lens model type
		use_diffuse,		// whether consider diffuse reflection. If false, then only considers specular reflection
		use_denoiser,		// whether use OptiX denoiser for diffuse reflection or area lights
		Integrator::PATH,	// integrator algorithm for rendering
		1.f,				// gamma correction
		false				// whether to use fog 
	);
	front_cam->SetName("Front Camera");
	front_cam->SetLag(cam_lag);
	front_cam->SetCollectionWindow(0.f);

	// ---- Create depth camera ---- //
	auto depth_cam = chrono_types::make_shared<ChDepthCamera>(
		moving_cube,	// body camera is attached to
		update_rate,	// update rate, [Hz]
		front_cam_pose,	// offset pose
		cam_w,			// image width, [px]
		cam_h,			// image height, [px]
		cam_hFOV,		// camera's horizontal field of view (hFOV)
		1e6f,			// maximum depth value, [m]
		lens_model		// lens model, for calibration
	);
	depth_cam->SetName("Depth Cam");
    depth_cam->SetLag(cam_lag);
    depth_cam->SetCollectionWindow(0.f);

	
	// Visualize data
	if (vis) {
		bird_viewer->PushFilter(chrono_types::make_shared<ChFilterVisualize>(viewer_w, viewer_h, "Bird Viewer"));
		front_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(cam_w, cam_h, "Front Camera"));
		depth_cam->PushFilter(chrono_types::make_shared<ChFilterDepthToRGBA8>(depth_cam->GetMaxDepth()));
        depth_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(cam_w, cam_h, "Depth Cam"));
	}
	// Save data
	if (save) {
		bird_viewer->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "sun_" + sun_angle + "_BirdViewer/"));
		front_cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "sun_" + sun_angle + "_FrontCam/"));
		depth_cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "sun_" + sun_angle + "_DepthCam/"));
	}
	manager->AddSensor(bird_viewer);
	manager->AddSensor(front_cam);
	manager->AddSensor(depth_cam);
	
	
	// Create the Irrlicht visualization for debugging
	// auto irrl_vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
	// irrl_vis->AttachSystem(my_hmmwv.GetSystem());
	// irrl_vis->SetWindowSize(1280, 720);
	// irrl_vis->SetWindowTitle("Irrlicht viz");
	// irrl_vis->Initialize();
	// irrl_vis->AddCamera({12, 0, 12});
	// irrl_vis->AddTypicalLights();

	ChTimer timer; // [sec], wall time in real world
	double ch_time = 0.0; // [sec], simulation time in Chrono
	float cout_timer = 0.1f; // [sec], simulation time to print out something
	// int cin_counter = 0;
	// double max_steering = CH_PI / 3;
	// double steering = 0;
	
	my_hmmwv.Advance(dynamics_step);
	moving_cube->SetPos(my_hmmwv.GetChassisBody()->GetPos());
	moving_cube->SetRot(QuatFromAngleAxis(my_hmmwv.GetChassisBody()->GetRot().GetCardanAnglesXYZ().z(), {0., 0, 1.}));
			
	// float focal_length_increase_rate = 3.75e-5f / 2;
	// float reverse = 1.0;
	// float focal_length_init = front_cam_f;
	// float focal_length_increase_stop = 0.016;
	// bool stop_take_picture = false;
	while (ch_time < sim_end_time) {
	// while (1) {
	// while (irrl_vis->Run()) {
		// irrl_vis->BeginScene();
		// irrl_vis->Render();
		// irrl_vis->EndScene();

		ch_time = my_hmmwv.GetSystem()->GetChTime();
		// std::cout << "Sim time: " << ch_time << " RTF: " << timer() / ch_time << std::endl; 
		if (ch_time > cout_timer && tune_cam_focal_length == false) {
			auto timenow = std::chrono::system_clock::to_time_t(std::chrono::high_resolution_clock::now());
			char* time_str = ctime(&timenow);
			time_str[strlen(time_str) - 1] = '\0'; // remove the '\n' character
			std::cout << "[" << time_str << "] " << "Sim. time: " << ch_time << " [sec], RTF: " << timer() / ch_time << std::endl;
			// std::cout << (my_hmmwv.GetChassisBody()->GetFrameRefToAbs() * front_cam_pose) << std::endl;
			
			cout_timer += 0.1f;
		}
		
		timer.start();
		
		if (debug_terrain == false && tune_cam_focal_length == false) {
			// Driver inputs
			DriverInputs driver_inputs = driver.GetInputs();

			// Update modules (process inputs from other modules)
			driver.Synchronize(ch_time);
			ground.Synchronize(ch_time);
			my_hmmwv.Synchronize(ch_time, driver_inputs, ground);


			// Advance simulation for one timestep for all modules
			driver.Advance(dynamics_step);
			ground.Advance(dynamics_step);
			my_hmmwv.Advance(dynamics_step);
			moving_cube->SetPos(my_hmmwv.GetChassisBody()->GetPos());
			moving_cube->SetRot(QuatFromAngleAxis(my_hmmwv.GetChassisBody()->GetRot().GetCardanAnglesXYZ().z(), {0., 0, 1.}));
			// sys.DoStepDynamics(dynamics_step);
		}
		
		/*
		if (tune_cam_focal_length == true) {
			my_hmmwv.SetChassisFixed(true);
			// if (cin_counter > cin_duration) {
			//     std::cout << "focal length [m]: ";
			//     std::cin >> front_cam_f ;
			//     front_cam->SetCtrlParameters(aperture_num, expsr_time, ISO, front_cam_f, focus_dist);

			//     cin_counter = 0;
			// }
			// ++cin_counter;
			front_cam->SetCtrlParameters(aperture_num, expsr_time, ISO, front_cam_f, focus_dist);
			front_cam_f += reverse * focal_length_increase_rate;
			if (front_cam_f > focal_length_increase_stop)
				reverse = -1.0f;
			else if (front_cam_f < focal_length_init)
				reverse =  0.f;
		}
		*/

		manager->Update();

		timer.stop();

		// std::chrono::high_resolution_clock::time_point tm2 = std::chrono::high_resolution_clock::now();
		// std::chrono::duration<double> wall_time_m = std::chrono::duration_cast<std::chrono::duration<double>>(tm2 - tm1);
		// std::cout << "wall time: " << wall_time_m.count() << "s.\n";
	}

	return 0;
}
