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
// ????
//
// =============================================================================
#include <iostream>
// #include <ctime>
#include <time.h> 
#include <cstring>
// #include <sstream>

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/assets/ChVisualShapeModelFile.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/utils/ChSensorUtils.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using namespace chrono;
using namespace chrono::sensor;
// using namespace irr;


//// General camera parameters ////
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
float cam_hFOV = 1.12781;	// [rad]
float update_rate = 30;		// [Hz], update rate
float cam_lag = 0.f;		// [sec], lag time between Camera sensing and when data becomes accessible
int alias_factor = 2;		// number of rays per pixel (spp)
unsigned int cam_h = 1536;	// [px]
unsigned int cam_w = 2048;	// [px]

//// Dynamics solver parameters ////
float end_time = 20.0f;			// [sec], simulation end time
const int cin_duration = 25;	// number of duration steps for next cin
double step_size = 0.01;		// [sec], Simulation step size

// ---- Switch setting ---- //
bool save = false; // Save camera and viewer images
bool viz = false; // visualize camera

bool use_diffuse = false;  // whether Camera consinders diffuse reflection
bool use_denoiser = false; // whether Camera 1 uses the OptiX denoiser

// ---- Paths ---- //
const std::string village_mesh_dir		= "/home/bohsun/UW_Madison/Research/DiffPhysCam_Data/NovelViewSynthesis_Output/RealScene01_wo_defocus/mesh/";
const std::string env_light_dir			= "/home/bohsun/UW_Madison/Research/DiffPhysCam_Data/NovelViewSynthesis_Data/RealScene01/envmaps/";
const std::string trnsfrm_table_path	= "/home/bohsun/UW_Madison/Research/DiffPhysCam_Data/NovelViewSynthesis_Data/RealScene01/trnsfrms_and_configs_train_wo_ground.json";
const std::string out_dir				= "LuminanceCali_RealScene01/"; // output folder for saved images

const int num_trnsfrms = 234 ; // number of poses


// Copied from ChUtilsJSON.cpp, read json file into a variable
void ReadFileJSON(const std::string& filename, rapidjson::Document& d) {
	std::ifstream ifs(filename);
	if (!ifs.good()) {
		std::cout << "ERROR: Could not open JSON file: " << filename << "\n";
	}
	else {
		rapidjson::IStreamWrapper isw(ifs);
		d.ParseStream<rapidjson::ParseFlag::kParseCommentsFlag>(isw);
		if (d.IsNull()) {
			std::cout << "ERROR: Invalid JSON file: " << filename << "\n";
		}
	}
}


int main(int argc, char* argv[]) {

	std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
	
	// ------------------ //
	// Argument operation //
	// ------------------ //
	
	if (argc < 3) {
		std::cout << "Please assign the number of samples per pixel (spp) and Sun angle to illuminate the scene." << std::endl;
		std::cout << "And optionally, you can decide whether Camera considers diffuse reflection and attach the OptiX denoisers,";
		std::cout << "following the command line as:" << std::endl;
		std::cout << "demo_SEN_LuminanceCali <number of samples per pixel> <Sun_angle> ";
		std::cout<< "--use_diffuse --use_denoiser"<< std::endl;
		exit(1);
	}

	alias_factor = (int)sqrt(std::atof(argv[1]));
	std::string sun_angle = argv[2];

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
		else if (std::strcmp(argv[i], "--viz") == 0) {
			viz = true;
			std::cout << "\nVisualize the camera\n";
		}
	}
	
	//// Define environment light ////
	std::string env_light_path = env_light_dir + "envmap_" + sun_angle + "_045_nvDiffRec_exp.exr";
	float env_light_scale = std::atof(argv[3]); // Principled
	// float env_light_scale = 0.0f; // SimplePrincipled
	

	// ----------------- //
	// Create the system //
	// ----------------- //
	ChSystemNSC sys;
	
	// ----------- //
	// Scene setup //
	// ----------- //
	// Create object body
	auto village_body = chrono_types::make_shared<ChBodyAuxRef>();
	
	// Set up object mesh
	auto village_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
	auto village_mesh_loader = ChTriangleMeshConnected::CreateFromWavefrontFile(village_mesh_dir + "mesh.obj", true, true);
	village_mesh_loader->Transform({0., 0., 0.}, ChMatrix33<>(1.0)); // scale to a different size
	
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
	village_body->SetPos({0., 0., 0.}); // [m]
	village_body->SetRot(QuatFromAngleAxis(CH_PI/2, {1., 0, 0.}));
	sys.Add(village_body);

	// create a fixed origin cube to attach camera
	auto origin_cube = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
	origin_cube->SetPos({0., 0., 0.});
	origin_cube->SetFixed(true);
	sys.Add(origin_cube);


	// ------------------------ //
	// Background manager setup //
	// ------------------------ //
	auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
	manager->SetRayRecursions(6);
	manager->SetVerbose(false);
	manager->scene->SetAmbientLight({0.f, 0.f, 0.f});
	
	Background b;
	b.mode = BackgroundMode::ENVIRONMENT_MAP;
	b.env_tex = env_light_path;

	manager->scene->SetBackground(b);
	manager->Update();
	int light_idx = manager->scene->AddEnvironmentLight(b.env_tex, env_light_scale);


	// ------------ //
	// Sensor setup //
	// ------------ //    
	
	// The camera
	chrono::ChFrame<double> cam_pose({0, 0, 0}, QUNIT);
	// auto cam = chrono_types::make_shared<ChPhysCameraSensor>(
    //     origin_cube,		// body camera is attached to
    //     update_rate,		// update rate, [Hz]
    //     cam_pose,			// offset pose
    //     cam_w,				// image width, [px]
    //     cam_h,				// image height, [px]
    //     lens_model,			// lens model to use
    //     alias_factor,		// super sampling factor for antialiasing
    //     use_diffuse,		// whether consider diffuse reflection
    //     use_denoiser,		// whether use OptiX denoiser
    //     false,				// whether to activate defocus blur
    //     false,				// whether to activate vignetting
    //     false,				// whether to aggregate illumination irradiance
    //     false,				// whether to add noises
    //     false,				// whether to convert exposure to digital values
    //     Integrator::PATH,	// integrator algorithm for rendering
    //     1.f,				// gamma correction, 1.0 for linear color space, 2.2 for sRGB
    //     false,				// whether to use fog on this camera
    //     false				// whether to use motion blur effect
    // );
	// PhysCameraGainParams phys_cam_gain_params;
    // PhysCameraNoiseParams phys_cam_noise_params;
	// float cam_focal_length = 0.005f; // [m]
	// float cam_sensor_width = 2 * cam_focal_length * tanf(cam_hFOV / 2);  // [m]
	// cam->SetCtrlParameters(0., 0., 0., cam_focal_length, 0.);
    // cam->SetModelParameters(cam_sensor_width, 0., 0., {0.f, 0.f, 0.f}, phys_cam_gain_params, phys_cam_noise_params);

	auto cam = chrono_types::make_shared<ChCameraSensor>(
		origin_cube,		// body camera is attached to
		update_rate,		// update rate in Hz
		cam_pose,			// offset pose
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
	cam->SetName("Camera");
	cam->SetLag(cam_lag);
	cam->SetCollectionWindow(0.f);
	

	if (viz) {
		cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(cam_w, cam_h, (use_denoiser) ? "Path Integrator Denoised" : "Path Integrator"));
	}

	std::shared_ptr<ChFilterSave> save_cam_filter_ptr = chrono_types::make_shared<ChFilterSave>(out_dir + "Cam/");
	if (save) {
		cam->PushFilter(save_cam_filter_ptr);
	}
	
	manager->AddSensor(cam);

	// Create the Irrlicht visualization for debugging
	// auto irrl_vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
	// irrl_vis->AttachSystem(my_hmmwv.GetSystem());
	// irrl_vis->SetWindowSize(1280, 720);
	// irrl_vis->SetWindowTitle("Irrlicht viz");
	// irrl_vis->Initialize();
	// irrl_vis->AddCamera({12, 0, 12});
	// irrl_vis->AddTypicalLights();

	rapidjson::Document trnsfrm_table;
	
	ReadFileJSON(trnsfrm_table_path, trnsfrm_table);
	std::string trnsfrm_name;
	
	// Iterate over all transforms to generate all simulated radiances
	for (size_t trnsfrm_idx = 0; trnsfrm_idx < num_trnsfrms; trnsfrm_idx += 2) {    
		
		rapidjson::Value& trnsfrm_params = trnsfrm_table["frames"][std::to_string(trnsfrm_idx).c_str()];
		trnsfrm_name = trnsfrm_params["img_name"].GetString();
		trnsfrm_name = trnsfrm_name.substr(0, trnsfrm_name.size() - 11);
		if (trnsfrm_name.substr(4, 3) != sun_angle) {
			continue;
		}

		// Read the transform matrix
		std::vector<std::vector<double>> trnsfrm_vec;
		const auto& trnsfrm = trnsfrm_params["trnsfrm"];
		for (auto& row : trnsfrm.GetArray()) {
			std::vector<double> matrix_row;
			for (auto& value : row.GetArray()) {
				matrix_row.push_back(value.GetDouble());
			}
			trnsfrm_vec.push_back(matrix_row);
		}

		cam_pose.SetPos({trnsfrm_vec[0][3], trnsfrm_vec[1][3], trnsfrm_vec[2][3]});
		ChMatrix33<double> cam_ori(
			ChVector3d({trnsfrm_vec[0][0], trnsfrm_vec[1][0], trnsfrm_vec[2][0]}),
			ChVector3d({trnsfrm_vec[0][1], trnsfrm_vec[1][1], trnsfrm_vec[2][1]}),
			ChVector3d({trnsfrm_vec[0][2], trnsfrm_vec[1][2], trnsfrm_vec[2][2]})
		);
		// cam_pose.SetRot(cam_ori);
		chrono::ChFrame<double> temp({0, 0, 0,}, cam_ori);
		cam_pose.SetRot(temp.GetRot() * QuatFromAngleAxis(-CH_PI/2, {0, 1, 0}) * QuatFromAngleAxis(-CH_PI/2, {1, 0, 0}));
		
		cam->SetOffsetPose(cam_pose);

		auto timenow = std::chrono::system_clock::to_time_t(std::chrono::high_resolution_clock::now());
		char* time_str = ctime(&timenow);
		time_str[strlen(time_str) - 1] = '\0'; // remove the '\n' character
		std::cout << "[" << time_str << "] Transform Idx " << trnsfrm_idx << ", " << trnsfrm_name << std::endl;
		
		save_cam_filter_ptr->ChangeDataPath(out_dir + "Cam/" + trnsfrm_name + "/");
		
		// Run simulation to take radiances
		for (int cin_count = 0; cin_count < cin_duration; ++cin_count) {
			manager->Update(); // Update sensor manager. Will render/save/filter automatically
			sys.DoStepDynamics(step_size); // Perform step of dynamics
		}
	}

	return 0;
}
