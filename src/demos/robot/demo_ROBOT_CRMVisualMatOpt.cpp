// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/optix/ChNVDBVolume.h"

// #include <cuda.h>
// #include <curand_kernel.h>
// #include <thrust/random.h>
// #include <thrust/device_ptr.h>
// #include <thrust/transform.h>
// #include <thrust/system/cuda/execution_policy.h>
// #include <thrust/system/omp/execution_policy.h>
// #include <thrust/extrema.h>
// #include <thrust/device_vector.h>
// #include <thrust/distance.h>

// #include "genetic_algo_HPC.cuh"

#ifdef USE_SENSOR_NVDB
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/ParticlesToLevelSet.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
#include <execution>
#endif

#include <time.h>
#include <iomanip>
#include <sstream>
#include <filesystem>

#include <random>
std::mt19937 rng(5678); // Mersenne Twister with seed 1234
std::uniform_real_distribution<float> UnifDist(0.0f, 1.0f);

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
double bxDim = 3.0;
double byDim = 2.0;
double bzDim = 0.1;

// Rover initial location
// ChVector3d init_loc(1.0 - bxDim / 2.0 + .25f, 0, bzDim + 0.4);
ChVector3d init_loc(0, 0, 2.1);

// Simulation time and stepsize
// double total_time = 20.0 / 3;
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
// double wheel_radius = 0.25;
// double wheel_wide = 0.2;
// double grouser_height = 0.025;
// double grouser_wide = 0.005;
// int grouser_num = 24;
// double wheel_AngVel = 0.8;

std::shared_ptr<Viper> rover; // Pointer to store the VIPER instance
std::shared_ptr<ViperSpeedDriver> driver; // Pointer to store the VIPER driver

// ---- Scene setup parameters ---- //
// std::shared_ptr<ChBodyAuxRef> rock_Body;
int rock_id = -1;
bool add_rocks = false;
float sun_radius = 5.f * 1.414f; // [m]
float cam_radius = 1.0f; // [m]

// ---- camera parameters ---- //
CameraLensModelType lens_model = CameraLensModelType::PINHOLE; // Camera lens model, either PINHOLE or SPHERICAL
float update_rate = 240;				// [Hz], Update rate
unsigned int image_width = 1280 / 2;	// [px], image width
unsigned int image_height = 720 / 2;	// [px], image height
float fov = (float)CH_PI / 2.;			// [rad] camera's horizontal field of view (hFOV)
float lag = 0.0f;						// [sec], lag between sensing and when data becomes accessible
float exposure_time = 0.00f;			// [sec], exposure time of each image
int alias_factor = 1;					// number of samples per pixel (spp)


std::string file_type = "curet";			// target database type
// std::string mat_name = "sandbox_train";		// target material name

// ---- Genetic algorithm parameters ---- //
#define CROSS_RATE 0.9				// mating probability (DNA crossover)
#define MUTATION_RATE 0.050			// mutation probability

#define NUM_PARAMS 5				// number of parameters to optimize	
#define THREADS_PER_BLOCK 1024		// number of threads in a block

#define NUM_GENERATIONS 30			// number of evolving generations
// #define NUM_GENERATIONS 10			// number of evolving generations, debug
// #define NUM_CHILDS 50				// number of children in each generation
#define NUM_CHILDS 20				// number of children in each generation, debug
#define NUM_PROBES 15				// number of fitness per child
#define DNA_SIZE 20					// DNA length, no larger than lg_2(2147483648) = 31

const size_t NUM_ALL_GENOMES = NUM_CHILDS * NUM_PARAMS * DNA_SIZE;    
int rand_permut_30[] = {12, 0, 6, 27, 21, 24, 25, 9, 29, 3, 17, 5, 2, 22, 10, 26, 16, 1, 15, 28, 8, 11, 4, 18, 7, 14, 13, 23, 19, 20};

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------
// Save camera images
bool save = true;
// Render camera images
bool vis = true;
// Output directory
const std::string sensor_out_dir = "SENSOR_OUTPUT/CRMVisMatOpt_CAM/";

bool use_gi = false;

// NANO VDB
// nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> nvdb_handle;

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
	void createVoxelGrid(
		std::vector<float> points,
		ChSystemNSC& sys,
		std::shared_ptr<ChScene> scene,
		std::shared_ptr<ChVisualMaterial> vis_mat
	);
#endif

int num_meshes = 100;
std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>> regolith_meshes;  // ChVisualShapeTriangleMesh 

// ----------------------------- //
// ---- Function set starts ---- //
// ----------------------------- //

// Read BRDF data csv file in CUReT dataset, (polar_o, azimuth_o, polar_i, azimuth_i, R, G, B)
bool ReadCuretBrdfFile(const char *file_path, double* &brdf) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file." << std::endl;
        return false;
    }

    std::vector<double> data;
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            try {
                data.push_back(std::stod(cell));
            }
			catch (const std::invalid_argument& e) {
                std::cerr << "Invalid number encountered in the file: " << e.what() << std::endl;
                delete[] brdf;
                return false;
            }
			catch (const std::out_of_range& e) {
                std::cerr << "Number out of range: " << e.what() << std::endl;
                delete[] brdf;
                return false;
            }
        }
    }

    // Allocate memory for the output array
    brdf = new double[data.size()];
    std::copy(data.begin(), data.end(), brdf);

    return true;
}

// ---- genetic_algo_HPC.cu ---- //
// Return squared value of x
inline float sqr(float x) {
    return (x * x);
}

// Calculate sum of squared errors
double GetSSE(double *rgb_pred, double *rgb_gt) {
    return (sqr(rgb_pred[0] - rgb_gt[0]) + sqr(rgb_pred[1] - rgb_gt[1]) + sqr(rgb_pred[2] - rgb_gt[2]));
}

// Generate childs and make mutation occurs
void GenerateChilds(size_t DNA_idx, bool* parents, bool* childs) {
	// size_t DNA_idx = threadIdx.x + blockIdx.x * blockDim.x;
    // if (DNA_idx < num_childs * num_params) {
	
	// size_t parent_idx = DNA_idx / PARAMS_SIZE;
	size_t param_idx = DNA_idx % NUM_PARAMS;
	size_t child_idx;
	size_t parent_genome_idx;
	size_t child_genome_idx;
	curandState rgn;
	
	for (size_t genome_idx = 0; genome_idx < DNA_SIZE; ++genome_idx) {
		// mating process (genes crossover)
		if (UnifDist(rng) < CROSS_RATE) {
			child_idx = static_cast<size_t>(UnifDist(rng) * NUM_CHILDS); // select another individual from childs
			if (UnifDist(rng) > 0.5) {
				parent_genome_idx = DNA_idx * DNA_SIZE + genome_idx;
				child_genome_idx = (child_idx * NUM_PARAMS + param_idx) * DNA_SIZE + genome_idx;
				parents[parent_genome_idx] = childs[child_genome_idx]; // mating and produce one child
			}
		}
		
		// mutation occurs
		if (UnifDist(rng) < MUTATION_RATE) {
		// if (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) < MUTATION_RATE) {
			parent_genome_idx = DNA_idx * DNA_SIZE + genome_idx;
			parents[parent_genome_idx] = (parents[parent_genome_idx] + 1) % 2;
		}
	}
	
	// }
}

// Select better ones from childs as parents
void Select(size_t genome_idx, bool* parents, bool* childs, size_t* select_indices) {
    // size_t genome_idx = threadIdx.x + blockIdx.x * blockDim.x;
    // if (genome_idx < num_childs * num_params * dna_size) {
	size_t genome_num_per_child = NUM_PARAMS * DNA_SIZE;
	size_t parent_idx = genome_idx / genome_num_per_child;
	size_t child_genome_idx = genome_idx % genome_num_per_child;
	size_t child_idx = select_indices[parent_idx];
	parents[genome_idx] = childs[child_idx * genome_num_per_child + child_genome_idx];
	// }
}

// Calculate fitnesses
void GetFitnesses(size_t child_idx, float *SSEs, float *fitnesses, double* rgb_preds, double *rgb_gts) {
	// size_t child_idx = threadIdx.x + blockIdx.x * blockDim.x;
    // if (child_idx < num_childs) {	
	// Compute fitness
	SSEs[child_idx] = 0.f;
	size_t sample_idx;
	for (int probe_idx = 0; probe_idx < NUM_PROBES; ++probe_idx) {
		sample_idx = child_idx * NUM_PROBES + probe_idx;
		SSEs[child_idx] += GetSSE(&rgb_preds[3 * sample_idx], &rgb_gts[3 * sample_idx]);
	}
	
	// printf("%d, %f\n", child_idx, SSEs[child_idx]);
	fitnesses[child_idx] = 1.0f / (sqrtf(SSEs[child_idx] / NUM_PROBES) + 1e-8);
	// }
}




int main(int argc, char* argv[]) {

    // Create a physical system and a corresponding FSI system
    ChSystemNSC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    // Use JSON file to set the FSI parameters

    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Viper_granular_NSC.json");

    total_mass = 440.0;
    slope_angle = 0.0 / 180.0 * CH_PI;
    // wheel_AngVel = 0.8;
    out_dir = GetChronoOutputPath() + "FSI_Viper_Slope_WheelSinkage/";

	if (argc < 4) {
		std::cout << "/demo_ROBOT_Viper_WheelSinkage [mode: train, test] [mat_name] [number of records]\n";
		exit(1);
	}

	std::string mode = std::string(argv[1]);		// "train", "test"
	std::string mat_name = std::string(argv[2]);	// material name
	int num_records = atoi(argv[3]);				// number of records in each CUReT dataset table
	int num_groups = num_records / NUM_PROBES;

	// Read BRDF data table
    double *brdf_table;
	std::string file_path = GetChronoDataFile("robot/curiosity/rocks/" + mat_name + ".csv");
    if (file_type == "curet") {
        if (!ReadCuretBrdfFile(file_path.c_str(), brdf_table)) {
            fprintf(stderr, "Error reading %s\n", file_path.c_str());
            exit(1);
        }
    }
    else {
        fprintf(stderr, "unknown file type");
        exit(1);
    }
	
	// ChVector3f regolith_albedo(std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5]));
	// float regolith_specular = std::stof(argv[6]);
	// float regolith_roughness = std::stof(argv[7]);
	// float regolith_metallic = std::stof(argv[8]);
	// ChVector3f regolith_albedo(0.29f, 0.29f, 0.235f);
	ChVector3f regolith_albedo(1.0f, 0.f, 0.f);
	float regolith_specular = 0.3f;
	float regolith_roughness = 1.0f;
	float regolith_metallic = 0.f;

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

    // double gravity_G = sysFSI.GetGravitationalAcceleration().z();  // Is g already set?
    double gravity_G = 0.;  // [m/sec^2]
    ChVector3d gravity = ChVector3d(gravity_G * sin(slope_angle), 0, gravity_G * cos(slope_angle));
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    // Get the simulation stepsize
    dT = sysFSI.GetStepSize();

    // Get the initial particle spacing
    iniSpacing = sysFSI.GetInitialSpacing();

    // Get the SPH kernel length
    kernelLength = sysFSI.GetKernelLength();

    // Set the simulation domain size
    sysFSI.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetConsistentDerivativeDiscretization(false, false);

    // Setup the solver based on the input value of the prameters
    sysFSI.SetSPHMethod(SPHMethod::WCSPH);

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
        sysFSI.AddSPHParticle(
			points[i], sysFSI.GetDensity(), 0, sysFSI.GetViscosity(),
			ChVector3d(0),         // initial velocity
			ChVector3d(-pre_ini),  // tauxxyyzz
			ChVector3d(0)          // tauxyxzyz
        );
    }

    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();


    // ----------------------- //
    // SENSOR SIMULATION BEGIN //
    // ----------------------- //

    // Load regolith meshes
    std::string mesh_name_prefix = "sensor/geometries/regolith/particle_";
    for (int i = 1; i <= num_meshes; i++) {
        auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
            GetChronoDataFile(mesh_name_prefix + std::to_string(i) + ".obj"), false, true
		);
        mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
        auto trimesh_shape = std::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(mmesh);
        // std::cout << "OK1" << std::endl;
        trimesh_shape->SetName("RegolithMesh" + std::to_string(i));
        trimesh_shape->SetMutable(false);
        regolith_meshes.push_back(trimesh_shape);
    }

    // initialize regolith visual material
    auto regolith_mat = chrono_types::make_shared<ChVisualMaterial>();
	regolith_mat->SetDiffuseColor({regolith_albedo.x(), regolith_albedo.y(), regolith_albedo.z()}); // 0.29f, 0.29f, 0.235f
	regolith_mat->SetRoughness(regolith_roughness);
	regolith_mat->SetMetallic(regolith_metallic);	
	// ChVector3f C_specular = ((1.f - regolith_metallic) * 0.08f * regolith_specular * ChVector3f(1, 1, 1) + regolith_albedo * regolith_metallic) / 0.08f;
	regolith_mat->SetUseSpecularWorkflow(false);
	// regolith_mat->SetSpecularColor({C_specular.x(), C_specular.y(), C_specular.z()});
    
	regolith_mat->SetClassID(255); // first 4 bits in semantic cam, FF00
    regolith_mat->SetInstanceID(65280); // last 4 bits in semantic cam, 00FF

    // origin floor
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sysMBS.Add(floor);

	// test pole (for testing Sun's position) debug
	// auto sundial = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 2.0, 1000, true, false);
    // sundial->SetPos({0, 0, 1.0});
    // sundial->SetFixed(true);
    // sysMBS.Add(sundial);

    // Create a sensor manager
    float intensity = 1.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sysMBS);
	std::unordered_map<int, ChVector3f> sun_posi_map{ // [m]
        {0, { 5.f, 0.f, 5.f}}, {1, {0.f,  5.f, 5.f}},
        {2, {-5.f, 0.f, 5.f}}, {3, {0.f, -5.f, 5.f}}
    };
	PointLight sun = {{0.f, 0.f, 10.f}, {intensity, intensity, intensity}, 1000.f}; // {float3 posi, float3 color, float max_range}
    manager->scene->AddPointLight(sun);
    manager->scene->SetAmbientLight({.01, .01, .01});
    
	Background b;
    b.mode = BackgroundMode::SOLID_COLOR;
    b.color_zenith = ChVector3f(0.f, 0.f, 0.f);
    
    manager->scene->SetBackground(b);
    manager->SetVerbose(false);
    manager->SetRayRecursions(4);
    Integrator integrator = Integrator::LEGACY;
    bool use_denoiser = false;

    // chrono::ChFrame<double> offset_pose1({0, 5, 0}, Q_from_AngAxis(0.2, {0, 0, 1}));  //-1200, -252, 100
    chrono::ChFrame<double> offset_pose1(
		{ -cam_radius * sin(56.31 * CH_PI/180.), 0., cam_radius * cos(56.31 * CH_PI/180.)},
		QuatFromAngleAxis((90 - 56.31) * CH_PI/180., {0, 1, 0})
	);  // Q_from_AngAxis(CH_PI_4, {0, 1, 0})  //-1200, -252, 100
    auto cam = chrono_types::make_shared<ChCameraSensor>(
		floor,         // body camera is attached to
		update_rate,   // update rate in Hz
		offset_pose1,  // offset pose
		image_width,   // image width
		image_height,  // image height
		fov,           // camera's horizontal field of view
		alias_factor,  // super sampling factor
		lens_model,    // lens model type
		use_gi
	);
    cam->SetIntegrator(integrator);
    cam->SetName("Third Person Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);
    
	// Provides the host access to the RGBA8 buffer
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
	
	if (vis) {
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Third Person Camera"));
	}

	std::shared_ptr<ChFilterSave> save_cam_filter_ptr = chrono_types::make_shared<ChFilterSave>(sensor_out_dir);
    if (save) {
		cam->PushFilter(save_cam_filter_ptr);
	}
    manager->AddSensor(cam);
    
    // Get the chassis of the rover
    auto body = sysMBS.GetBodies()[1];

    float addNVDBTime = 0.0f;
    int sensor_render_steps = (unsigned int)round(1 / (update_rate * dT));
    int RTF_cout_period = 100; // [count]
    int RTF_cout_count = 0; // [count]
    std::vector<float> h_points;
	UserRGBA8BufferPtr rgba8_ptr;

	std::cout << "sensor_render_steps: " << sensor_render_steps << std::endl;
    
	// debug log, check CUReT brdf_table correctly read
	// for (size_t i = 0; i < 205; ++i) {
	// 	printf("%2ld: ", i);
	// 	for (size_t j =0; j < 7; ++j) {
	// 		printf("%6.3f ", brdf_table[7 * i + j]);
	// 	}
	// 	printf("\n");
	// }

	if (mode == "train") {
		// ---- Initialize GA variables ---- //
		size_t blocks_per_grid;

		// create timing variables for CUDA
		cudaEvent_t start_time_GA, stop_time_GA;
		float elapsed_time_GA; // [ms]
		cudaEventCreate(&start_time_GA);
		cudaEventCreate(&stop_time_GA);

		// containers
		bool childs[NUM_ALL_GENOMES] = {false};
		bool parents[NUM_ALL_GENOMES] = {false};
		
		float mapped_param_values[NUM_CHILDS * NUM_PARAMS] = {0.f};

		float fitnesses[NUM_CHILDS] = {0.f};
		float SSEs[NUM_CHILDS] = {0.f};
		
		float polar_ins[NUM_CHILDS * NUM_PROBES] = {0.f};
		float azi_ins[NUM_CHILDS * NUM_PROBES] = {0.f};
		float polar_outs[NUM_CHILDS * NUM_PROBES] = {0.f};
		float azi_outs[NUM_CHILDS * NUM_PROBES] = {0.f};
		float expsrs[NUM_CHILDS * NUM_PROBES] = {0.f};
		
		double rgb_gts[NUM_CHILDS * NUM_PROBES * 3] = {0.7};
		double rgb_preds[NUM_CHILDS * NUM_PROBES * 3] = {0.};
		
		// debug
		// for (size_t rgb_idx = 0; rgb_idx < NUM_CHILDS * NUM_PROBES; ++rgb_idx) {
		// 	rgb_gts[rgb_idx * 3 + 0] = 0.0;	// Red
		// 	rgb_gts[rgb_idx * 3 + 1] = 1.0;	// Green
		// 	rgb_gts[rgb_idx * 3 + 2] = 0.0;	// Blue
		// }

		size_t select_indices[NUM_CHILDS] = {0};

		// statistical variables
		float best_params[NUM_PARAMS];
		float global_min_SSE = FLT_MAX;
		size_t argmax_fitness;
		float avg_fitness;
		float tot_elapsed_time_GA = 0.; // [ms]

		// randomly initialize genomes for each child in the 0th generation
		for (size_t genome_idx = 0; genome_idx < NUM_ALL_GENOMES; ++genome_idx) {
			childs[genome_idx] = rand() % 2;
		}

		// debug
		// printf("\n\n");
		// for (size_t child_idx = 0; child_idx < NUM_CHILDS; ++child_idx) {
		// 	printf("Child %zd: ", child_idx);
		// 		for (size_t param_idx = 0; param_idx < NUM_PARAMS; ++param_idx) {
		// 			for (size_t genome_idx = 0; genome_idx < DNA_SIZE; ++genome_idx) {
		// 				printf()
		// 			}


		// ------------------------------- //
		// ---- Run genetic algorithm ---- //
		// ------------------------------- //
		// size_t brdf_record_idx = 0;
		for (size_t iter = 0; iter < NUM_GENERATIONS; ++iter) {
			
			// debug
			// theta_in = 45  * (M_PI / 180); // [rad]
			// phi_in = 0.; // [rad]
			// theta_out = 45  * (M_PI / 180); // [rad]
			// phi_out = M_PI; // [rad]
			
			// Map all binary DNAs to decimal values
			size_t param_value_idx = 0;
			for (size_t child_idx = 0; child_idx < NUM_CHILDS; ++child_idx) {
				for (size_t param_idx = 0; param_idx < NUM_PARAMS; ++param_idx) {
					param_value_idx = child_idx * NUM_PARAMS + param_idx;
					size_t value = 0, base = 1;
					for (int genome_idx = 0; genome_idx < DNA_SIZE; ++genome_idx) {
						// bool *child = &childs[param_value_idx * DNA_SIZE]
						// value += child[genome_idx] * base;
						value += childs[param_value_idx * DNA_SIZE + genome_idx] * base;
						base *= 2;
					}
					mapped_param_values[param_value_idx] = static_cast<float>(value) / powf(2, DNA_SIZE);
				}
			}

			// debug
			// printf("\n\n");
			// for (size_t child_idx = 0; child_idx < NUM_CHILDS; ++child_idx) {
			//     printf("Child %zd: ", child_idx);
			// 	for (size_t param_idx = 0; param_idx < NUM_PARAMS; ++param_idx) {
			//         param_value_idx = child_idx * NUM_PARAMS + param_idx;
			// 		printf("%.4f ", mapped_param_values[param_value_idx]);
			// 	}
			// 	printf("\n");
			// }
			// printf("\n");

			// Randomly choose feature data and corresponding g.t. labels
			for (size_t child_idx = 0; child_idx < NUM_CHILDS; ++child_idx) {
				for (size_t probe_idx = 0; probe_idx < NUM_PROBES; ++probe_idx) {
					int sample_idx = child_idx * NUM_PROBES + probe_idx;
					
					// polar_ins[sample_idx]	= (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) + 0.8f) * 0.25 * M_PI; // [rad]
					// azi_ins[sample_idx]		= static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 * M_PI; // [rad]
					// polar_outs[sample_idx]	= (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) + 0.8f) * 0.25 * M_PI; // [rad]
					// expsrs[sample_idx] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 * M_PI; // [rad]
					
					// LookUpBRDFVal(
					//     brdf_table, (double)polar_ins[sample_idx], (double)azi_ins[sample_idx], (double)polar_outs[sample_idx],
					//     (double)azi_outs[sample_idx], &rgb_gts[3 * sample_idx]
					// );
					
					// (polar_o, azimuth_o, polar_i, azimuth_i, R, G, B)
					// int brdf_record_idx;
					// do {
					// 	brdf_record_idx = rand() % num_records;
					// } while (brdf_table[7 * brdf_record_idx + 2] < 0.25 * M_PI);
					
					int group_idx = child_idx % num_groups;
					int brdf_record_idx = rand_permut_30[2 * probe_idx + group_idx];
					polar_outs[sample_idx]		= brdf_table[7 * brdf_record_idx + 0];
					azi_outs[sample_idx]		= brdf_table[7 * brdf_record_idx + 1];
					polar_ins[sample_idx]		= brdf_table[7 * brdf_record_idx + 2];
					azi_ins[sample_idx]			= brdf_table[7 * brdf_record_idx + 3];
					rgb_gts[3 * sample_idx + 0]	= brdf_table[7 * brdf_record_idx + 4];
					rgb_gts[3 * sample_idx + 1]	= brdf_table[7 * brdf_record_idx + 5];
					rgb_gts[3 * sample_idx + 2]	= brdf_table[7 * brdf_record_idx + 6];
					// brdf_record_idx += 1;
				}
			}

			// collect records of sample data
			int steps_to_change = 5;
			int total_time_steps = sensor_render_steps * steps_to_change * (NUM_CHILDS * NUM_PROBES + 1);
			float polar_in = 0.f, azi_in = 0.f, polar_out = 0.f, azi_out = 0.f;
			// Start the simulation
			// unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));
			double time = 0.0;
			int current_step = 0;
			ChTimer timer;
			ChTimer timerNVDB;
			while (current_step < total_time_steps) {
				
				// Time log
				// ++RTF_cout_count;
				// if (RTF_cout_count > RTF_cout_period) {
				// 	std::cout << current_step << "  time: " << time << " [sec], sim. time: " << timer() << " [sec], RTF: " << timer()/time << std::endl;
				// 	RTF_cout_count = 0;
				// }

				timer.start();

				// Render sensors
				if (current_step % sensor_render_steps == 0 && current_step > 0) {
					
					//timerNVDB.start();
					h_points = sysFSI.GetParticleData();
					
					int child_idx;
					int sample_idx;
					// Assign a new sample
					// std::cout << current_step / sensor_render_steps << std::endl;
					if ((current_step / sensor_render_steps) % steps_to_change == 0) {
						
						if ((current_step / sensor_render_steps / steps_to_change - 1) % NUM_PROBES == 0) {
							child_idx = (current_step / sensor_render_steps / steps_to_change - 1) / NUM_PROBES;
							
							printf("\nChild %02d: ", child_idx);

							// Randomly assign regolith visual material parameters (albedo, roughness, metallic), debug
							// regolith_albedo = ChVector3f(UnifDist(rng), UnifDist(rng), UnifDist(rng));
							// regolith_specular = UnifDist(rng);
							// regolith_roughness = UnifDist(rng);
							// regolith_metallic = UnifDist(rng);
							
							float* param_values = &(mapped_param_values[child_idx * NUM_PARAMS]);
							regolith_albedo = ChVector3f(param_values[0], param_values[1], param_values[2]);
							regolith_roughness = param_values[3];
							regolith_metallic = param_values[4];

							// debug
							// regolith_albedo = ChVector3f(0.6519f, 0.3839f, 0.2709f);
							// regolith_roughness = 0.6964f;
							// regolith_metallic = 0.1383f;
							
							// change regolith visual material parameters
							regolith_mat->SetDiffuseColor({regolith_albedo.x(), regolith_albedo.y(), regolith_albedo.z()});
							regolith_mat->SetRoughness(regolith_roughness);
							regolith_mat->SetMetallic(regolith_metallic);	
							// ChVector3f C_specular = ((1.f - regolith_metallic) * 0.08f * regolith_specular * ChVector3f(1.f, 1.f, 1.f) + regolith_albedo * regolith_metallic) / 0.08f;
							// regolith_mat->SetUseSpecularWorkflow(false);
							// regolith_mat->SetSpecularColor({C_specular.x(), C_specular.y(), C_specular.z()});
							// regolith_mat->SetSpecularColor({1.f, 1.f, 1.f});

							// debug
							// printf("albedo: %.4f, %.4f, %.4f\n", regolith_mat->GetDiffuseColor().R, regolith_mat->GetDiffuseColor().G, regolith_mat->GetDiffuseColor().B);
							// printf("specular color: %.4f, %.4f, %.4f\n", regolith_mat->GetSpecularColor().R, regolith_mat->GetSpecularColor().G, regolith_mat->GetSpecularColor().B);
							// printf("rough: %.4f , metal: %.4f\n", regolith_mat->GetRoughness(), regolith_mat->GetMetallic());
							// printf("\n");
							
							printf(
								"params (%.4f, %.4f, %.4f, %.4f, %.4f) , ",
								regolith_mat->GetDiffuseColor().R, regolith_mat->GetDiffuseColor().G, regolith_mat->GetDiffuseColor().B,
								regolith_mat->GetRoughness(), regolith_mat->GetMetallic()
							);

							manager->ReconstructScenes();

						}
						
						int probe_idx = (current_step / sensor_render_steps / steps_to_change - 1) % NUM_PROBES;						
						sample_idx = child_idx * NUM_PROBES + probe_idx;
						polar_in = polar_ins[sample_idx];
						azi_in = azi_ins[sample_idx];
						polar_out = polar_outs[sample_idx];
						azi_out = azi_outs[sample_idx];

						// debug log
						// printf(
						// 	"\n%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n",
						// 	polar_out, azi_out, polar_in, azi_in, rgb_gts[3 * sample_idx + 0], rgb_gts[3 * sample_idx + 1], rgb_gts[3 * sample_idx + 2]
						// );
						
						// Change Sun's position and camera's pose
						// ChVector3f sun_posi = sun_posi_map[rand() % 4]; // debug
						ChVector3f sun_posi(
							sun_radius * sinf(polar_in) * cosf(azi_in),
							sun_radius * sinf(polar_in) * sinf(azi_in),
							sun_radius * cosf(polar_in)
						);
						PointLight p = { // {float3 posi, float3 color, float max_range}
							{sun_posi.x(), sun_posi.y(), sun_posi.z()}, {intensity, intensity, intensity}, 1000.f
						}; 
						manager->scene->ModifyPointLight(0, p);

						// Change Cmaera's pose
						chrono::ChFrame<double> cam_pose(
							{cam_radius * sin(polar_out) * cos(azi_out), cam_radius * sin(polar_out) * sin(azi_out), cam_radius * cos(polar_out)},
							QuatFromAngleAxis(CH_PI + azi_out, {0, 0, 1}) * QuatFromAngleAxis(CH_PI_2 - polar_out, {0, 1, 0})
						);

						cam->SetOffsetPose(cam_pose);
						
					}

					#ifdef USE_SENSOR_NVDB
						createVoxelGrid(h_points, sysMBS, manager->scene, regolith_mat);
					#else
						createVoxelGrid(h_points, sysMBS, manager->scene, regolith_mat);
					#endif

					manager->Update();
					
					// Collect average reflectance color values
					if ((current_step / sensor_render_steps) % steps_to_change == 2) {
						// Access the camera output buffer
						rgba8_ptr = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
						
						if (rgba8_ptr->Buffer) {
							// Calculate the average RGB values in the buffer
							unsigned int img_h = rgba8_ptr->Height;
							unsigned int img_w = rgba8_ptr->Width;
							
							PixelRGBA8 pixel;
							unsigned int RGB_tot_values[3] = {0};
							unsigned int num_px = 0;
							for (int i = 0; i < img_h; ++i) {
								for (int j = 0; j < img_w; ++j) {
									pixel = rgba8_ptr->Buffer[(i * img_w + j)];
									if ((unsigned(pixel.R) > 1) || (unsigned(pixel.G) > 1) || (unsigned(pixel.B) > 1)) {
										// std::cout << uint8_t(pixel.R) << ", " << uint8_t(pixel.G) << ", " << uint8_t(pixel.B) << std::endl;
										// std::cout << unsigned(pixel.R) << ", " << unsigned(pixel.G) << ", " << unsigned(pixel.B) << ", ";
										RGB_tot_values[0] += unsigned(pixel.R);
										RGB_tot_values[1] += unsigned(pixel.G);
										RGB_tot_values[2] += unsigned(pixel.B);
										num_px += 1;
									}
								}
							}
							
							// printf("\n");
							// printf(
							// 	"Average RGB values: %.4f, %.4f, %.4f\n\n",
							// 	RGB_tot_values[0] / 255.0 / num_px, RGB_tot_values[1] / 255.0 / num_px, RGB_tot_values[2] / 255.0 / num_px
							// );
							
							// printf("response (");
							printf("SSE ");
							for (int ch_idx = 0; ch_idx < 3; ++ch_idx) {
								rgb_preds[3 * sample_idx + ch_idx] = RGB_tot_values[ch_idx] / 255.0 / num_px * 1.0; // 2.0: magic number
								// printf("%.4f, ", rgb_preds[3 * sample_idx + ch_idx]);
							}
							// printf("), ")
							printf("%.5f, ", GetSSE(&(rgb_preds[3 * sample_idx]), &(rgb_gts[3 * sample_idx])));
						}
					}
				}

				// rover->Update();

				sysFSI.DoStepDynamics_FSI();
				timer.stop();

				time += dT;
				current_step++;
			}

			printf("\n\nelapsed time for rendering per generation : %f sec\n\n", timer());

			// ---- Step 1: Calculate fitness ---- //
			cudaEventRecord(start_time_GA);
			
			for (size_t child_idx = 0; child_idx < NUM_CHILDS; ++child_idx) {
				GetFitnesses(child_idx, SSEs, fitnesses, rgb_preds, rgb_gts);
			}
			// blocks_per_grid = (NUM_CHILDS + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
			// GetFitnesses<<<blocks_per_grid, THREADS_PER_BLOCK>>>(SSEs, fitnesses, rgb_preds, rgb_gts, NUM_CHILDS, NUM_PROBES);
			// cudaDeviceSynchronize();
			// printf("Gen %zd: Step 1 finished\n", iter); //debug

			

			
			// debug
			// for (size_t child_idx = 0; child_idx < NUM_CHILDS; ++child_idx) {
			//     printf("%zd, %f\n", child_idx, fitnesses[child_idx]);
			// }
			// printf("Gen %zd: max fitness = %f @ %zd\n", iter, max_fitness, argmax_fitness);
			// printf("Gen %zd: best SSE = %f, most fitted parameter:\n", iter, SSEs[argmax_fitness]);
			// printf("baseColor_r = %f, baseColor_g = %f, baseColor_b = %f, specular = %f, roughness = %f, metallic = %f\n",
			//         mapped_param_values[argmax_fitness * NUM_PARAMS + 0], mapped_param_values[argmax_fitness * NUM_PARAMS + 1],
			//         mapped_param_values[argmax_fitness * NUM_PARAMS + 2], mapped_param_values[argmax_fitness * NUM_PARAMS + 3],
			//         mapped_param_values[argmax_fitness * NUM_PARAMS + 4], mapped_param_values[argmax_fitness * NUM_PARAMS + 5]);
			// std::cout << iter << ": Step 2 finished" << std::endl;
		
			// ---- Step 2: Select better parents ---- //

			std::default_random_engine generator;
			std::discrete_distribution<> select_distr(fitnesses, fitnesses + NUM_CHILDS);
			for (size_t parent_idx = 0; parent_idx < NUM_CHILDS; ++parent_idx) {
				select_indices[parent_idx] = select_distr(generator);
			}

			for (size_t genome_idx = 0; genome_idx < NUM_ALL_GENOMES; ++genome_idx){
				Select(genome_idx, parents, childs, select_indices);
			}
			// blocks_per_grid = ((NUM_ALL_GENOMES) + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
			// Select<<<blocks_per_grid, THREADS_PER_BLOCK>>>(parents, childs, select_indices, NUM_CHILDS, DNA_SIZE, NUM_PARAMS);
			
			// Old code
			// for (size_t parent_idx = 0; parent_idx < NUM_CHILDS; ++parent_idx) {
			//     size_t child_idx = select_distr(generator);
			//     for (size_t i = 0; i < genome_num_per_child; ++i) {
			//         parents[parent_idx * genome_num_per_child + i] = childs[child_idx * genome_num_per_child + i];
			//     }
			// }
			
			// std::cout << iter << ": Step 3 finished" << std::endl; // debug

			// ---- Step 3: generate childs and make mutation occurs ---- //
			size_t num_DNAs = NUM_CHILDS * NUM_PARAMS;
			for (size_t DNA_idx = 0; DNA_idx < num_DNAs; ++DNA_idx) {
				GenerateChilds(DNA_idx, parents, childs);
			}

			// blocks_per_grid = ((NUM_CHILDS * NUM_PARAMS) + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
			// GenerateChilds<<<blocks_per_grid, THREADS_PER_BLOCK>>>(parents, childs, rand(), NUM_CHILDS, DNA_SIZE);
			// cudaDeviceSynchronize();
			
			// std::cout << (parent_idx * NUM_PARAMS + param_idx) << std::endl;
			// crossover(parents, childs, parent_idx, param_idx, rand());
			// mutate(parents, parent_idx, param_idx, rand());
			std::swap(childs, parents);
			// std::cout << iter << ": Step 4 finished" << std::endl;

			cudaEventRecord(stop_time_GA);
			cudaEventSynchronize(stop_time_GA);
			cudaEventElapsedTime(&elapsed_time_GA, start_time_GA, stop_time_GA);
			tot_elapsed_time_GA += elapsed_time_GA;

			printf("elapsed time of GA per generation : %f ms\n\n", tot_elapsed_time_GA / (iter + 1));

			// Get the best parameters
			// ref: https://forums.developer.nvidia.com/t/using-thrust-to-sort-unified-memory-buffer/37849/2
			// ref: https://stackoverflow.com/questions/12614164/generating-random-numbers-with-uniform-distribution-using-thrust
			// ref: https://gist.github.com/ashwin/7245048
			
			// Find the max and argmax of the fitness values, and calculate average fitness
			float max_fitness = fitnesses[0];
			avg_fitness = 0.;
			size_t argmax_fitness = 0;
			for (size_t child_idx = 1; child_idx < NUM_CHILDS; ++child_idx) {
				if (max_fitness < fitnesses[child_idx]) {
					max_fitness = fitnesses[child_idx];
					argmax_fitness = child_idx;
				}
				avg_fitness += fitnesses[child_idx];
			}
			avg_fitness /= NUM_CHILDS;
			
			// thrust::device_vector<float> fitnesses_dev(fitnesses, fitnesses + NUM_CHILDS);
			// thrust::device_vector<float>::iterator max_fitness_iter;
			// max_fitness_iter = thrust::max_element(thrust::device, fitnesses_dev.begin(), fitnesses_dev.end());
			// argmax_fitness = thrust::distance(fitnesses_dev.begin(), max_fitness_iter);
			// avg_fitness = thrust::reduce(fitnesses_dev.begin(), fitnesses_dev.end(), 0.f, thrust::plus<float>()) / NUM_CHILDS;

			if (SSEs[argmax_fitness] < global_min_SSE) {
				global_min_SSE = SSEs[argmax_fitness];
				for (int i = 0; i < NUM_PARAMS; ++i) {
					best_params[i] = mapped_param_values[argmax_fitness * NUM_PARAMS + i];
				}
			}
			
			// declaring argument of time() 
			std::time_t my_time = std::time(NULL); 
			printf("%s\n", std::ctime(&my_time));

			printf("Iter %zd: avg_fitness %f\n\n", iter + 1, avg_fitness); // commented when running scalability analysis
			
			// Print best parameters
			printf("best params: ");
			for (int i = 0; i < NUM_PARAMS; ++i) {
				printf("%.4f, ", best_params[i]);
			}
			printf("min_SSE = %.5f\n\n", global_min_SSE);
		}	
		
	}
	else if (mode == "test") { 
		regolith_albedo = ChVector3f(0.8197f, 0.7391f, 0.8095f);
		regolith_roughness = 0.0267f;
		regolith_metallic = 0.5399f;

		// change regolith visual material parameters
		regolith_mat->SetDiffuseColor({regolith_albedo.x(), regolith_albedo.y(), regolith_albedo.z()});
		regolith_mat->SetRoughness(regolith_roughness);
		regolith_mat->SetMetallic(regolith_metallic);	
		regolith_mat->SetUseSpecularWorkflow(false);

		float polar_ins[num_records] = {0.f};
		float azi_ins[num_records] = {0.f};
		float polar_outs[num_records] = {0.f};
		float azi_outs[num_records] = {0.f};
		for (int record_idx = 0; record_idx < num_records; ++record_idx) {
			polar_outs[record_idx]		= brdf_table[4 * record_idx + 0];
			azi_outs[record_idx]		= brdf_table[4 * record_idx + 1];
			polar_ins[record_idx]		= brdf_table[4 * record_idx + 2];
			azi_ins[record_idx]			= brdf_table[4 * record_idx + 3];
		}

		// -------------------- //
		// START THE SIMULATION //
		// -------------------- //
		int steps_to_change = 5;
		int total_time_steps = sensor_render_steps * steps_to_change * num_records;
		float polar_in = 0.f, azi_in = 0.f, polar_out = 0.f, azi_out = 0.f;
		
		double time = 0.0;
		int current_step = 0;
		ChTimer timer;
		while (current_step < total_time_steps) {
			
			// Time log
			// ++RTF_cout_count;
			// if (RTF_cout_count > RTF_cout_period) {
			// 	std::cout << current_step << "  time: " << time << " [sec], sim. time: " << timer() << " [sec], RTF: " << timer()/time << std::endl;
			// 	RTF_cout_count = 0;
			// }

			timer.start();

			// Render sensors
			if (current_step % sensor_render_steps == 0) {
				
				h_points = sysFSI.GetParticleData();
				
				// Assign a new sample
				if ((current_step / sensor_render_steps) % steps_to_change == 0) {
					
					int sample_idx = (current_step / sensor_render_steps / steps_to_change) % num_records;						
					polar_out = polar_outs[sample_idx];
					azi_out = azi_outs[sample_idx];
					polar_in = polar_ins[sample_idx];
					azi_in = azi_ins[sample_idx];

					// debug log
					// printf("\n%.4f, %.4f, %.4f, %.4f\n", polar_out, azi_out, polar_in, azi_in);
					
					// Change Sun's position and camera's pose
					ChVector3f sun_posi(
						sun_radius * sinf(polar_in) * cosf(azi_in),
						sun_radius * sinf(polar_in) * sinf(azi_in),
						sun_radius * cosf(polar_in)
					);
					PointLight p = { // {float3 posi, float3 color, float max_range}
						{sun_posi.x(), sun_posi.y(), sun_posi.z()}, {intensity, intensity, intensity}, 1000.f
					}; 
					manager->scene->ModifyPointLight(0, p);

					// Change Cmaera's pose
					chrono::ChFrame<double> cam_pose(
						{cam_radius * sin(polar_out) * cos(azi_out), cam_radius * sin(polar_out) * sin(azi_out), cam_radius * cos(polar_out)},
						QuatFromAngleAxis(CH_PI + azi_out, {0, 0, 1}) * QuatFromAngleAxis(CH_PI_2 - polar_out, {0, 1, 0})
					);

					cam->SetOffsetPose(cam_pose);

					// Change folder to save images
					std::ostringstream oss;
					float new_azi_out = (azi_out < 0) ? (2 * CH_PI + azi_out) : azi_out;
					float new_azi_in = (azi_in < 0) ? (2 * CH_PI + azi_in) : azi_in;
					oss << std::setw(3) << std::setfill('0') << static_cast<int>(polar_out * 180/CH_PI) << "_"
						<< std::setw(3) << std::setfill('0') << static_cast<int>(new_azi_out * 180/CH_PI) << "_"
						<< std::setw(3) << std::setfill('0') << static_cast<int>(polar_in * 180/CH_PI) << "_"
						<< std::setw(3) << std::setfill('0') << static_cast<int>(new_azi_in * 180/CH_PI);
					std::string folder_path = sensor_out_dir + oss.str() + "/";
					std::filesystem::create_directory(folder_path);
					save_cam_filter_ptr->ChangeDataPath(folder_path);	
				}

				#ifdef USE_SENSOR_NVDB
					createVoxelGrid(h_points, sysMBS, manager->scene, regolith_mat);
				#else
					createVoxelGrid(h_points, sysMBS, manager->scene, regolith_mat);
				#endif

				// Save images of testing data
				manager->Update();
			}

			// rover->Update();

			sysFSI.DoStepDynamics_FSI();
			timer.stop();

			time += dT;
			current_step++;
		}

	}
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

    //   openvdb::points::PointDataGrid::Ptr grid =
    //       openvdb::points::createPointDataGrid<openvdb:
    //       :points::NullCodec, openvdb::points::PointDataGrid>(points,*transform);

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

    // std::cout << "Creating CPU Voxel Grid for " << points.size() / 6 << " particles" << std::endl;
    float spacing = iniSpacing / 2.f;
    float r = spacing;
    int pointsPerVoxel = 1;
    float voxelSize = spacing;  
    // std::cout << "VoxelSize=" << voxelSize << std::endl;

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
                 ChVector3d pos(points[6*i],points[6*i + 1],points[6*i+2]);
                if (!idList.empty() && ((voxelCount < prevActiveVoxels) || (voxelCount < idList.size()))) {
                    numUpdates++;
                    auto voxelBody = voxelBodyList[idList[voxelCount]];
                    float offsetX = offsetXList[idList[voxelCount]];
                    float offsetY = offsetYList[idList[voxelCount]];
                    ChVector3d voxelPos(pos.x() + offsetX, pos.y() + offsetY, pos.z());
                    double xRot = voxelPos.x() * cos(-slope_angle) + voxelPos.z() * sin(-slope_angle);
                    double yRot = voxelPos.y();
                    double zRot = -voxelPos.x() * sin(-slope_angle) + voxelPos.z() * cos(-slope_angle);

                    // Create a new rotated vector
                    ChVector3d rotatedVoxelPos(xRot, yRot, zRot);
                    voxelBody->SetPos(rotatedVoxelPos);
                    voxelBody->SetRot(QuatFromAngleY(-slope_angle));
                    // voxelBody->SetPos({voxelPos.x(), voxelPos.y(), voxelPos.z()});

                    voxelBody->GetVisualShape(0)->SetMaterial(0, vis_mat);
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
    // std::cout << "Num Voxels: " << voxelBodyList.size() << std::endl;
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
    sysFSI.AddBoxContainerBCE(
		box,                                        //
		ChFrame<>(ChVector3d(0, 0, bzDim), QUNIT),  //
		ChVector3d(bxDim, byDim, 2 * bzDim),        //
		ChVector3i(2, 0, -1)
	);
    
	rover = chrono_types::make_shared<Viper>(&sysMBS, wheel_type);
	
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    
    rover->SetDriver(driver);
    // rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    //rover->Initialize(ChFrame<>(init_loc, QUNIT));
    double xRot = init_loc.x() * cos(-slope_angle) + init_loc.z() * sin(-slope_angle);
    double yRot = init_loc.y();
    double zRot = -init_loc.x() * sin(-slope_angle) + init_loc.z() * cos(-slope_angle);

    // Create a new rotated vector
    ChVector3d rotatedRoverInitLoc(xRot, yRot, zRot - 0.1);
    rover->Initialize(ChFrame<>(rotatedRoverInitLoc, QuatFromAngleY(-slope_angle)));


    // Create the wheel's BCE particles
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    std::vector<ChVector3d> BCE_wheel;
    sysFSI.CreateMeshPoints(*trimesh, initSpace0, BCE_wheel);

    // Add BCE particles, mesh, and visual material IDs of wheels to the system
	auto wheel_mat = chrono_types::make_shared<ChVisualMaterial>();
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> wheel_body;
        if (i == 0) {
            wheel_body = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
			wheel_body->GetVisualShape(0)->SetMaterial(0, wheel_mat);
			// wheel_body->GetVisualModel()->GetShapeInstances()[0].first->GetMaterial(0)->SetClassID(0);
        	// wheel_body->GetVisualModel()->GetShapeInstances()[0].first->GetMaterial(0)->SetInstanceID(65535);
			wheel_body->GetVisualShape(0)->GetMaterial(0)->SetClassID(0);
        	wheel_body->GetVisualShape(0)->GetMaterial(0)->SetInstanceID(65535);
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


