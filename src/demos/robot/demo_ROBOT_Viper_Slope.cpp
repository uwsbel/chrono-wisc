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
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_models/robot/viper/Viper.h"

#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include <filesystem>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
using namespace chrono::viper;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// ===================================================================================================================

// CRM terrain patch type
enum class PatchType { RECTANGULAR, HEIGHT_MAP };
PatchType patch_type = PatchType::HEIGHT_MAP;

// Terrain dimensions (for RECTANGULAR or HEIGHT_MAP patch type)
double terrain_length = 10;  // 5m unsloped + 5m slope
double terrain_width = 3;

// ===================================================================================================================

// Function to handle CLI arguments
bool GetProblemSpecs(int argc,
                     char** argv,
                     double& slope_angle,
                     bool& render,
                     double& render_fps,
                     bool& visualization_sph,
                     bool& visualization_bndry_bce,
                     bool& visualization_rigid_bce,
                     double& density,
                     double& cohesion,
                     double& friction,
                     double& youngs_modulus,
                     double& poisson_ratio,
                     double& gravity) {
    ChCLI cli(argv[0], "Viper rover on CRM deformable terrain");

    cli.AddOption<double>("Simulation", "slope_angle", "Slope angle in degrees", std::to_string(slope_angle));

    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");
    cli.AddOption<double>("Visualization", "render_fps", "Render frequency (FPS)", std::to_string(render_fps));
    cli.AddOption<bool>("Visualization", "sph", "Render SPH particles", std::to_string(visualization_sph));
    cli.AddOption<bool>("Visualization", "bndry_bce", "Render boundary BCE markers",
                        std::to_string(visualization_bndry_bce));
    cli.AddOption<bool>("Visualization", "rigid_bce", "Render wheel BCE markers",
                        std::to_string(visualization_rigid_bce));

    cli.AddOption<double>("Physics", "density", "Density", std::to_string(density));
    cli.AddOption<double>("Physics", "cohesion", "Cohesion", std::to_string(cohesion));
    cli.AddOption<double>("Physics", "friction", "Friction", std::to_string(friction));
    cli.AddOption<double>("Physics", "youngs_modulus", "Young's modulus", std::to_string(youngs_modulus));
    cli.AddOption<double>("Physics", "poisson_ratio", "Poisson ratio", std::to_string(poisson_ratio));

    cli.AddOption<double>("Physics", "gravity", "Gravity", std::to_string(gravity));

    if (!cli.Parse(argc, argv))
        return false;

    slope_angle = cli.GetAsType<double>("slope_angle");
    render = !cli.GetAsType<bool>("no_vis");
    render_fps = cli.GetAsType<double>("render_fps");
    visualization_sph = cli.GetAsType<bool>("sph");
    visualization_bndry_bce = cli.GetAsType<bool>("bndry_bce");
    visualization_rigid_bce = cli.GetAsType<bool>("rigid_bce");
    density = cli.GetAsType<double>("density");
    cohesion = cli.GetAsType<double>("cohesion");
    friction = cli.GetAsType<double>("friction");
    youngs_modulus = cli.GetAsType<double>("youngs_modulus");
    poisson_ratio = cli.GetAsType<double>("poisson_ratio");
    gravity = cli.GetAsType<double>("gravity");
    return true;
}

int main(int argc, char* argv[]) {
    // Default values
    double slope_angle = 15.0;
    bool render = true;
    double render_fps = 200.0;
    bool visualization_sph = true;
    bool visualization_bndry_bce = false;
    bool visualization_rigid_bce = false;
    double density = 1700;
    double cohesion = 5e3;
    double friction = 0.7;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;
    double gravity = 9.81;

    // Parse command line arguments
    if (!GetProblemSpecs(argc, argv, slope_angle, render, render_fps, visualization_sph, visualization_bndry_bce,
                         visualization_rigid_bce, density, cohesion, friction, youngs_modulus, poisson_ratio,
                         gravity)) {
        return 1;
    }

    std::cout << "Using slope angle: " << slope_angle << " degrees" << std::endl;
    std::cout << "Visualization: " << (render ? "enabled" : "disabled") << std::endl;
    if (render) {
        std::cout << "  Render FPS: " << render_fps << std::endl;
        std::cout << "  SPH particles: " << (visualization_sph ? "enabled" : "disabled") << std::endl;
        std::cout << "  Boundary BCE: " << (visualization_bndry_bce ? "enabled" : "disabled") << std::endl;
        std::cout << "  Rigid BCE: " << (visualization_rigid_bce ? "enabled" : "disabled") << std::endl;
    }

    double tend = 7;
    double step_size = 2e-4;
    ChVector3d active_box_dim(0.6, 0.6, 0.6);

    bool verbose = true;

    // Create the Chrono system and associated collision system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create rover
    cout << "Create rover..." << endl;
    ViperWheelType wheel_type = ViperWheelType::RealWheel;
    ChContactMaterialData wheel_mat(0.4f,   // mu
                                    0.2f,   // cr
                                    2e7f,   // Y
                                    0.3f,   // nu
                                    2e5f,   // kn
                                    40.0f,  // gn
                                    2e5f,   // kt
                                    20.0f   // gt
    );
    ChVector3d init_loc(1.25, 0.0, 0.55);

    // Set constant angular velocity for all wheels (rad/s)
    // This replaces the DC motor control with a simple constant speed control
    double wheel_angular_velocity = 10;  // Approx 2 m/s
    double ramp_time = 2.0;              // 0.5 second ramp-up time to avoid sudden starts

    auto driver = chrono_types::make_shared<ViperSpeedDriver>(ramp_time, wheel_angular_velocity);
    auto rover = chrono_types::make_shared<Viper>(&sys, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(wheel_mat.CreateMaterial(sys.GetContactMethod()));
    rover->Initialize(ChFrame<>(init_loc, QUNIT));

    // Create the CRM terrain system
    double initial_spacing = 0.02;
    CRMTerrain terrain(sys, initial_spacing);
    auto sysFSI = terrain.GetFsiSystemSPH();
    auto sysSPH = terrain.GetFluidSystemSPH();
    sysSPH->EnableCudaErrorCheck(false);
    terrain.SetVerbose(verbose);
    terrain.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
    rover->GetSystem()->SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
    terrain.SetStepSizeCFD(step_size);

    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = density;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = cohesion;
    terrain.SetElasticSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1.3;
    sph_params.free_surface_threshold = 0.8;
    sph_params.artificial_viscosity = 0.2;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.kernel_type = KernelType::WENDLAND;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.25;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.num_proximity_search_steps = 10;
    sph_params.use_variable_time_step = true;  // This makes the step size irrelevant - now we just make sure we are
                                               // within the max of exchange_info (meta step)
    terrain.SetSPHParameters(sph_params);

    // Set output level from SPH simulation
    terrain.SetOutputLevel(OutputLevel::STATE);

    // Add rover wheels as FSI bodies
    cout << "Create wheel BCE markers..." << endl;
    std::string mesh_filename = GetChronoDataFile("robot/viper/obj/viper_cylwheel.obj");
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->materials.push_back(ChContactMaterialData());
    geometry->coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, QUNIT, mesh_filename, VNULL));

    //// TODO: FIX ChFsiProblemSPH to allow rotating geometry on body!!
    for (int i = 0; i < 4; i++) {
        auto wheel_body = rover->GetWheels()[i]->GetBody();
        auto yaw = (i % 2 == 0) ? QuatFromAngleZ(CH_PI) : QUNIT;
        terrain.AddRigidBody(wheel_body, geometry, false);
    }

    terrain.SetActiveDomain(active_box_dim);

    // Construct the terrain
    cout << "Create terrain..." << endl;
    switch (patch_type) {
        case PatchType::RECTANGULAR:
            // Create a rectangular terrain patch
            terrain.Construct({terrain_length, terrain_width, 0.25},  // length X width X height
                              ChVector3d(terrain_length / 2, 0, 0),   // patch center
                              BoxSide::ALL & ~BoxSide::Z_POS          // all boundaries, except top
            );
            break;
        case PatchType::HEIGHT_MAP:
            // Create a patch from a height field map image
            // Generate filename for slope heightmap
            std::string heightmap_filename =
                "terrain/height_maps/slope_" + std::to_string((int)slope_angle) + "deg.bmp";
            // "terrain/height_maps/slope.bmp";
            std::string full_path = GetVehicleDataFile(heightmap_filename.c_str());

            // Check if heightmap file exists
            if (!std::filesystem::exists(std::filesystem::path(full_path))) {
                std::cerr << "Error: Heightmap file not found: " << full_path << std::endl;
                std::cerr << "Please generate the heightmap first using:" << std::endl;
                std::cerr << "  python data/vehicle/terrain/height_maps/generate_slope_heightmap.py "
                          << (int)slope_angle << std::endl;
                return 1;
            }
            double slope_angle_deg = (int)slope_angle;

            double min_height = 0.25;
            double max_height = min_height + (terrain_length - 5) * std::tan(slope_angle_deg * CH_DEG_TO_RAD);

            std::cout << "Loading heightmap: " << heightmap_filename << std::endl;

            terrain.Construct(full_path.c_str(),                     // height map image file
                              terrain_length, terrain_width,         // length (X) and width (Y)
                              {min_height, max_height},              // height range
                              0.25,                                  // depth
                              true,                                  // uniform depth
                              ChVector3d(terrain_length / 2, 0, 0),  // patch center
                              BoxSide::Z_NEG                         // bottom wall
            );
            break;
    }

    // Initialize the terrain system
    terrain.Initialize();

    auto aabb = terrain.GetSPHBoundingBox();
    cout << "  SPH particles:     " << terrain.GetNumSPHParticles() << endl;
    cout << "  Bndry BCE markers: " << terrain.GetNumBoundaryBCEMarkers() << endl;
    cout << "  SPH AABB:          " << aabb.min << "   " << aabb.max << endl;

// Create run-time visualization
#ifdef CHRONO_VSG
    std::shared_ptr<ChVisualSystem> vis;
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleHeightColorCallback>(aabb.min.z(), aabb.max.z());

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::BROWN);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sys);
        visVSG->SetWindowTitle(
            ("Viper rover on CRM deformable terrain - Slope: " + std::to_string((int)slope_angle) + "°").c_str());
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(init_loc + ChVector3d(0, 6, 0.5), init_loc);
        visVSG->SetLightIntensity(0.9f);

        visVSG->Initialize();
        vis = visVSG;
    }
#endif

    // Create output directory
    std::string output_dir = "viper_slope_tests";
    if (!std::filesystem::exists(output_dir)) {
        std::filesystem::create_directory(output_dir);
    }

    // Generate unique filename with simulation parameters
    std::string filename = output_dir + "/viper_data_d" + std::to_string((int)density) + "_f" +
                           std::to_string((int)(friction * 10)) + "_c" + std::to_string((int)(cohesion / 100)) + "_g" +
                           std::to_string((int)(gravity * 10)) + "_s" + std::to_string((int)slope_angle) + ".csv";

    // Start the simulation
    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;
    double exchange_info = 5 * step_size;

    std::ofstream position_file;
    position_file.open(filename);

    // Write CSV header
    position_file << "time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z" << std::endl;
    while (time < tend) {
        rover->Update();

        // Get Viper position and velocity
        ChVector3d pos = rover->GetChassis()->GetPos();
        ChVector3d vel = rover->GetChassis()->GetLinVel();

        // Write position and velocity data to CSV file
        position_file << time << "," << pos.x() << "," << pos.y() << "," << pos.z() << "," << vel.x() << "," << vel.y()
                      << "," << vel.z() << std::endl;

        // Run-time visualization
#ifdef CHRONO_VSG
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }
#endif

        // Advance dynamics of multibody and fluid systems concurrently
        terrain.DoStepDynamics(exchange_info);

        time += exchange_info;
        sim_frame++;
    }

    // Close the position file
    position_file.close();
    std::cout << "Position and velocity data saved to: " << filename << std::endl;

    // terrain.PrintStats();
    // std::string out_dir = GetChronoOutputPath() + "ROBOT_Viper_CRM_Slope" + std::to_string((int)slope_angle) +
    // "deg/"; if (!std::filesystem::create_directory(std::filesystem::path(out_dir))) {
    //     std::cerr << "Error creating directory " << out_dir << std::endl;
    //     return 1;
    // }
    // terrain.PrintTimeSteps(out_dir + "time_steps.txt");

    return 0;
}
