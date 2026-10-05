// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: bgwitt
// =============================================================================
//
// VIPER rover on deformable planetary regolith, seen through Chrono::Sensor.
// The rover drives away from a lunar landing site over a PlanetSCMTerrain, as
// in demo_PLANET_Viper_SCM, with procedural boulders spawned around it. A mast
// camera and a chase camera render the terrain and rocks with the Hapke BRDF
// under a low Sun. ChPlanetVisualMesh keeps the quadtree tiles around the
// rover in a visual shape the cameras render;
// its quadtree is built on the deformed surface, so the ruts SCM publishes
// show up in the images, and ground the wheels have compacted is drawn darker,
// as disturbed regolith is on the Moon. The rover drives a traverse to a goal 250 m to the
// east (--distance <m> moves it), around craters and other steep ground: a route over the whole corridor
// is planned from the terrain at the start, and the path around the rover is
// replanned live along it (PlanetHazardAvoidance). The run ends at the goal.
//
// Command line: GeoTIFF paths form the DEM stack; with none, the best Moon DEM
// resources installed are used (CreateSiteSurface in PlanetDemoSetup.h). Flags: --lambert renders with a
// Lambertian material instead of Hapke, --save writes the camera images to the
// demo output directory (the driven path and planned route are always written
// there), --no-window skips the camera windows, --time <s> sets the simulated duration, --no-rocks skips the
// boulder field, --no-avoidance drives the fixed S-curve of demo_PLANET_Viper_SCM
// instead of avoiding hazards, --bulldozing enables soil displacement at the rut borders, --sun-elevation <deg>
// sets the Sun's elevation (default 12, output then goes to its own directory), --lights adds three
// spotlights on the mast lighting the ground ahead and to either side, --sunset drives a short way and then
// runs a time lapse of the Sun setting as the lights come on (output to its own directory),
// --cpu-raycast forces SCM's CPU ray casting in a build with the SCM GPU backend, --dust has the wheels throw
// up the soil they loosen as a ChDustField the cameras draw (Vulkan RT only). At VIPER's walking pace the
// grains barely leave the ground, as they would on the Moon; demo_PLANET_DustRoosterTail shows a faster wheel.
//
// =============================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/utils/ChOpenMP.h"

#include "chrono_models/robot/viper/Viper.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChPlanetVisualMesh.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_vehicle/terrain/ChDustField.h"
#include "chrono_planet/filters/ChDeformationFilter.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"
#include "chrono_planet/planets/moon/ChMoon.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/terrain/PlanetSCMTerrain.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "PlanetBoulderField.h"
#include "PlanetDust.h"
#include "PlanetHazardAvoidance.h"
#include "PlanetDemoSetup.h"

using namespace chrono;
using namespace chrono::planet;
using namespace chrono::vehicle;
using namespace chrono::viper;
using namespace chrono::sensor;

// Landing site (Apollo 17 region) and the quadtree zoom the physics surface is sampled at.
const double site_lon = 30.75;
const double site_lat = 20.19;
const int zoom = 15;

// Quadtree root tile size (deg), ring half-width (tiles), and the range of terrain given to the sensors (m).
// The sensors upload every triangle in range each time the scene changes, so the range bounds that cost.
const double root_tile_deg = 16.0;
const int view_range_tiles = 4;
const double terrain_range = 1500.0;

// Terrain detail next to the rover. Below the NAC DTM's 5 m, the ground's relief is the Moon preset's
// roughness, down to about 3 cm wavelengths; by default the quadtree stops at cells of about 6 cm (zoom 17),
// which leaves the ground by the rover smooth. max_zoom 19 gives 1.5 cm cells, and a split distance of 2 (in
// tile widths, ChPlanetQuadtree::SetSplitDistance) lets the cameras, 2 to 5 m from the ground, reach it.
const int max_zoom = 19;
const double split_distance = 2.0;
// Split tiles ease in over this long (s, ChPlanetVisualMesh::SetMorphTime). At a walking pace the tiles around
// the cameras split every few seconds, and a long ease leaves them part way to their coarser parent's shape
// much of the time.
const double morph_time = 0.2;

// Physics
const double gravity = moon::kGravity;
// A 2 ms step and 50 solver iterations run the rover in real time. Over a 20 s drive, they keep it within 2 cm of
// its path at 1 ms and 150 iterations, and its ruts within a millimeter of their depth; SCM's CPU and GPU ray casts
// alone differ by 20 cm over that drive.
const double step_size = 2e-3;
const int solver_iterations = 50;

// Soil grid
const double scm_delta = 0.025;
const double rut_stats_period = 2.0;  // sim seconds between rut summaries

// Hazard avoidance: how far east of the site origin the goal is (m, default), how close counts as reached (m),
// and the time between plans (s)
const double default_goal_distance = 250.0;
const double goal_tolerance = 2.0;
const double plan_period = 0.25;

// Boulders: every rock at least this big within this distance of the rover
const double rock_min_radius = 0.15;
const double rock_spawn_radius = 30.0;
const double rock_update_period = 0.2;  // sim seconds between boulder field updates

// VIPER geometry (real wheel)
const double wheel_radius = 0.25;
const double wheel_width = 0.30;

// Sensors. The Sun is low and behind the rover, as it drives east. Its irradiance sets the
// camera exposure.
const float sensor_rate = 10.0f;  // camera updates per second
const double default_sun_elevation = 12;  // deg
const float sun_azimuth = float(200 * CH_DEG_TO_RAD);
const float sun_irradiance = 2.0f;

// Sunset (--sunset): the rover drives sunset_drive along its route and stops, rolling to a halt with its motors
// off (for at most sunset_stop_time). Then, as a time lapse, the physics holds still while the Sun sinks at a
// steady rate from its starting elevation to sunset_end_elevation over lapse_time, dimming as its disk (half a
// degree across) goes below the horizon, and the rover lights come on as it drops from lights_on_elevation to
// the horizon. The scene holds for lapse_hold after that.
const double sunset_drive = 60.0;          // m
const double sunset_stop_time = 5.0;       // s
const double lapse_time = 45.0;            // s
const double lapse_hold = 5.0;             // s
const double sunset_end_elevation = -1.0;  // deg
const double lights_on_elevation = 4.0;    // deg

// Rover lights: spotlights on the mast below its head, fanned out ahead and to either side and angled down.
// Their irradiance falls off with the square of the distance, reaching about the Sun's at 4 m.
const double light_yaws[] = {0.0, 50 * CH_DEG_TO_RAD, -50 * CH_DEG_TO_RAD};
const double light_tilt = 15 * CH_DEG_TO_RAD;    // below the chassis horizontal
const ChVector3d light_mount(0.7, 0.0, 1.45);    // on the mast, in the chassis frame (m)
const double light_offset = 0.2;                 // out from the mast along each light's direction (m)
const float light_intensity = 2.0f;
const float light_range = 40.0f;                 // m
const float light_cone = float(60 * CH_DEG_TO_RAD);     // full cone angle
const float light_falloff = float(40 * CH_DEG_TO_RAD);  // full angle of the undimmed core

// Lunar regolith Hapke parameters (as in demo_ROBOT_Viper_SCM_Sensor): single-scattering albedo, phase function
// shape and back/forward weight, for the ground and the dust alike
const float hapke_w = 0.32357f, hapke_b = 0.23955f, hapke_c = 0.30452f;

std::shared_ptr<ChVisualMaterial> CreateRegolithMaterial(bool hapke, const ChColor& albedo) {
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetAmbientColor({0.0f, 0.0f, 0.0f});
    mat->SetDiffuseColor(albedo);
    mat->SetSpecularColor({0.0f, 0.0f, 0.0f});
    mat->SetRoughness(1.0f);
    mat->SetMetallic(0.0f);
    if (hapke) {
        // Lunar regolith parameters, as in demo_ROBOT_Viper_SCM_Sensor.
        mat->SetBSDF(BSDFType::HAPKE);
        mat->SetHapkeParameters(hapke_w, hapke_b, hapke_c, 1.80238f, 0.07145f, 0.3f, float(23.4 * CH_DEG_TO_RAD));
    }
    return mat;
}

void AddCamera(ChSensorManager& manager,
               std::shared_ptr<ChBody> mount,
               const ChFrame<double>& pose,
               const std::string& name,
               bool window,
               const std::string& save_dir) {
    auto cam = chrono_types::make_shared<ChCameraSensor>(mount, sensor_rate, pose, 1280, 720, float(70 * CH_DEG_TO_RAD));
    cam->SetName(name);
    cam->SetLag(0.f);
    cam->SetCollectionWindow(0.f);
    if (window)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, name));
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    if (!save_dir.empty())
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(save_dir + name + "/"));
    manager.AddSensor(cam);
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    bool hapke = true;
    bool save = false;
    bool window = true;
    double end_time = 900.0;
    double goal_distance = default_goal_distance;
    bool use_rocks = true;
    bool avoidance = true;
    bool bulldozing = false;
    bool lights = false;
    bool sunset = false;
    double sun_elevation_deg = default_sun_elevation;
    bool cpu_raycast = false;
    bool use_dust = false;
    std::vector<ChGeoTiffSource> dems;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--lambert"))
            hapke = false;
        else if (!std::strcmp(argv[i], "--save"))
            save = true;
        else if (!std::strcmp(argv[i], "--no-window"))
            window = false;
        else if (!std::strcmp(argv[i], "--time") && i + 1 < argc)
            end_time = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--distance") && i + 1 < argc)
            goal_distance = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--no-rocks"))
            use_rocks = false;
        else if (!std::strcmp(argv[i], "--cpu-raycast"))
            cpu_raycast = true;
        else if (!std::strcmp(argv[i], "--dust"))
            use_dust = true;
        else if (!std::strcmp(argv[i], "--no-avoidance"))
            avoidance = false;
        else if (!std::strcmp(argv[i], "--sun-elevation") && i + 1 < argc)
            sun_elevation_deg = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--bulldozing"))
            bulldozing = true;
        else if (!std::strcmp(argv[i], "--lights"))
            lights = true;
        else if (!std::strcmp(argv[i], "--sunset"))
            sunset = lights = true;
        else
            dems.push_back({argv[i], 0, 30});
    }
    const ChVector2d goal(goal_distance, 0.0);
    std::cout << "Material: " << (hapke ? "Hapke" : "Lambertian") << ", " << end_time << " s, goal " << goal_distance
              << " m east" << std::endl;

    // Output directory
    const std::string out_dir = GetChronoOutputPath() + "PLANET_Viper_SCM_Sensor/";
    // A Sun other than the default gets its own directory, so runs under both keep their output.
    std::string run_dir = out_dir + (hapke ? "hapke" : "lambert");
    if (sun_elevation_deg != default_sun_elevation) {
        std::ostringstream suffix;
        suffix << "_sun" << sun_elevation_deg;
        run_dir += suffix.str();
    }
    if (sunset)
        run_dir += "_sunset";
    run_dir += "/";
    const std::string save_dir = save ? run_dir : "";
    if (!CreateOutputDirectory(out_dir) || !CreateOutputDirectory(run_dir)) {
        std::cerr << "Error creating directory " << run_dir << std::endl;
        return 1;
    }

    // Surface and site frame, one surface for physics and rendering so the wheels ride the rendered ground
    auto surface = CreateSiteSurface(dems, zoom, root_tile_deg);
    if (!surface)
        return 1;
    ChSiteFrame site = surface->MakeSiteFrame(site_lon, site_lat);
    std::cout << "Site origin elevation: " << site.GetOriginElevation() << " m" << std::endl;

    // The rendered terrain is a view of the physics surface with the soil deformation on top. SCM samples the
    // undeformed surface; the view adds the ruts it publishes, and the quadtree rebuilds the tiles they touch.
    auto deformation = chrono_types::make_shared<ChDeformationFilter>(site, scm_delta);
    auto drawn_surface = surface->CreateView(deformation);
    auto world = chrono_types::make_shared<ChPlanetQuadtree>(drawn_surface, view_range_tiles);
    world->SetSplitDistance(split_distance);
    world->SetMaxZoom(max_zoom);

    // Multibody system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(solver_iterations);
    // SCM ray-casts every wheel's grid nodes in parallel on Chrono's threads. Gains flatten past about 8 threads.
    sys.SetNumThreads(std::min(8, ChOMP::GetNumProcs()), 1, 4);

    auto rock_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    rock_mat->SetFriction(0.8f);
    rock_mat->SetRestitution(0.01f);

    // Rover, wheels resting on the undeformed surface
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    Viper viper(&sys, ViperWheelType::RealWheel);
    viper.SetDriver(driver);
    viper.SetWheelContactMaterial(rock_mat);

    auto height_at = [&](double x, double y) {
        double lon, lat;
        site.ToLonLat(x, y, lon, lat);
        return surface->GetElevation(lon, lat) - site.GetOriginElevation();
    };
    // Plan the route over the whole corridor from the terrain, and start the rover pointed along it: VIPER cannot
    // turn tightly, and the site has a crater two meters from the start.
    PlanetHazardAvoidance planner(height_at);
    planner.SetGoal(goal);
    double start_heading = 0;
    if (avoidance) {
        planner.BuildRoute(ChVector2d(0, 0));
        for (const auto& pt : planner.GetGlobalRoute(ChVector2d(0, 0))) {
            if (pt.Length() >= 3.0) {
                start_heading = std::atan2(pt.y(), pt.x());
                break;
            }
        }
    }
    const ChQuaternion<> start_rot = QuatFromAngleZ(start_heading);
    double ground = -1e9;
    for (double wx : {-0.64, 0.64})
        for (double wy : {-0.61, 0.61}) {
            const ChVector3d wheel = start_rot.Rotate(ChVector3d(wx, wy, 0));
            ground = std::max(ground, height_at(wheel.x(), wheel.y()));
        }
    viper.Initialize(ChFrame<>(ChVector3d(0, 0, ground + wheel_radius + 0.05), start_rot));
    auto chassis = viper.GetChassis()->GetBody();

    const std::array<ViperWheelID, 4> wheel_ids = {V_LF, V_RF, V_LB, V_RB};
    std::vector<PlanetSCMTerrain::Wheel> wheels;
    for (auto id : wheel_ids)
        wheels.push_back({viper.GetWheel(id)->GetBody(), wheel_radius, wheel_width});

    // Deformable terrain
    PlanetSCMTerrain terrain(&sys, surface, site);
    PlanetSCMTerrain::Params params;
    params.delta = scm_delta;
    // Soft, loose regolith, so the rover leaves clear ruts: the lunar set of demo_ROBOT_Viper_SCM_Sensor with
    // half its stiffnesses and 1 kPa of cohesion, the upper end of Apollo estimates for the lunar surface.
    params.bekker_kphi = 0.1e6;   // frictional modulus (Pa/m^n)
    params.bekker_kc = 0;         // cohesive modulus (Pa/m^(n-1))
    params.bekker_n = 1.1;        // sinkage exponent
    params.mohr_cohesion = 1e3;   // cohesion (Pa)
    params.mohr_friction = 30;    // internal friction angle (deg)
    params.janosi_shear = 0.01;   // Janosi shear displacement (m)
    params.elastic_k = 2e7;       // elastic stiffness (Pa/m), must exceed bekker_kphi
    params.damping_r = 3e4;      // vertical damping (Pa s/m)
    params.bulldozing = bulldozing;
    terrain.Initialize(params, wheels);
    terrain.SetDeformationFilter(deformation);
#ifdef CHRONO_HAS_SCM_GPU
    if (cpu_raycast)
        terrain.EnableRaycastGpuHip(false);
#else
    if (cpu_raycast)
        std::cout << "--cpu-raycast has no effect: this build has no SCM GPU backend" << std::endl;
#endif

    // Terrain and rocks as the sensors see them
    // Compacting regolith breaks up its porous surface, so tracks look darker than the ground around them.
    const ChColor regolith_albedo(0.7f, 0.7f, 0.7f);
    const float compacted_darkening = 0.7f;  // albedo of compacted ground, relative to undisturbed
    const double compacted_depth = 0.01;     // lowering that marks ground as compacted (m)
    ChPlanetVisualMesh visual_terrain(&sys, world, site);
    visual_terrain.SetMaterial(CreateRegolithMaterial(hapke, regolith_albedo));
    const ChColor compacted_albedo(regolith_albedo.R * compacted_darkening, regolith_albedo.G * compacted_darkening,
                                   regolith_albedo.B * compacted_darkening);
    visual_terrain.SetCompaction(deformation, CreateRegolithMaterial(hapke, compacted_albedo), compacted_depth);
    visual_terrain.SetMaxDistance(terrain_range);
    visual_terrain.SetMorphTime(morph_time);
    visual_terrain.Update(chassis->GetPos(), 0.0);

    PlanetBoulderField boulders(&sys, surface, site, rock_mat, rock_min_radius);
    boulders.SetVisualMaterial(CreateRegolithMaterial(hapke, ChColor(0.55f, 0.53f, 0.5f)));
    if (use_rocks)
        boulders.Update(chassis->GetPos(), rock_spawn_radius);
    std::cout << "Boulders around the site: " << boulders.GetNumRocks() << std::endl;

    // Sensors: sunlight only, with no ambient term, under a black sky
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    // The Sun's elevation (deg) at a time, its irradiance at an elevation, and the rover lights' share of full
    // brightness at a Sun elevation
    enum class Phase { DRIVE, STOP, LAPSE };
    Phase phase = Phase::DRIVE;
    double phase_start = 0;  // time the current phase began (s)
    auto sun_elevation_at = [&](double t) {
        if (phase != Phase::LAPSE)
            return sun_elevation_deg;
        return sun_elevation_deg + std::clamp((t - phase_start) / lapse_time, 0.0, 1.0) * (sunset_end_elevation - sun_elevation_deg);
    };
    auto sun_color_at = [&](double elevation) {
        const float e = sun_irradiance * float(std::clamp(elevation + 0.5, 0.0, 1.0));
        return ChColor(e, e, e);
    };
    auto lights_share_at = [&](double elevation) {
        if (!sunset)
            return 1.0;
        const double s = std::clamp(1.0 - elevation / lights_on_elevation, 0.0, 1.0);
        return s * s * (3 - 2 * s);
    };
    const unsigned int sun = manager->scene->AddDirectionalLight(sun_color_at(sun_elevation_at(0)),
                                                                 float(sun_elevation_at(0) * CH_DEG_TO_RAD), sun_azimuth);
    manager->scene->SetAmbientLight({0.f, 0.f, 0.f});
    Background background;
    background.mode = BackgroundMode::SOLID_COLOR;
    background.color_zenith = ChVector3f(0.f, 0.f, 0.f);
    background.color_horizon = ChVector3f(0.f, 0.f, 0.f);
    manager->scene->SetBackground(background);

    // Rover lights, if requested, moved with the chassis before each sensor update
    struct RoverLight {
        unsigned int id;
        ChVector3d pos, dir;  // in the chassis reference frame
    };
    std::vector<RoverLight> rover_lights;
    ChColor light_color(light_intensity, light_intensity, light_intensity);
    if (lights) {
        for (double yaw : light_yaws) {
            const ChVector3d dir(std::cos(light_tilt) * std::cos(yaw), std::cos(light_tilt) * std::sin(yaw), -std::sin(light_tilt));
            const ChVector3d pos = light_mount + ChVector3d(std::cos(yaw), std::sin(yaw), 0) * light_offset;
            const unsigned int id = manager->scene->AddSpotLight(ChVector3f(0, 0, 0), light_color, light_range, ChVector3f(1, 0, 0),
                                                                 light_falloff, light_cone, false);
            rover_lights.push_back({id, pos, dir});
        }
    }
    auto place_lights = [&](double share) {
        light_color = ChColor(float(share) * light_intensity, float(share) * light_intensity, float(share) * light_intensity);
        // The chassis reference frame, which the cameras are mounted on, not its center of mass frame
        const ChFrame<> frame = chassis->GetVisualModelFrame();
        for (const auto& light : rover_lights) {
            const ChVector3d pos = frame.TransformPointLocalToParent(light.pos);
            const ChVector3d dir = frame.TransformDirectionLocalToParent(light.dir);
            manager->scene->ModifySpotLight(light.id, ChVector3f(pos), light_color, light_range, ChVector3f(dir), light_falloff,
                                            light_cone, false);
        }
    };

    // Dust the wheels throw up, over the undeformed ground
    std::unique_ptr<ChDustField> dust;
    if (use_dust) {
        ChDustField::Params dust_params;
        dust_params.gravity = gravity;
        dust = chrono_types::make_unique<ChDustField>(dust_params, height_at);
    }
    auto sun_dir_at = [&](double elevation_deg) {
        const double e = elevation_deg * CH_DEG_TO_RAD;
        return ChVector3d(std::cos(e) * std::cos(sun_azimuth), std::cos(e) * std::sin(sun_azimuth), std::sin(e));
    };
    std::vector<ChDustField::WheelState> wheel_states(wheels.size());

    // A mast camera just ahead of the mast head, looking ahead and down, and a chase camera behind and left
    // of the rover, looking at it. The chassis body spans about x in [-0.75, 1.0] and z up to 1.2 m, and the
    // mast head sits near x = 0.5 to 0.9 m, z = 1.6 to 1.9 m in its frame.
    AddCamera(*manager, chassis, ChFrame<>(ChVector3d(0.95, 0, 1.75), QuatFromAngleY(20 * CH_DEG_TO_RAD)), "mast", window,
              save_dir);
    AddCamera(*manager, chassis,
              ChFrame<>(ChVector3d(-5.0, -2.5, 2.2), QuatFromAngleZ(std::atan2(2.5, 5.0)) * QuatFromAngleY(19 * CH_DEG_TO_RAD)),
              "chase", window, save_dir);

    // Simulation loop. Ruts and the terrain level of detail are refreshed once per camera update.
    const int sensor_steps = static_cast<int>(std::round(1.0 / (sensor_rate * step_size)));
    const int rock_steps = static_cast<int>(std::round(rock_update_period / step_size));
    const int report_steps = static_cast<int>(std::round(1.0 / step_size));
    const auto wall_start = std::chrono::steady_clock::now();
    double distance = 0;
    double next_rut_request = rut_stats_period;

    // Steering: paths planned over the undeformed surface, replanned from the rover's pose
    PlanetHazardAvoidance::Plan plan;
    double next_plan = 0;
    double max_tilt = 0;
    // The chassis frame is not z-up, so tilt is measured from the chassis's up direction at the start.
    const ChVector3d chassis_up = chassis->GetRot().RotateBack(ChVector3d(0, 0, 1));
    std::ofstream path_file(run_dir + "path.csv");
    path_file << "time x y heading steering max_slope_ahead blocked tilt" << std::endl;
    std::optional<PlanetSCMTerrain::RutStats> rut;
    ChVector3d last_pos = chassis->GetPos();
    int step = 0;
    while (sys.GetChTime() < end_time) {
        const double time = sys.GetChTime();
        const ChVector3d pos = chassis->GetPos();

        // Sunset sequence: stop once far enough, then the time lapse once the rover is still
        if (sunset && phase == Phase::DRIVE && distance >= sunset_drive) {
            phase = Phase::STOP;
            phase_start = time;
            for (auto id : wheel_ids)
                driver->SetMotorStallTorque(0.0, id);
            std::cout << "Stopping at t = " << time << " s after " << distance << " m" << std::endl;
        }
        if (phase == Phase::STOP && (chassis->GetPosDt().Length() < 0.01 || time - phase_start > sunset_stop_time)) {
            phase = Phase::LAPSE;
            phase_start = time;
            std::cout << "Sunset from t = " << time << " s, rover at (" << pos.x() << ", " << pos.y() << ") m" << std::endl;
        }
        if (phase == Phase::LAPSE && time - phase_start > lapse_time + lapse_hold)
            break;

        if (step % sensor_steps == 0) {
            terrain.PublishDeformation();
            visual_terrain.Update(pos, time);
            const double elevation = sun_elevation_at(time);
            if (sunset)
                manager->scene->ModifyDirectionalLight(sun, sun_color_at(elevation), float(elevation * CH_DEG_TO_RAD), sun_azimuth);
            place_lights(lights_share_at(elevation));
            if (dust) {
                dust->Update(time);
                dust->UpdateGrid(time, ChVector3d(pos.x(), pos.y(), height_at(pos.x(), pos.y())), sun_dir_at(elevation));
                if (!SetDustVolume(*manager, *dust, hapke_w, hapke_b, hapke_c, regolith_albedo) && step == 0)
                    std::cout << "This render backend does not draw the dust; only Vulkan RT does." << std::endl;
            }
        }
        manager->Update();

        // Chassis tilt: the angle between its up direction and the vertical
        const double tilt = std::acos(std::clamp(chassis->GetRot().Rotate(chassis_up).z(), -1.0, 1.0));
        max_tilt = std::max(max_tilt, tilt);

        if (phase != Phase::DRIVE) {
            // Stopped: no more steering or planning
        } else if (avoidance) {
            if (time >= next_plan) {
                const ChVector3d forward = chassis->GetRot().GetAxisX();
                const double heading = std::atan2(forward.y(), forward.x());
                plan = planner.Update(ChVector2d(pos.x(), pos.y()), heading, plan_period);
                driver->SetSteering(plan.steering);
                next_plan += plan_period;
                path_file << time << " " << pos.x() << " " << pos.y() << " " << heading << " " << plan.steering << " "
                          << plan.max_slope << " " << plan.blocked << " " << tilt << std::endl;
                if (next_plan == plan_period) {
                    // The first plan builds the route over the whole corridor.
                    std::ofstream route_file(run_dir + "route.csv");
                    route_file << "index x y" << std::endl;
                    const auto route = planner.GetGlobalRoute(ChVector2d(pos.x(), pos.y()));
                    for (size_t r = 0; r < route.size(); ++r)
                        route_file << r << " " << route[r].x() << " " << route[r].y() << std::endl;
                }
                if ((ChVector2d(pos.x(), pos.y()) - goal).Length() < goal_tolerance) {
                    std::cout << "Goal reached at t = " << time << " s after " << distance << " m" << std::endl;
                    break;
                }
            }
        } else {
            // Steering schedule: straight for 10 s, then a gentle S-curve
            driver->SetSteering(time < 10.0 ? 0.0 : 0.10 * std::sin(0.20 * (time - 10.0)));
            if (time >= next_plan) {
                path_file << time << " " << pos.x() << " " << pos.y() << " 0 0 0 0 " << tilt << std::endl;
                next_plan += plan_period;
            }
        }

        // In the time lapse only the clock moves, for the sensors
        if (phase == Phase::LAPSE) {
            sys.SetChTime(time + step_size);
        } else {
            if (dust) {
                for (size_t w = 0; w < wheels.size(); ++w)
                    wheel_states[w] = DustWheelState(wheels[w].body, wheel_radius, wheel_width);
                dust->EmitFromWheels(wheel_states, time, step_size);
            }
            sys.DoStepDynamics(step_size);
            viper.Update();
        }

        // Rut summary, computed on a worker thread
        if (time >= next_rut_request) {
            terrain.RequestRutStats();
            next_rut_request += rut_stats_period;
        }
        if (auto stats = terrain.TakeRutStats())
            rut = stats;

        // Boulders around the rover
        if (use_rocks && step % rock_steps == 0)
            boulders.Update(pos, rock_spawn_radius);

        distance += (pos - last_pos).Length();
        last_pos = pos;
        if (++step % report_steps == 0) {
            const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
            std::cout << "t = " << time << " s (" << time / wall << "x real time), at (" << pos.x() << ", " << pos.y()
                      << ") m, distance " << distance << " m, " << visual_terrain.GetNumTiles() << " tiles ("
                      << visual_terrain.GetNumTriangles() << " triangles), " << visual_terrain.GetNumRebuilds()
                      << " terrain rebuilds, " << deformation->GetNumNodes() << " deformed nodes, " << world->GetNumRebuiltTiles()
                      << " tiles rebuilt for ruts, " << boulders.GetNumRocks()
                      << " rocks, max tilt " << max_tilt * CH_RAD_TO_DEG << " deg";
            if (avoidance)
                std::cout << ", steering " << plan.steering << (plan.blocked ? " (blocked)" : "");
            if (rut)
                std::cout << ", rut area " << rut->area_m2 << " m2, max depth " << rut->max_depth_m << " m";
            if (dust)
                std::cout << ", dust " << dust->GetStats().aloft * 1e3 << " g aloft in " << dust->GetStats().particles << " particles";
            std::cout << std::endl;
        }
    }

    std::cout << "Driven path in " << run_dir << "path.csv" << std::endl;
    if (save)
        std::cout << "Sensor data saved to " << save_dir << std::endl;
    return 0;
}
