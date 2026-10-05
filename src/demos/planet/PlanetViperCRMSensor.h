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
// Shared by the VIPER-on-CRM sensor demos (demo_PLANET_Viper_CRM_Sensor,
// demo_PLANET_Viper_CRM_HillClimb, demo_PLANET_Viper_CRM_CraterFording), which
// differ only in their ViperCRMScenario: RunViperCRMSensor builds and runs the
// scene below.
//
// The scene: a VIPER rover on CRM regolith at a lunar landing site, seen through
// Chrono::Sensor. The soil under the rover is CRM in a window that follows it
// (vehicle::PlanetCRMWindow), as in demo_PLANET_Viper_CRM; the cameras see the
// planet's terrain (ChPlanetVisualMesh) with the ruts the wheels leave, darker
// where the wheels compacted the ground, the berms of soil they push up beside
// the ruts, boulders, and the soil they throw up as dust (Vulkan RT only), all
// shaded with the Hapke BRDF under a low Sun and a black sky.
//
// The soil's particles are not drawn as such: each stands for tens of grams of
// soil, and drawn as a sphere would look like a pebble. Soil still on the
// ground is drawn as ground (ruts and berms). Dust is thrown off the wheels as
// in demo_PLANET_Viper_SCM_Sensor (ChDustField::EmitFromWheels), at the rate
// the wheels' speed and slip over the CRM soil give; with
// --dust-model hybrid (the default) throws soil off the tread as the kinematic
// wheel model does (shed progressively, some wrapping over the wheel), as much as
// the wheel's sinkage and driving slip in the CRM soil give; --dust-model physics
// takes the dust from physics alone: soil carried
// in the tread's grooves, held there by the soil's cohesion and released where
// the pull out of the groove exceeds it (ChDustField::WheelEmission::cohesion),
// and soil the CRM soil throws into free flight at each wheel, measured and
// emitted smoothly (PlanetCRMWindow::MeasureEjecta, ChDustField::EmitSource);
// --dust-model kinematic keeps the tuned kinematic wheel model. With
// --dust-model particles (or --dust-from-particles), the soil's particles in free flight are handed to
// the dust field instead (PlanetCRMWindow::EmitDust), which follows the soil
// but, at tens of grams a particle, puffs.
//
// Flags (defaults in parentheses are demo_PLANET_Viper_CRM_Sensor's; the time,
// rate, Sun, speed, camera, start and friction defaults are the scenario's):
// --spacing <m> (default 0.03) particle spacing, --time <s> simulated
// duration (default 30), --save writes the camera images to the demo output
// directory, --no-window skips the camera windows (the cameras render
// offscreen either way), --rate <Hz> camera updates per second (default 10),
// --friction (default 0.5) and --cohesion <Pa> (default 200) soil,
// --rut-darkening <x> albedo of the most compacted ground relative to
// undisturbed (default 0.5), --rut-flatten <x> share of the fine relief pressed
// out of rut floors (default 0.6),
// --sun-elevation <deg> (default 12), --sun-azimuth <deg> the Sun's azimuth
// from the rover's start heading (default 140, behind it and to its left), --lambert shades with a Lambertian
// material instead of Hapke, --no-rocks skips the boulder field, --no-dust
// and --no-berms leave those out, --no-gi leaves out the light the regolith
// bounces into the shadows, --samples <n> rays per pixel, rounded to a square
// (default 4), --speed
// <rad/s> wheel speed (default 0.4, about 0.1 m/s; faster wheels throw soil up),
// --camera chase|mast|both|side (default both), --fine-roughness <x> slope growth per
// octave of the ground's fine roughness (default 1.2; the Moon preset's is 1.4),
// --compaction-depth <m> lowering drawn as compacted, darker ground (default 0.003),
// --dust-voxel <m> (default 0.025) and --dust-blur <passes> (default 0) set the
// dust grid, --start <x> <y> <heading deg> sets where the rover starts (the
// Sun keeps its azimuth relative to the start heading). --noise-tolerance <steps> lets the cameras stop
// sampling a pixel once its noise is below that many 8-bit steps (default 0.5;
// 0 takes every sample), --profile reports where the wall time goes each second, and
// --gpu-checks turns on the CRM solver's checks for GPU errors, --no-gi-cache traces
// a GI ray per camera sample instead of taking the GI from a cache
// (ChCameraSensor::SetGICache; with it, use --samples 64), --gi-rays <n>
// sets the cache's GI rays per texel (default 128).
//
// =============================================================================

#ifndef DEMO_PLANET_VIPER_CRM_SENSOR_H
#define DEMO_PLANET_VIPER_CRM_SENSOR_H

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_models/robot/viper/Viper.h"

#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChPlanetVisualMesh.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_vehicle/terrain/ChDustField.h"
#include "chrono_planet/filters/ChDeformationFilter.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"
#include "chrono_planet/planets/moon/ChMoon.h"

#include "chrono_vehicle/terrain/PlanetCRMWindow.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "PlanetBoulderField.h"
#include "PlanetDemoSetup.h"
#include "PlanetDust.h"

using namespace chrono;
using namespace chrono::planet;
using namespace chrono::vehicle;
using namespace chrono::viper;
using namespace chrono::sensor;
using namespace chrono::fsi::sph;

/// What a run of RunViperCRMSensor shows: where the rover starts and how fast its wheels turn, how long, the cameras,
/// and the Sun. The command-line flags override any of these.
struct ViperCRMScenario {
    std::string name = "PLANET_Viper_CRM_Sensor";  ///< output directory under the demo output path
    double start_x = 24, start_y = 42;              ///< rover start in the site frame (m): the smoothest drive nearby
    double start_heading = 120;                     ///< rover heading at the start (deg from east)
    double speed = 0.4;                             ///< wheel speed once settled (rad/s): 0.4 is about 0.1 m/s
    double end_time = 30;                           ///< simulated duration (s)
    float rate = 10;                                ///< camera updates per second
    std::string camera = "both";                    ///< chase, mast, both or side
    double sun_elevation = 12;                      ///< Sun elevation (deg)
    double sun_azimuth = 140;                       ///< Sun azimuth relative to the start heading (deg)
    double friction = 0.5;                          ///< friction of the loose surface regolith
};

namespace viper_crm_sensor {

// Landing site (Apollo 17 region), quadtree zoom of the physics surface, and where the rover starts in the site
// frame (m) and its heading (deg from east), set from the scenario
const double site_lon = 30.75;
const double site_lat = 20.19;
const int zoom = 15;
double start_x, start_y, start_heading;

// Rendered terrain: quadtree root tile (deg), ring half-width (tiles), range given to the sensors (m), and detail
// next to the rover (see demo_PLANET_Viper_SCM_Sensor)
const double root_tile_deg = 16.0;
const int view_range_tiles = 4;
const double terrain_range = 1500.0;
const int max_zoom = 19;
const double split_distance = 2.0;
const double morph_time = 0.2;

const double gravity = moon::kGravity;

// Soil: window following the rover, time steps, and loose surface regolith (demo_PLANET_Viper_CRM)
const double window_length = 5.0, window_width = 3.5, window_depth = 0.25, window_margin = 1.3;
const double cfd_step = 5e-4;
const double exchange_step = 2.5e-3;
const double rut_spacing = 0.025;

// Wheels as the soil sees them (the real wheel's rim and grousers), driven at VIPER's traverse pace once settled
const double wheel_radius = 0.245, wheel_width = 0.29;
double drive_speed;  // rad/s
const double settle_time = 2.0;

// Pixel noise (8-bit steps) at which a camera stops sampling a pixel (ChCameraSensor::SetNoiseTolerance), 0 for every
// sample: at 64 samples per pixel, sunlit ground stops after 16 (below 16, as with the GI cache, it has no effect)
float noise_tolerance = 0.5f;

// Take the cameras' GI from a cache traced at a quarter of the resolution (ChCameraSensor::SetGICache): the samples
// per pixel are then left for the edges, and 4 give shadows as smooth as 64 GI rays per pixel did, in about a
// twentieth of the time
bool gi_cache = true;
unsigned int gi_cache_rays = 0;  // GI rays per cache texel (ChCameraSensor::SetGICacheRays), 0 for the default

// Boulders around the rover
const double rock_min_radius = 0.15;
const double rock_spawn_radius = 30.0;
const double rock_update_period = 0.2;

// Soil thrown up is caught this often (s)
const double dust_period = 0.01;

// The Sun, its azimuth relative to the start heading (deg); its irradiance sets the camera exposure
double sun_azimuth_offset;
inline float SunAzimuth() {
    return float((start_heading + sun_azimuth_offset) * CH_DEG_TO_RAD);
}
const float sun_irradiance = 2.0f;

// Lunar regolith Hapke parameters (demo_PLANET_Viper_SCM_Sensor)
const float hapke_w = 0.32357f, hapke_b = 0.23955f, hapke_c = 0.30452f;

// Wheels held still while the rover settles into the soil, then brought up to speed over a second
class SettleThenDrive : public ViperDriver {
  public:
    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::SPEED; }
    virtual void Update(double time) override {
        const double speed = drive_speed * std::clamp(time - settle_time, 0.0, 1.0);
        drive_speeds = {speed, speed, speed, speed};
    }
};

inline std::shared_ptr<ChVisualMaterial> CreateRegolithMaterial(bool hapke, const ChColor& albedo, double roughness_deg = 23.4) {
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetAmbientColor({0.0f, 0.0f, 0.0f});
    mat->SetDiffuseColor(albedo);
    mat->SetSpecularColor({0.0f, 0.0f, 0.0f});
    mat->SetRoughness(1.0f);
    mat->SetMetallic(0.0f);
    if (hapke) {
        mat->SetBSDF(BSDFType::HAPKE);
        mat->SetHapkeParameters(hapke_w, hapke_b, hapke_c, 1.80238f, 0.07145f, 0.3f, float(roughness_deg * CH_DEG_TO_RAD));
    }
    return mat;
}

inline void AddCamera(ChSensorManager& manager,
               std::shared_ptr<ChBody> mount,
               float rate,
               const ChFrame<double>& pose,
               const std::string& name,
               bool window,
               const std::string& save_dir,
               unsigned int samples,
               bool gi) {
    // Light bounced off the sunlit regolith lights the shadows (gi); several samples per pixel smooth its noise. The
    // camera takes a supersampling factor per axis: factor^2 samples per pixel.
    const unsigned int factor = std::max(1u, static_cast<unsigned int>(std::lround(std::sqrt(double(samples)))));
    auto cam = chrono_types::make_shared<ChCameraSensor>(mount, rate, pose, 1280, 720, float(70 * CH_DEG_TO_RAD), factor,
                                                         CameraLensModelType::PINHOLE, gi);
    cam->SetName(name);
    cam->SetLag(0.f);
    cam->SetCollectionWindow(0.f);
    cam->SetNoiseTolerance(noise_tolerance);
    cam->SetGICache(gi_cache);
    cam->SetGICacheRays(gi_cache_rays);
    if (window)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, name));
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    if (!save_dir.empty())
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(save_dir + name + "/"));
    manager.AddSensor(cam);
}

// The coarse grains of a dust field in flight as a mesh of small, irregular octahedra (clods a few millimeters across,
// larger than a single grain so a camera sees them), for the cameras to draw as flung soil. The finer grains are left
// to the dust volume.
inline std::shared_ptr<ChTriangleMeshConnected> DustGrainMesh(const ChDustField& dust, double time, int bin, size_t max_grains) {
    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    auto& v = mesh->GetCoordsVertices();
    auto& f = mesh->GetIndicesVertices();
    size_t n = 0;
    uint32_t index = 0;
    for (const auto& p : dust.GetParticles()) {
        ++index;
        if (p.bin != bin || p.time0 > time)
            continue;
        if (++n > max_grains)
            break;
        const ChVector3d c = dust.Position(p, time);
        // a size and a shape of its own, the same from frame to frame
        uint32_t h = index * 2654435761u;
        auto next = [&h]() {
            h ^= h << 13, h ^= h >> 17, h ^= h << 5;
            return (h & 0xffff) / 65535.0;
        };
        const double r = 0.001 + 0.0015 * next();
        const int base = static_cast<int>(v.size());
        const ChVector3d axes[3] = {ChVector3d(1, 0.3 * next(), 0.3 * next()).GetNormalized(), ChVector3d(0.3 * next(), 1, 0.3 * next()).GetNormalized(),
                                    ChVector3d(0.3 * next(), 0.3 * next(), 1).GetNormalized()};
        for (int k = 0; k < 3; ++k) {
            v.push_back(c + axes[k] * r * (0.7 + 0.6 * next()));
            v.push_back(c - axes[k] * r * (0.7 + 0.6 * next()));
        }
        const int faces[8][3] = {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4}, {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5}};
        for (const auto& t : faces)
            f.push_back(ChVector3i(base + t[0], base + t[1], base + t[2]));
    }
    return mesh;
}

}  // namespace viper_crm_sensor

/// Run the VIPER-on-CRM scene seen through Chrono::Sensor for a scenario, the command-line flags overriding it (see
/// the flags at the top of this file).
inline int RunViperCRMSensor(int argc, char* argv[], const ViperCRMScenario& scenario) {
    using namespace viper_crm_sensor;
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    start_x = scenario.start_x, start_y = scenario.start_y, start_heading = scenario.start_heading;
    drive_speed = scenario.speed;
    sun_azimuth_offset = scenario.sun_azimuth;
    double spacing = 0.03;
    double end_time = scenario.end_time;
    bool save = false;
    bool window = true;
    float rate = scenario.rate;
    double sun_elevation = scenario.sun_elevation;
    bool hapke = true;
    bool use_rocks = true;
    bool use_dust = true;
    bool berms = true;
    unsigned int samples = 4;
    bool gi = true;
    std::string cameras = scenario.camera;
    double fine_boost = 1.2;
    double compaction_depth = 0.003;
    double rut_darkening = 0.5;  // albedo of compacted ground, relative to undisturbed
    double flatten_strength = 0.6;  // share of the fine relief pressed out of rut floors
    double friction = scenario.friction, cohesion = 200;  // loose surface regolith (demo_PLANET_Viper_CRM)
    double dust_voxel = 0.025;
    int dust_blur = 0;
    bool profile = false;
    bool gpu_checks = false;  // the CRM solver's checks for GPU errors, a wait on the GPU after each of its kernels
    std::string dust_model = "hybrid";  // hybrid, physics, kinematic or particles
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--spacing") && i + 1 < argc)
            spacing = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--time") && i + 1 < argc)
            end_time = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--save"))
            save = true;
        else if (!std::strcmp(argv[i], "--no-window"))
            window = false;
        else if (!std::strcmp(argv[i], "--rate") && i + 1 < argc)
            rate = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--rut-flatten") && i + 1 < argc)
            flatten_strength = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--rut-darkening") && i + 1 < argc)
            rut_darkening = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--friction") && i + 1 < argc)
            friction = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--cohesion") && i + 1 < argc)
            cohesion = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--sun-azimuth") && i + 1 < argc)
            sun_azimuth_offset = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--sun-elevation") && i + 1 < argc)
            sun_elevation = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--lambert"))
            hapke = false;
        else if (!std::strcmp(argv[i], "--no-rocks"))
            use_rocks = false;
        else if (!std::strcmp(argv[i], "--no-dust"))
            use_dust = false;
        else if (!std::strcmp(argv[i], "--no-berms"))
            berms = false;
        else if (!std::strcmp(argv[i], "--samples") && i + 1 < argc)
            samples = static_cast<unsigned int>(std::atoi(argv[++i]));
        else if (!std::strcmp(argv[i], "--speed") && i + 1 < argc)
            drive_speed = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--start") && i + 3 < argc) {
            start_x = std::atof(argv[++i]);
            start_y = std::atof(argv[++i]);
            start_heading = std::atof(argv[++i]);
        } else if (!std::strcmp(argv[i], "--dust-from-particles"))
            dust_model = "particles";
        else if (!std::strcmp(argv[i], "--dust-voxel") && i + 1 < argc)
            dust_voxel = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--dust-blur") && i + 1 < argc)
            dust_blur = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--compaction-depth") && i + 1 < argc)
            compaction_depth = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--fine-roughness") && i + 1 < argc)
            fine_boost = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--camera") && i + 1 < argc)
            cameras = argv[++i];
        else if (!std::strcmp(argv[i], "--gi-rays") && i + 1 < argc)
            gi_cache_rays = static_cast<unsigned int>(std::atoi(argv[++i]));
        else if (!std::strcmp(argv[i], "--no-gi-cache"))
            gi_cache = false;
        else if (!std::strcmp(argv[i], "--noise-tolerance") && i + 1 < argc)
            noise_tolerance = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--profile"))
            profile = true;
        else if (!std::strcmp(argv[i], "--gpu-checks"))
            gpu_checks = true;
        else if (!std::strcmp(argv[i], "--no-gi"))
            gi = false;
    }

    const std::string out_dir = GetChronoOutputPath() + scenario.name + "/";
    const std::string save_dir = save ? out_dir : "";
    if (save && !CreateOutputDirectory(out_dir)) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Surface and site frame; the rendered terrain is a view of the surface with the ruts and berms on top
    auto surface = CreateSiteSurface({}, zoom, root_tile_deg);
    if (!surface)
        return 1;
    // The Moon preset's relief, with gentler roughness at the finest scales: the Hapke BRDF already carries the
    // regolith's roughness below a few centimeters (theta_p), and the preset's steep fine octaves, each casting its own
    // shadow under the low Sun, would count it twice
    {
        const ChPlanetBody body = moon::Body();
        auto roughness = moon::RoughnessParams();
        roughness.fine_boost = fine_boost;
        auto chain = chrono_types::make_shared<ChFilterChain>();
        chain->AddFilter(chrono_types::make_shared<ChBaseReliefLayer>(moon::BaseReliefParams()));
        chain->AddFilter(chrono_types::make_shared<ChCraterLayer>(body, moon::CraterParams()));
        chain->AddFilter(chrono_types::make_shared<ChRockLayer>(body, moon::RockParams()));
        chain->AddFilter(chrono_types::make_shared<ChRoughnessLayer>(body, roughness));
        surface->SetFilterChain(chain);
    }
    ChSiteFrame site = surface->MakeSiteFrame(site_lon, site_lat);
    auto ruts = chrono_types::make_shared<ChDeformationFilter>(site, rut_spacing);
    auto world = chrono_types::make_shared<ChPlanetQuadtree>(surface->CreateView(ruts), view_range_tiles);
    world->SetSplitDistance(split_distance);
    world->SetMaxZoom(max_zoom);
    auto height_at = [&](double x, double y) {
        double lon, lat;
        site.ToLonLat(x, y, lon, lat);
        return surface->GetElevation(lon, lat) - site.GetOriginElevation();
    };

    // Rover, resting on the surface along the start's smooth path
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    auto rock_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    rock_mat->SetFriction(0.8f);
    rock_mat->SetRestitution(0.01f);
    Viper viper(&sys, ViperWheelType::RealWheel);
    viper.SetDriver(chrono_types::make_shared<SettleThenDrive>());
    viper.SetWheelContactMaterial(rock_mat);
    const ChQuaternion<> start_rot = QuatFromAngleZ(start_heading * CH_DEG_TO_RAD);
    double ground = -1e9;
    for (double wx : {-0.64, 0.64})
        for (double wy : {-0.61, 0.61}) {
            const ChVector3d w = start_rot.Rotate(ChVector3d(wx, wy, 0));
            ground = std::max(ground, height_at(start_x + w.x(), start_y + w.y()));
        }
    viper.Initialize(ChFrame<>(ChVector3d(start_x, start_y, ground + wheel_radius + 0.005), start_rot));
    auto chassis = viper.GetChassis()->GetBody();

    // The wheels as the soil sees them: cylinders about the axle (y in a wheel's reference frame), half a spacing
    // smaller than the wheel, since soil particles keep about a spacing from the outer markers
    auto wheel_geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    wheel_geometry->materials.push_back(ChContactMaterialData());
    wheel_geometry->coll_cylinders.push_back(utils::ChBodyGeometry::CylinderShape(VNULL, ChVector3d(0, 1, 0), wheel_radius - 0.5 * spacing, wheel_width));

    // CRM soil in a window that follows the rover (demo_PLANET_Viper_CRM)
    auto setup = [&](PlanetCRMTerrain& crm) {
        crm.SetVerbose(false);
        crm.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
        crm.SetStepSizeCFD(cfd_step);
        ChFsiFluidSystemSPH::SoilProperties soil;
        soil.density = 1700;
        soil.Young_modulus = 1e6;
        soil.Poisson_ratio = 0.3;
        soil.mu_I0 = 0.04;
        soil.mu_fric_s = friction;
        soil.mu_fric_2 = friction;
        soil.average_diam = 0.005;
        soil.cohesion_coeff = cohesion;
        crm.SetCrmSPH(soil);
        ChFsiFluidSystemSPH::SPHParameters sph;
        sph.integration_scheme = IntegrationScheme::RK2;
        sph.initial_spacing = spacing;
        sph.d0_multiplier = 1.3;
        sph.free_surface_threshold = 2.4;
        sph.artificial_viscosity = 0.5;
        sph.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
        sph.boundary_method = BoundaryMethod::ADAMI;
        sph.use_variable_time_step = true;
        sph.num_proximity_search_steps = 10;
        crm.SetSPHParameters(sph);
        for (int i = 0; i < 4; ++i)
            crm.AddRigidBody(viper.GetWheels()[i]->GetBody(), wheel_geometry, true);
        crm.SetActiveDomain(ChVector3d(0.5, 0.5, 0.5));
        crm.GetFluidSystemSPH()->EnableGPUErrorCheck(gpu_checks);
    };
    auto crm_soil = chrono_types::make_unique<PlanetCRMWindow>(sys, surface, site, ruts, spacing, setup);
    crm_soil->SetWindow(window_length, window_width, window_depth, window_margin);
    for (int i = 0; i < 4; ++i)
        crm_soil->AddWheel(viper.GetWheels()[i]->GetBody(), wheel_radius, wheel_width);
    crm_soil->SetBerms(berms);
    crm_soil->Initialize(chassis);
    std::cout << "CRM window: " << crm_soil->GetTerrain().GetNumSPHParticles() << " particles" << std::endl;
    auto crm_window = [&]() -> PlanetCRMWindow* { return crm_soil.get(); };
    auto soil_height = [&](double x, double y) { return crm_soil->GetHeight(x, y); };

    // Terrain and rocks as the cameras see them. Compacting regolith breaks up its porous surface, so ruts look
    // darker than the ground around them.
    const ChColor regolith_albedo(0.7f, 0.7f, 0.7f);
    const float compacted_darkening = float(rut_darkening);
    // Ground counts as compacted where the wheels lowered it by more than compaction_depth: a few millimeters, as fast
    // wheels skim the soil and leave shallow ruts
    ChPlanetVisualMesh visual_terrain(&sys, world, site);
    visual_terrain.SetMaterial(CreateRegolithMaterial(hapke, regolith_albedo));
    // A wheel presses the regolith smooth: compacted ground has less of the roughness below the mesh that the Hapke
    // BRDF models (macroscopic roughness 12 deg rather than 23.4), as its relief above it is pressed flat (the ruts'
    // heights, ChDeformationFilter::SetFlattenDepth)
    // Darker the deeper it is pressed, by steps from compaction_depth down to 2.5 cm, so ruts shade in from their edges
    // rather than switching color at an outline
    std::vector<ChPlanetVisualMesh::CompactionLevel> levels;
    const double level_depths[] = {compaction_depth, 0.008, 0.015, 0.025};
    for (int l = 0; l < 4; ++l) {
        const float share = float(l + 1) / 4;  // of the full darkening
        const float albedo_scale = 1 - share * (1 - compacted_darkening);
        const double roughness = 23.4 - share * (23.4 - 12.0);
        levels.push_back({std::max(level_depths[l], compaction_depth),
                          CreateRegolithMaterial(hapke, ChColor(regolith_albedo.R * albedo_scale, regolith_albedo.G * albedo_scale, regolith_albedo.B * albedo_scale), roughness)});
    }
    visual_terrain.SetCompaction(ruts, levels);
    // and keeps some of the regolith's texture on its floor
    ruts->SetFlattenStrength(flatten_strength);
    visual_terrain.SetMaxDistance(terrain_range);
    visual_terrain.SetMorphTime(morph_time);
    visual_terrain.Update(chassis->GetPos(), 0.0);

    PlanetBoulderField boulders(&sys, surface, site, rock_mat, rock_min_radius);
    boulders.SetVisualMaterial(CreateRegolithMaterial(hapke, ChColor(0.55f, 0.53f, 0.5f)));
    if (use_rocks)
        boulders.Update(chassis->GetPos(), rock_spawn_radius);

    // Soil thrown up, flying ballistically over the undeformed ground
    std::unique_ptr<ChDustField> dust;
    if (use_dust) {
        ChDustField::Params dust_params;
        dust_params.gravity = gravity;
        // Lunar fines cling to the coarser grains, so less of the thrown mass flies as fine dust than the soil's
        // size distribution alone would give: a thinner, grainier spray
        dust_params.bins = {{5e-6, 0.05}, {25e-6, 0.35}, {100e-6, 0.60}};
        // Over the ground with the ruts, so a wheel riding in its rut is in contact with the soil
        dust = chrono_types::make_unique<ChDustField>(dust_params, [&soil_height](double x, double y) { return soil_height(x, y); });
        // Soil carried on the tread and shed as the wheel turns: most right behind the contact, some riding up and over
        // the top of the wheel
        ChDustField::WheelEmission emission;
        if (dust_model == "physics") {
            // Soil carried in the tread's grooves (grousers about 1 cm deep), held by the soil's cohesion and released
            // where the pull out of the groove exceeds it
            emission.bulk_density = 1700;
            emission.cohesion = cohesion;
            emission.grouser_height = 0.01;
        } else {
            // Soil shed progressively off the tread: most right behind the contact, some riding up and over the top
            emission.max_release_angle = 150 * CH_DEG_TO_RAD;
            emission.release_decay = 0.6;
            if (dust_model == "hybrid") {
                // as much as the wheel's state in the CRM soil gives: the tread bites as deep as the wheel sinks, up to
                // its grousers (about 1 cm), and driving slip loosens more
                emission.bulk_density = 1700;
                emission.grouser_height = 0.01;
                emission.scale_with_sinkage = true;
                emission.loose_depth = 4e-4;
            }
        }
        dust->SetWheelEmission(emission);
        // Fine voxels around the rover, so the spray keeps its shape
        ChDustField::GridSpec grid;
        grid.voxel = dust_voxel;
        grid.nx = grid.ny = static_cast<int>(std::round(4.0 / dust_voxel));
        // Deep enough below the ground under the rover for the dust of wheels lower down a slope
        grid.nz = static_cast<int>(std::round(2.5 / dust_voxel));
        grid.below = 1.0;
        grid.blur = dust_blur;
        // Streaked over a frame's exposure, as the cameras would see the grains
        grid.exposure = 1.0 / std::max(rate, 30.0f);
        grid.exposure_samples = 6;
        dust->SetGridSpec(grid);
    }
    // The coarse grains in flight, drawn as small clods on a body of their own
    auto grains = chrono_types::make_shared<ChBody>();
    grains->SetFixed(true);
    grains->EnableCollision(false);
    sys.AddBody(grains);
    auto grain_material = CreateRegolithMaterial(hapke, ChColor(0.6f, 0.6f, 0.6f));
    auto update_grains = [&](double t) {
        if (!dust)
            return;
        grains->GetVisualModel() ? grains->GetVisualModel()->Clear() : void();
        auto shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        shape->SetMesh(DustGrainMesh(*dust, t, static_cast<int>(dust->GetParams().bins.size()) - 1, 15000));
        shape->SetMutable(false);
        shape->AddMaterial(grain_material);
        if (shape->GetMesh()->GetNumTriangles() > 0)
            grains->AddVisualShape(shape);
    };

    const double e = sun_elevation * CH_DEG_TO_RAD;
    const ChVector3d sun_dir(std::cos(e) * std::cos(SunAzimuth()), std::cos(e) * std::sin(SunAzimuth()), std::sin(e));

    // Sensors: sunlight only, with no ambient term, under a black sky; a mast camera looking ahead and down, and a
    // chase camera behind and left of the rover, looking at it
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddDirectionalLight(ChColor(sun_irradiance, sun_irradiance, sun_irradiance), float(e), SunAzimuth());
    manager->scene->SetAmbientLight({0.f, 0.f, 0.f});
    Background background;
    background.mode = BackgroundMode::SOLID_COLOR;
    background.color_zenith = ChVector3f(0.f, 0.f, 0.f);
    background.color_horizon = ChVector3f(0.f, 0.f, 0.f);
    manager->scene->SetBackground(background);
    if (cameras == "mast" || cameras == "both")
        AddCamera(*manager, chassis, rate, ChFrame<>(ChVector3d(0.95, 0, 1.75), QuatFromAngleY(20 * CH_DEG_TO_RAD)), "mast", window, save_dir, samples, gi);
    // The chase camera rides a level rig that follows the rover's position and heading smoothly, but not its roll or
    // pitch, so the rover is seen tilting over rough ground against a level horizon
    auto rig = chrono_types::make_shared<ChBody>();
    rig->SetFixed(true);
    rig->EnableCollision(false);
    sys.AddBody(rig);
    auto chassis_heading = [&]() {
        ChVector3d x = chassis->GetVisualModelFrame().GetRotMat().GetAxisX();
        x.z() = 0;
        return x.Length() > 1e-6 ? std::atan2(x.y(), x.x()) : 0.0;
    };
    ChVector3d rig_pos = chassis->GetVisualModelFrame().GetPos();
    double rig_heading = chassis_heading();
    auto move_rig = [&](double dt) {
        const double follow = 1 - std::exp(-dt / 0.3), turn = 1 - std::exp(-dt / 1.0);
        rig_pos += follow * (chassis->GetVisualModelFrame().GetPos() - rig_pos);
        rig_heading += turn * std::remainder(chassis_heading() - rig_heading, CH_2PI);
        rig->SetPos(rig_pos);
        rig->SetRot(QuatFromAngleZ(rig_heading));
    };
    move_rig(0);
    // A side camera on the rig, off the rover's left, the side the Sun lights, looking across at it
    if (cameras == "side")
        AddCamera(*manager, rig, rate, ChFrame<>(ChVector3d(0.3, 4.5, 1.0), QuatFromAngleZ(-CH_PI_2) * QuatFromAngleY(8 * CH_DEG_TO_RAD)), "side",
                  window, save_dir, samples, gi);
    if (cameras == "chase" || cameras == "both")
        AddCamera(*manager, rig, rate,
              ChFrame<>(ChVector3d(-4.0, -2.2, 1.8), QuatFromAngleZ(std::atan2(2.2, 4.0)) * QuatFromAngleY(18 * CH_DEG_TO_RAD)), "chase",
              window, save_dir, samples, gi);

    // Simulation loop: the ruts, berms and terrain detail are refreshed once per camera update
    const double frame_period = 1.0 / rate;
    double next_frame = 0, next_dust = 0, next_rocks = 0, next_report = 1.0;
    // Soil thrown up at each wheel, as the CRM soil gives it (--dust-model physics): rate (kg/s), where, how fast
    struct ThrownSoil {
        double rate = 0;
        ChVector3d pos, vel;
        double spread = 0;
        bool seen = false;
    };
    std::vector<ThrownSoil> thrown_soil(4);
    size_t thrown = 0;
    const auto wall_start = std::chrono::steady_clock::now();
    // Wall time spent per report period in each part of the loop (--profile)
    enum Part { kRuts, kTerrain, kDust, kEmission, kSensors, kSoil, kParts };
    const char* part_names[kParts] = {"ruts", "terrain", "dust", "emission", "sensors", "soil"};
    double part_time[kParts] = {};
    auto lap = std::chrono::steady_clock::now();
    auto clock = [&](Part part) {
        if (!profile)
            return;
        const auto now = std::chrono::steady_clock::now();
        part_time[part] += std::chrono::duration<double>(now - lap).count();
        lap = now;
    };
    while (sys.GetChTime() < end_time) {
        lap = std::chrono::steady_clock::now();
        const double time = sys.GetChTime();
        const ChVector3d pos = chassis->GetPos();
        if (time >= next_frame) {
            crm_soil->Publish();
            clock(kRuts);
            visual_terrain.Update(pos, time);
            clock(kTerrain);
            if (dust) {
                dust->Update(time);
                dust->UpdateGrid(time, ChVector3d(pos.x(), pos.y(), height_at(pos.x(), pos.y())), sun_dir);
                SetDustVolume(*manager, *dust, hapke_w, hapke_b, hapke_c, regolith_albedo);
                update_grains(time);
            }
            clock(kDust);
            next_frame += frame_period;
        }
        if (dust && dust_model == "particles" && time >= next_dust && crm_window()) {
            thrown += crm_window()->EmitDust(*dust, time);
            next_dust += dust_period;
        }
        if (dust && dust_model == "physics" && crm_window()) {
            // Soil the CRM soil throws into free flight at each wheel, measured every dust_period and emitted smoothly
            // at its rate, from where and at the velocities the soil had
            if (time >= next_dust) {
                const auto ejecta = crm_window()->MeasureEjecta(time);
                const double follow = 1 - std::exp(-dust_period / 0.2);
                for (size_t w = 0; w < ejecta.size(); ++w) {
                    ThrownSoil& t = thrown_soil[w];
                    t.rate += follow * (ejecta[w].mass / dust_period - t.rate);
                    if (ejecta[w].mass > 0) {
                        thrown += 1;
                        if (!t.seen) {
                            t.pos = ejecta[w].pos, t.vel = ejecta[w].vel, t.spread = ejecta[w].vel_spread, t.seen = true;
                        } else {
                            t.pos += follow * (ejecta[w].pos - t.pos);
                            t.vel += follow * (ejecta[w].vel - t.vel);
                            t.spread += follow * (ejecta[w].vel_spread - t.spread);
                        }
                    }
                }
                next_dust += dust_period;
            }
            for (const auto& t : thrown_soil)
                if (t.seen && t.rate > 0)
                    dust->EmitSource(t.pos, 0.5 * spacing, t.vel, std::max(t.spread, 0.1 * t.vel.Length()), t.rate * exchange_step, time,
                                     exchange_step);
        }
        if (dust && dust_model != "particles") {
            // Soil loosened at each wheel's contact and thrown off its rim, at the rate its speed and slip over the
            // CRM soil give
            std::vector<ChDustField::WheelState> wheel_states;
            for (int i = 0; i < 4; ++i)
                wheel_states.push_back(DustWheelState(viper.GetWheels()[i]->GetBody(), wheel_radius, wheel_width));
            dust->EmitFromWheels(wheel_states, time, exchange_step);
        }
        if (use_rocks && time >= next_rocks) {
            boulders.Update(pos, rock_spawn_radius);
            next_rocks += rock_update_period;
        }
        move_rig(exchange_step);
        clock(kEmission);
        manager->Update();
        clock(kSensors);

        viper.Update();
        crm_soil->Advance(exchange_step);
        clock(kSoil);

        if (sys.GetChTime() >= next_report) {
            next_report += 1.0;
            const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
            std::cout << "t = " << sys.GetChTime() << " s (" << sys.GetChTime() / wall << "x real time), at (" << pos.x() << ", "
                      << pos.y() << "), speed " << chassis->GetPosDt().Length() << " m/s, window moves " << (crm_window() ? crm_window()->GetNumMoves() : 0) << ", "
                      << ruts->GetNumNodes() << " rut nodes, " << thrown << " particles thrown up, " << visual_terrain.GetNumTiles()
                      << " tiles (" << visual_terrain.GetNumTriangles() << " triangles)";
            // slip: how much of the wheels' rim speed does not carry the rover forward
            double rim = 0;
            for (int i = 0; i < 4; ++i) {
                const auto w = viper.GetWheels()[i]->GetBody();
                rim += 0.25 * std::abs(w->GetAngVelParent().Dot(w->GetFrameRefToAbs().GetRotMat().GetAxisY())) * wheel_radius;
            }
            ChVector3d fwd = chassis->GetVisualModelFrame().GetRotMat().GetAxisX();
            const double advance = chassis->GetPosDt().Dot(fwd);
            if (rim > 1e-3)
                std::cout << ", slip " << 1 - advance / rim;
            if (dust)
                std::cout << ", dust " << dust->GetStats().aloft * 1e3 << " g aloft";
            std::cout << std::endl;
            if (profile) {
                std::cout << "  wall s:";
                for (int k = 0; k < kParts; ++k) {
                    std::cout << " " << part_names[k] << " " << part_time[k];
                    part_time[k] = 0;
                }
                std::cout << std::endl;
            }
        }
    }
    if (save)
        std::cout << "Camera images in " << out_dir << std::endl;
    return 0;
}

#endif
