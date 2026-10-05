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
// From the whole Moon down to VIPER, seen through a Chrono::Sensor camera. The
// camera starts thousands of kilometers above a lunar landing site, with the
// full disk of the Moon in view, and descends straight down onto the site at a
// steady rate in log altitude, then swings behind the VIPER rover parked there
// and follows it as it drives off. One quadtree carries the terrain the whole
// way, from the coarse global DEM to the landing-site DEM and the procedural
// relief below it: ChPlanetVisualMesh keeps its tiles around the camera as
// visual shapes the camera renders with the Hapke BRDF. The rover rides a rigid
// PlanetTerrain collision patch among procedural boulders.
//
// Command line: GeoTIFF paths form the DEM stack; with none, the Moon DEM
// resources shipped with Chrono are used, plus the 64 px/deg global DEM if it
// is installed (see data/planet/moon/README.md), which gives the descent far
// more relief above the landing-site DEM. Flags: --lambert renders with a
// Lambertian material instead of Hapke, --save writes the camera images to the
// demo output directory, --no-window skips the camera window, --no-rocks skips
// the boulder field.
//
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/collision/ChCollisionShapeCylinder.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolver.h"

#include "chrono_models/robot/viper/Viper.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChPlanetVisualMesh.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"
#include "chrono_planet/planets/moon/ChMoon.h"

#include "chrono_vehicle/terrain/PlanetTerrain.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "PlanetBoulderField.h"
#include "PlanetDemoSetup.h"

using namespace chrono;
using namespace chrono::planet;
using namespace chrono::sensor;
using namespace chrono::vehicle;
using namespace chrono::viper;

// Landing site (Apollo 17 region) and the quadtree zoom the physics surface is sampled at.
const double site_lon = 30.75;
const double site_lat = 20.19;
const int zoom = 15;

// Quadtree root tile size (deg) and ring half-width (tiles). From the start altitude the camera sees a cap of
// the Moon about 75 degrees in radius, so the ring must reach that far.
const double root_tile_deg = 16.0;
const int view_range_tiles = 6;

// The quadtree splits a tile once the camera is within this many tile widths of it (ChPlanetQuadtree::
// SetSplitDistance). At the default of 1, suited to a rover's view, a tile's cells are about 1/64 radian apart
// as seen from the camera, some 19 pixels in this camera. For the fly-in the split distance is fly_split,
// about 3 pixel cells, tapering back to 1 as the camera comes down from taper_start_height to taper_end_height.
// Coarse tiles have fewer cells, so from orbit it also grows with the camera's height (height /
// split_scale_height, up to max_split_distance): the default would leave the disk a polyhedron of 8 degree
// facets.
const double fly_split = 6.0;
const double taper_start_height = 2000.0;  // m
const double taper_end_height = 20.0;      // m
const double split_scale_height = 5.0e4;   // m
const double max_split_distance = 25.0;
// However far the camera, tiles in view are split to at least this zoom: 1/8 degree cells.
const int min_zoom = 3;

// Tiles are drawn out to this many camera heights, and at least draw_min_distance: far enough to cover the
// view looking down and the shadows the terrain around it casts, near enough that the renderer does not hold
// the finely split terrain all the way to the horizon.
const double draw_heights = 4.0;
const double draw_min_distance = 5000.0;  // m

// Physics
const double gravity = moon::kGravity;
const double step_size = 1e-3;

// Rigid patch that follows the rover
const double patch_size = 40.0;
const double patch_resolution = 0.25;
const double rebuild_margin = 8.0;

// Boulders: every rock at least this big within this distance of the rover
const double rock_min_radius = 0.12;
const double rock_spawn_radius = 35.0;

// VIPER, drawn with its real wheels and colliding through cylinders, as in demo_PLANET_Viper_Rigid
const double wheel_radius = 0.25;
const double wheel_width = 0.29;
const double wheel_speed = 0.6 * CH_PI;  // wheel angular speed once driving (rad/s)
const double drive_ramp = 3.0;           // time to reach it (s)
const double drive_look = 12.0;          // ground ahead the rover's heading is chosen over (m), past where it gets to

// Flight plan. The camera holds on the full Moon, descends straight down onto the rover, easing out of the
// vertical into a chase view over the last few hundred meters, and then follows the rover.
const double hold_time = 3.0;              // on the full Moon (s)
const double descent_time = 40.0;          // from start_height to the chase view (s)
const double follow_time = 12.0;           // behind the rover once there (s)
const double start_height = 5.0e6;         // camera height above the site at the start (m)
const double swing_height = 300.0;         // height at which the camera starts to leave the vertical (m)
const ChVector3d chase_offset(-7, -4, 3);  // camera offset from the rover in the chase view (m, rover heading frame)
const double drive_lead = 8.0;             // the rover starts driving this long before the camera arrives (s)

// Sensor
const float frame_rate = 30.0f;
const unsigned int image_width = 1280;
const unsigned int image_height = 720;
const float fov = float(60 * CH_DEG_TO_RAD);  // horizontal

// The Sun. Seen from high over the site, the Moon is full only with the Sun near the site's zenith, but under a
// high Sun the ground shows little relief. So, as in a time lapse, the Sun sets from sun_start_elevation to
// sun_end_elevation as the camera comes down to sun_settle_height (interpolated in log height), and stays there.
// Give both elevations the same value for a fixed Sun.
const double sun_start_elevation = 65 * CH_DEG_TO_RAD;
const double sun_end_elevation = 20 * CH_DEG_TO_RAD;
const double sun_settle_height = 1.0e6;  // m
const float sun_azimuth = float(200 * CH_DEG_TO_RAD);
const float sun_irradiance = 2.5f;

// -----------------------------------------------------------------------------

// Wheel speeds held at zero until a start time, then ramped up.
class DelayedSpeedDriver : public ViperDriver {
  public:
    DelayedSpeedDriver(double start, double ramp, double speed) : m_start(start), m_ramp(ramp), m_speed(speed) {}

  private:
    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::SPEED; }
    virtual void Update(double time) override {
        const double speed = m_speed * std::clamp((time - m_start) / m_ramp, 0.0, 1.0);
        drive_speeds = {speed, speed, speed, speed};
    }

    double m_start, m_ramp, m_speed;
};

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
        mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, float(23.4 * CH_DEG_TO_RAD));
    }
    return mat;
}

double Ease(double s) {
    s = std::clamp(s, 0.0, 1.0);
    return s * s * (3.0 - 2.0 * s);
}

// Split distance for a camera at the given height above the site
double SplitDistance(double height) {
    const double s = std::log(std::max(height, 1.0) / taper_end_height) / std::log(taper_start_height / taper_end_height);
    const double fly = 1 + Ease(s) * (fly_split - 1);
    return std::clamp(height / split_scale_height, fly, max_split_distance);
}

// Sun elevation for a camera at the given height above the site
float SunElevation(double height) {
    const double s = std::log(start_height / std::max(height, 1.0)) / std::log(start_height / sun_settle_height);
    return float(sun_start_elevation + Ease(s) * (sun_end_elevation - sun_start_elevation));
}

// Camera pose at time t, for a rover at `target` heading along `heading` (unit, horizontal). The camera looks
// along its x axis with z up.
ChFrame<> CameraAt(double t, const ChVector3d& target, const ChVector3d& heading) {
    const ChVector3d left(-heading.y(), heading.x(), 0);
    const ChVector3d chase = chase_offset.x() * heading + chase_offset.y() * left + ChVector3d(0, 0, chase_offset.z());

    // Height above the rover, interpolated in log space: the ground grows in view at a steady rate.
    const double s = Ease((t - hold_time) / descent_time);
    const double height = start_height * std::pow(chase.z() / start_height, s);

    // Out of the vertical below swing_height, into the chase view
    const double w = height >= swing_height ? 0.0 : Ease(std::log(swing_height / height) / std::log(swing_height / chase.z()));
    const ChVector3d pos = target + (1 - w) * ChVector3d(0, 0, height) + w * chase;

    // Looking down, image up is north; in the chase view, it is up.
    const ChVector3d forward = (target - pos).GetNormalized();
    const ChVector3d up_hint = (1 - w) * ChVector3d(0, 1, 0) + w * ChVector3d(0, 0, 1);
    const ChVector3d y = Vcross(up_hint, forward).GetNormalized();
    const ChVector3d z = Vcross(forward, y);
    return ChFrame<>(pos, ChMatrix33<>(forward, y, z).GetQuaternion());
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    bool hapke = true;
    bool save = false;
    bool window = true;
    bool use_rocks = true;
    std::vector<ChGeoTiffSource> dems;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--lambert"))
            hapke = false;
        else if (!std::strcmp(argv[i], "--save"))
            save = true;
        else if (!std::strcmp(argv[i], "--no-window"))
            window = false;
        else if (!std::strcmp(argv[i], "--no-rocks"))
            use_rocks = false;
        else
            dems.push_back({argv[i], 0, 30});
    }
    const double end_time = hold_time + descent_time + follow_time;
    std::cout << "Material: " << (hapke ? "Hapke" : "Lambertian") << ", " << end_time << " s" << std::endl;

    const std::string out_dir = GetChronoOutputPath() + "PLANET_MoonDescent/";
    const std::string run_dir = out_dir + (hapke ? "hapke/" : "lambert/");
    if (save && (!CreateOutputDirectory(out_dir) || !CreateOutputDirectory(run_dir))) {
        std::cerr << "Error creating directory " << run_dir << std::endl;
        return 1;
    }

    // Surface and site frame, one surface for physics and rendering so the wheels ride the rendered ground
    // The best Moon DEMs installed: the shipped ones alone give the descent only 7.6 km samples down to the
    // landing-site grid, which is striped by LOLA's ground tracks.
    auto surface = CreateSiteSurface(dems, zoom, root_tile_deg);
    if (!surface)
        return 1;
    ChSiteFrame site = surface->MakeSiteFrame(site_lon, site_lat);
    std::cout << "Site origin elevation: " << site.GetOriginElevation() << " m" << std::endl;

    // Multibody system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetNumThreads(1, 1, 4);

    auto ground_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    ground_mat->SetFriction(0.9f);
    ground_mat->SetRestitution(0.01f);

    // Rigid terrain patch
    PlanetTerrain terrain(&sys, surface, site);
    terrain.SetPatchSize(patch_size);
    terrain.SetPatchResolution(patch_resolution);
    terrain.SetRebuildMargin(rebuild_margin);
    terrain.SetContactMaterial(ground_mat);
    terrain.Initialize();
    // The ground body gets the family the boulders ignore; a rebuild creates a new body, so this is repeated then.
    terrain.GetGroundBody()->GetCollisionModel()->SetFamily(PlanetBoulderField::kTerrainFamily);

    // Rover, resting just above the highest ground under its wheels and pointed along the flattest strip of
    // ground ahead of it, since it drives straight: the steepest rise or drop between points half a meter apart
    // along the strip and across its width is least that way (preferring east, among equals). It sets off
    // shortly before the camera reaches it.
    const double drive_start = hold_time + descent_time - drive_lead;
    auto driver = chrono_types::make_shared<DelayedSpeedDriver>(drive_start, drive_ramp, wheel_speed);
    Viper viper(&sys, ViperWheelType::RealWheel);
    viper.SetDriver(driver);
    viper.SetWheelContactMaterial(ground_mat);

    double start_heading = 0, flattest = 1e9;
    for (int k = 0; k < 36; ++k) {
        const double heading = k * CH_2PI / 36;
        const ChVector3d ahead(std::cos(heading), std::sin(heading), 0), left(-ahead.y(), ahead.x(), 0);
        double steepest = 0;
        for (double side : {-0.6, 0.0, 0.6}) {
            double last = terrain.GetHeight(side * left);
            for (double d = 0.5; d <= drive_look; d += 0.5) {
                const double h = terrain.GetHeight(d * ahead + side * left);
                steepest = std::max(steepest, std::abs(h - last) / 0.5);
                last = h;
            }
        }
        if (steepest < flattest - 1e-3) {
            flattest = steepest;
            start_heading = heading;
        }
    }
    std::cout << "Rover heading " << start_heading * CH_RAD_TO_DEG << " deg from east, steepest slope ahead " << std::atan(flattest) * CH_RAD_TO_DEG << " deg" << std::endl;
    const ChQuaternion<> start_rot = QuatFromAngleZ(start_heading);
    double ground = -1e9;
    for (double wx : {-0.64, 0.64})
        for (double wy : {-0.61, 0.61})
            ground = std::max(ground, terrain.GetHeight(start_rot.Rotate(ChVector3d(wx, wy, 0))));
    viper.Initialize(ChFrame<>(ChVector3d(0, 0, ground + wheel_radius + 0.1), start_rot));
    auto chassis = viper.GetChassis()->GetBody();

    // The wheel spins about its local y axis, so the cylinder's axis (z by default) is rotated onto y.
    for (auto id : {V_LF, V_RF, V_LB, V_RB}) {
        auto wheel = viper.GetWheel(id)->GetBody();
        wheel->GetCollisionModel()->Clear();
        auto cyl = chrono_types::make_shared<ChCollisionShapeCylinder>(ground_mat, wheel_radius, wheel_width);
        wheel->AddCollisionShape(cyl, ChFrame<>(VNULL, Q_ROTATE_Z_TO_Y));
    }

    // Terrain and rocks as the camera sees them. The quadtree follows the camera, from orbit to the ground, and
    // every tile it holds is drawn, at its true place on the sphere: the site frame is flat, which the rover
    // cannot tell apart from the sphere over the few tens of meters it drives, but the camera can from orbit.
    auto world = chrono_types::make_shared<ChPlanetQuadtree>(surface, view_range_tiles);
    world->SetMinZoom(min_zoom);
    ChPlanetVisualMesh visual_terrain(&sys, world, site);
    visual_terrain.SetShape(ChPlanetVisualMesh::Shape::SPHERE);
    visual_terrain.SetMaterial(CreateRegolithMaterial(hapke, ChColor(0.7f, 0.7f, 0.7f)));

    PlanetBoulderField boulders(&sys, surface, site, ground_mat, rock_min_radius);
    boulders.SetVisualMaterial(CreateRegolithMaterial(hapke, ChColor(0.55f, 0.53f, 0.5f)));
    if (use_rocks)
        boulders.Update(chassis->GetPos(), rock_spawn_radius);
    std::cout << "Boulders around the site: " << boulders.GetNumRocks() << std::endl;

    // A kinematic body carrying the camera, moved along the flight plan each step
    auto rover_target = [&]() { return chassis->GetPos() + ChVector3d(0, 0, 0.5); };
    auto rover_heading = [&]() {
        // The chassis reference frame, as the rover's own sensors use it
        const ChVector3d x = chassis->GetVisualModelFrame().GetRotMat().GetAxisX();
        return ChVector3d(x.x(), x.y(), 0).GetNormalized();
    };
    auto camera_body = chrono_types::make_shared<ChBody>();
    camera_body->SetFixed(true);
    camera_body->EnableCollision(false);
    camera_body->SetCoordsys(CameraAt(0, rover_target(), rover_heading()).GetCoordsys());
    sys.AddBody(camera_body);

    // Sunlight only, with no ambient term, under a black sky
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    const ChColor sun_color(sun_irradiance, sun_irradiance, sun_irradiance);
    float sun_elevation = SunElevation(start_height);
    const unsigned int sun = manager->scene->AddDirectionalLight(sun_color, sun_elevation, sun_azimuth);
    manager->scene->SetAmbientLight({0.f, 0.f, 0.f});
    Background background;
    background.mode = BackgroundMode::SOLID_COLOR;
    background.color_zenith = ChVector3f(0.f, 0.f, 0.f);
    background.color_horizon = ChVector3f(0.f, 0.f, 0.f);
    manager->scene->SetBackground(background);

    auto cam = chrono_types::make_shared<ChCameraSensor>(camera_body, frame_rate, ChFrame<>(), image_width, image_height, fov);
    cam->SetName("descent");
    cam->SetLag(0.f);
    cam->SetCollectionWindow(0.f);
    if (window)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width / 2, image_height / 2, "Moon descent"));
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    if (save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(run_dir + "descent/"));
    manager->AddSensor(cam);

    // Simulation loop. The terrain level of detail is refreshed once per camera frame.
    const int frame_steps = static_cast<int>(std::round(1.0 / (frame_rate * step_size)));
    const auto wall_start = std::chrono::steady_clock::now();
    int step = 0;
    while (sys.GetChTime() < end_time) {
        const double time = sys.GetChTime();
        const ChVector3d pos = chassis->GetPos();

        camera_body->SetCoordsys(CameraAt(time, rover_target(), rover_heading()).GetCoordsys());
        if (step % frame_steps == 0) {
            const double height = camera_body->GetPos().z();
            world->SetSplitDistance(SplitDistance(height));
            // Changing a light restages the scene, so only while the Sun moves
            const float elevation = SunElevation(height);
            if (elevation != sun_elevation) {
                manager->scene->ModifyDirectionalLight(sun, sun_color, elevation, sun_azimuth);
                sun_elevation = elevation;
            }
            visual_terrain.SetMaxDistance(std::max(draw_heights * height, draw_min_distance));
            visual_terrain.Update(camera_body->GetPos(), time);
        }
        manager->Update();

        driver->SetSteering(0);
        sys.DoStepDynamics(step_size);
        viper.Update();

        // Keep the collision patch and the boulder field around the rover
        if (terrain.UpdatePatch(pos))
            terrain.GetGroundBody()->GetCollisionModel()->SetFamily(PlanetBoulderField::kTerrainFamily);
        if (use_rocks && step % 200 == 0)
            boulders.Update(pos, rock_spawn_radius);

        if (++step % 1000 == 0) {
            const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
            const double height = (camera_body->GetPos() - pos).Length();
            std::cout << "t = " << time << " s (" << time / wall << "x real time), camera " << height << " m from the rover, rover at (" << pos.x() << ", " << pos.y() << ") m, "
                      << visual_terrain.GetNumTiles() << " tiles (" << visual_terrain.GetNumTriangles() << " triangles), " << visual_terrain.GetNumRebuilds() << " terrain rebuilds"
                      << std::endl;
        }
    }

    if (save)
        std::cout << "Images saved to " << run_dir << "descent/" << std::endl;
    return 0;
}
