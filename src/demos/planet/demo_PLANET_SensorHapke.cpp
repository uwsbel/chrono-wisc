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
// Chrono::Sensor cameras on a body driven across the Moon's quadtree terrain,
// shaded with the Hapke BRDF under a low Sun. ChPlanetVisualMesh keeps the
// terrain tiles around the body as visual shapes that the sensors render,
// replacing them as the level of detail changes. Pass "lambert" on the command
// line to render the same run with a Lambertian material instead. Images are
// saved to the demo output directory.
//
// =============================================================================

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChPlanetVisualMesh.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "PlanetDemoSetup.h"

using namespace chrono;
using namespace chrono::planet;
using namespace chrono::sensor;

// Landing site (Apollo 17 region) and the zoom the surface is sampled at.
const double site_lon = 30.75;
const double site_lat = 20.19;
const int zoom = 15;

// Quadtree root tile size (deg), ring half-width (tiles), and the range of terrain given to the sensors (m).
const double root_tile_deg = 16.0;
const int view_range_tiles = 4;
const double terrain_range = 3000.0;

// Sun elevation and azimuth (rad). A low Sun, as at the Apollo 17 landing.
const float sun_elevation = float(12 * CH_DEG_TO_RAD);
const float sun_azimuth = float(200 * CH_DEG_TO_RAD);

// The body's path: straight east at a fixed speed, at a fixed height above the ground.
const double speed = 10.0;
const double body_height = 1.5;
const double end_time = 5.0;
const double step_size = 1e-2;
const float frame_rate = 10.0f;

std::shared_ptr<ChVisualMaterial> CreateRegolithMaterial(bool hapke) {
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetAmbientColor({0.0f, 0.0f, 0.0f});
    mat->SetDiffuseColor({0.7f, 0.7f, 0.7f});
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

void AddCamera(ChSensorManager& manager,
               std::shared_ptr<ChBody> mount,
               const ChFrame<double>& pose,
               const std::string& name,
               const std::string& out_dir) {
    auto cam = chrono_types::make_shared<ChCameraSensor>(mount, frame_rate, pose, 1280, 720, float(CH_PI / 3));
    cam->SetName(name);
    cam->SetLag(0.f);
    cam->SetCollectionWindow(0.f);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + name + "/"));
    manager.AddSensor(cam);
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    const bool hapke = !(argc > 1 && std::strcmp(argv[1], "lambert") == 0);
    std::cout << "Material: " << (hapke ? "Hapke" : "Lambertian") << std::endl;

    const std::string demo_dir = GetChronoOutputPath() + "PLANET_SensorHapke/";
    const std::string out_dir = demo_dir + (hapke ? "hapke/" : "lambert/");
    if (!CreateOutputDirectory(demo_dir) || !CreateOutputDirectory(out_dir)) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    auto surface = CreateDemoSurface({}, zoom, root_tile_deg);
    if (!surface)
        return 1;
    const ChSiteFrame site = surface->MakeSiteFrame(site_lon, site_lat);
    auto ground_height = [&](double x, double y) {
        double lon, lat;
        site.ToLonLat(x, y, lon, lat);
        return surface->GetElevation(lon, lat) - site.GetOriginElevation();
    };

    ChSystemNSC sys;

    // The terrain the sensors see, with its own quadtree driven from the body's position.
    auto world = chrono_types::make_shared<ChPlanetQuadtree>(surface, view_range_tiles);
    ChPlanetVisualMesh terrain(&sys, world, site);
    terrain.SetMaterial(CreateRegolithMaterial(hapke));
    terrain.SetMaxDistance(terrain_range);

    // A kinematic body carrying the cameras, moved along the path each step.
    auto body_pos = [&](double t) {
        const double x = -0.5 * speed * end_time + speed * t;
        return ChVector3d(x, 0, ground_height(x, 0) + body_height);
    };
    auto body = chrono_types::make_shared<ChBody>();
    body->SetFixed(true);
    body->SetPos(body_pos(0));
    sys.AddBody(body);

    // Sunlight only: no ambient term and a black sky.
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddDirectionalLight({1.0f, 1.0f, 1.0f}, sun_elevation, sun_azimuth);
    manager->scene->SetAmbientLight({0.f, 0.f, 0.f});
    Background b;
    b.mode = BackgroundMode::SOLID_COLOR;
    b.color_zenith = ChVector3f(0.f, 0.f, 0.f);
    b.color_horizon = ChVector3f(0.f, 0.f, 0.f);
    manager->scene->SetBackground(b);

    // A forward view from the body and a chase view from behind and above it.
    AddCamera(*manager, body, ChFrame<>(ChVector3d(0, 0, 0), QuatFromAngleY(10 * CH_DEG_TO_RAD)), "rover", out_dir);
    AddCamera(*manager, body,
              ChFrame<>(ChVector3d(-40, -20, 25), QuatFromAngleZ(25 * CH_DEG_TO_RAD) * QuatFromAngleY(30 * CH_DEG_TO_RAD)), "chase",
              out_dir);

    using clock = std::chrono::steady_clock;
    double terrain_s = 0, sensor_s = 0, sensor_after_rebuild_s = 0;
    int rebuild_steps = 0;
    const int num_steps = static_cast<int>(std::ceil(end_time / step_size));
    for (int i = 0; i < num_steps; ++i) {
        const double t = sys.GetChTime();
        body->SetPos(body_pos(t));

        const auto t0 = clock::now();
        const bool rebuilt = terrain.Update(body->GetPos(), t);
        const auto t1 = clock::now();
        manager->Update();
        const auto t2 = clock::now();

        const double terrain_step_s = std::chrono::duration<double>(t1 - t0).count();
        const double sensor_step_s = std::chrono::duration<double>(t2 - t1).count();
        terrain_s += terrain_step_s;
        sensor_s += sensor_step_s;
        if (rebuilt) {
            sensor_after_rebuild_s += sensor_step_s;
            ++rebuild_steps;
            std::cout << "t = " << t << " s: " << terrain.GetNumTiles() << " tiles, " << terrain.GetNumTriangles()
                      << " triangles; LOD and mesh " << terrain_step_s << " s, sensor update " << sensor_step_s << " s" << std::endl;
        }

        sys.DoStepDynamics(step_size);
    }

    std::cout << "Mesh rebuilds: " << terrain.GetNumRebuilds() << "; LOD and mesh " << terrain_s << " s in total, sensor updates "
              << sensor_s << " s (" << sensor_after_rebuild_s << " s of it in the " << rebuild_steps << " steps with a rebuild)"
              << std::endl;
    std::cout << "Images saved to " << out_dir << std::endl;
    return 0;
}
