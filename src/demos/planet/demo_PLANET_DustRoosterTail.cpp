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
// The dust trail of a wheel driven across lunar regolith, seen through
// Chrono::Sensor. A wheel the size of the Apollo Lunar Roving Vehicle's rolls
// over level ground at the Rover's cruising speed with a little slip, and a
// ChDustField throws the soil it loosens into ballistic flight; the cameras
// draw the dust as a participating medium, scattering sunlight as the
// regolith's Hapke parameters say grains of it do. The run prints how high and
// how far behind the wheel the trail reaches, for comparison with the Rover's
// rooster tails in the Apollo 16 films, which Hersh et al. 2012 (Am. J. Phys.
// 80, 452) measured at a ground speed of about 2.5 m/s.
//
// Two cameras ride alongside the wheel: one down-Sun, with the Sun behind it
// (the regolith's grains scatter back toward the Sun), and one looking toward
// the Sun, where the trail is backlit. The dust is drawn only by the Vulkan RT
// render backend.
//
// Flags: --speed <m/s> (default 2.5), --slip <0-1> (default 0.1), --time <s>
// (default 6), --sun-elevation <deg> (default 20), --save writes the camera
// images and a top view of the dust's optical depth to the demo output
// directory, --no-window skips the camera windows.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/terrain/ChDustField.h"
#include "chrono_planet/planets/moon/ChMoon.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "PlanetDust.h"

using namespace chrono;
using namespace chrono::planet;
using namespace chrono::vehicle;
using namespace chrono::sensor;

// Lunar Roving Vehicle wheel: 81.8 cm across, 23 cm wide
const double wheel_radius = 0.409;
const double wheel_width = 0.23;
const double sinkage = 0.01;  // m

// Regolith: Hapke parameters as in demo_ROBOT_Viper_SCM_Sensor, and the albedo tint of the ground
const float hapke_w = 0.32357f, hapke_b = 0.23955f, hapke_c = 0.30452f;
const ChColor regolith_albedo(0.7f, 0.7f, 0.7f);

const double step_size = 1e-3;
const float sensor_rate = 20.0f;
const float sun_azimuth = float(-90 * CH_DEG_TO_RAD);  // the Sun to the south, on the down-Sun camera's side
const float sun_irradiance = 2.0f;

std::shared_ptr<ChVisualMaterial> RegolithMaterial(const ChColor& albedo) {
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetAmbientColor({0.0f, 0.0f, 0.0f});
    mat->SetDiffuseColor(albedo);
    mat->SetSpecularColor({0.0f, 0.0f, 0.0f});
    mat->SetRoughness(1.0f);
    mat->SetMetallic(0.0f);
    mat->SetBSDF(BSDFType::HAPKE);
    mat->SetHapkeParameters(hapke_w, hapke_b, hapke_c, 1.80238f, 0.07145f, 0.3f, float(23.4 * CH_DEG_TO_RAD));
    return mat;
}

// The value below which the given share of the total weight lies
double WeightedQuantile(std::vector<std::pair<double, double>>& samples, double share) {
    if (samples.empty())
        return 0;
    std::sort(samples.begin(), samples.end());
    double total = 0;
    for (const auto& s : samples)
        total += s.second;
    double sum = 0;
    for (const auto& s : samples) {
        sum += s.second;
        if (sum >= share * total)
            return s.first;
    }
    return samples.back().first;
}

void AddCamera(ChSensorManager& manager, std::shared_ptr<ChBody> mount, const ChFrame<double>& pose, const std::string& name, bool window, const std::string& save_dir) {
    auto cam = chrono_types::make_shared<ChCameraSensor>(mount, sensor_rate, pose, 1280, 720, float(60 * CH_DEG_TO_RAD));
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

    double speed = 2.5;
    double slip = 0.1;
    double end_time = 6.0;
    double sun_elevation_deg = 20;
    bool save = false;
    bool window = true;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--speed") && i + 1 < argc)
            speed = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--slip") && i + 1 < argc)
            slip = std::clamp(std::stod(argv[++i]), 0.0, 0.95);
        else if (!std::strcmp(argv[i], "--time") && i + 1 < argc)
            end_time = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--sun-elevation") && i + 1 < argc)
            sun_elevation_deg = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--save"))
            save = true;
        else if (!std::strcmp(argv[i], "--no-window"))
            window = false;
    }

    const std::string out_dir = GetChronoOutputPath() + "PLANET_DustRoosterTail/";
    const std::string save_dir = save ? out_dir : "";
    if (!CreateOutputDirectory(out_dir)) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // A wheel rolling along +x about the +y axis turns at a positive rate; slip makes the rim outrun the ground.
    const double omega = speed / (wheel_radius * (1 - slip));
    std::cout << "Wheel speed " << speed << " m/s, slip " << slip << ", rim speed " << omega * wheel_radius << " m/s: grains leaving at the rim speed straight up would rise "
              << std::pow(omega * wheel_radius, 2) / (2 * moon::kGravity) << " m" << std::endl;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -moon::kGravity));

    // Level ground, its top at z = 0
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    auto ground_shape = chrono_types::make_shared<ChVisualShapeBox>(400, 60, 1);
    ground_shape->AddMaterial(RegolithMaterial(regolith_albedo));
    ground->AddVisualShape(ground_shape, ChFrame<>(ChVector3d(150, 0, -0.5)));
    sys.AddBody(ground);

    // The wheel, moved along its path each step
    auto wheel = chrono_types::make_shared<ChBody>();
    wheel->SetFixed(true);
    auto wheel_shape = chrono_types::make_shared<ChVisualShapeCylinder>(wheel_radius, wheel_width);
    auto wheel_mat = chrono_types::make_shared<ChVisualMaterial>();
    wheel_mat->SetDiffuseColor({0.25f, 0.25f, 0.27f});
    wheel_mat->SetRoughness(0.8f);
    wheel_shape->AddMaterial(wheel_mat);
    wheel->AddVisualShape(wheel_shape, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));
    sys.AddBody(wheel);
    auto wheel_center = [&](double t) { return ChVector3d(speed * t, 0, wheel_radius - sinkage); };
    wheel->SetPos(wheel_center(0));

    // Camera rig, following the wheel
    auto rig = chrono_types::make_shared<ChBody>();
    rig->SetFixed(true);
    sys.AddBody(rig);

    // Dust over level ground, on a grid reaching back along the trail
    ChDustField::Params params;
    params.gravity = moon::kGravity;
    ChDustField dust(params, [](double, double) { return 0.0; });
    ChDustField::GridSpec spec;
    spec.voxel = 0.06;
    spec.nx = 240;
    spec.ny = 80;
    spec.nz = 56;
    spec.below = 0.3;
    dust.SetGridSpec(spec);
    const double grid_lead = 3.0;  // m of grid ahead of the wheel

    // Sensors: sunlight only, under a black sky
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    const float sun_elevation = float(sun_elevation_deg * CH_DEG_TO_RAD);
    const ChVector3d sun_dir(std::cos(sun_elevation) * std::cos(sun_azimuth), std::cos(sun_elevation) * std::sin(sun_azimuth), std::sin(sun_elevation));
    manager->scene->AddDirectionalLight(ChColor(sun_irradiance, sun_irradiance, sun_irradiance), sun_elevation, sun_azimuth);
    manager->scene->SetAmbientLight({0.f, 0.f, 0.f});
    Background background;
    background.mode = BackgroundMode::SOLID_COLOR;
    background.color_zenith = ChVector3f(0.f, 0.f, 0.f);
    background.color_horizon = ChVector3f(0.f, 0.f, 0.f);
    manager->scene->SetBackground(background);

    // Down-Sun from the south side, looking north at the trail; backlit from the north side, looking south
    AddCamera(*manager, rig, ChFrame<>(ChVector3d(-1.5, -7.0, 1.2), QuatFromAngleZ(CH_PI_2) * QuatFromAngleY(6 * CH_DEG_TO_RAD)), "down_sun", window, save_dir);
    AddCamera(*manager, rig, ChFrame<>(ChVector3d(-1.5, 7.0, 1.2), QuatFromAngleZ(-CH_PI_2) * QuatFromAngleY(6 * CH_DEG_TO_RAD)), "backlit", window, save_dir);

    const int sensor_steps = static_cast<int>(std::round(1.0 / (sensor_rate * step_size)));
    // Extent of the trail: the height and the distance behind the wheel holding 95% of the dust's cross section
    const double extent_share = 0.95;
    double trail_height = 0, trail_length = 0;
    std::vector<std::pair<double, double>> heights, behind;
    bool drawn = true;
    for (int step = 0; sys.GetChTime() < end_time; ++step) {
        const double time = sys.GetChTime();
        const ChVector3d center = wheel_center(time);
        wheel->SetPos(center);
        wheel->SetRot(QuatFromAngleY(omega * time));
        rig->SetPos(ChVector3d(center.x(), 0, 0));

        dust.EmitFromWheels({{center, ChVector3d(0, 1, 0), ChVector3d(speed, 0, 0), omega, wheel_radius, wheel_width}}, time, step_size);

        if (step % sensor_steps == 0) {
            dust.Update(time);
            heights.clear();
            behind.clear();
            for (const auto& p : dust.GetParticles()) {
                if (p.time0 > time)
                    continue;
                const ChVector3d pos = dust.Position(p, time);
                const double cross_section = p.mass * dust.MassExtinction(p.bin);
                heights.push_back({pos.z(), cross_section});
                behind.push_back({center.x() - pos.x(), cross_section});
            }
            trail_height = WeightedQuantile(heights, extent_share);
            trail_length = WeightedQuantile(behind, extent_share);
            dust.UpdateGrid(time, ChVector3d(center.x() + grid_lead - 0.5 * spec.nx * spec.voxel, 0, 0), sun_dir);
            drawn = SetDustVolume(*manager, dust, hapke_w, hapke_b, hapke_c, regolith_albedo);
        }
        manager->Update();
        sys.DoStepDynamics(step_size);

        if (step % 1000 == 0) {
            const auto& s = dust.GetStats();
            std::cout << "t = " << time << " s: " << s.particles << " particles, " << s.aloft * 1e3 << " g aloft, " << s.emitted * 1e3 << " g released, trail " << trail_height
                      << " m high and " << trail_length << " m long" << std::endl;
        }
    }

    if (!drawn)
        std::cout << "This render backend does not draw the dust; only Vulkan RT does." << std::endl;
    if (save) {
        dust.WriteOpticalDepthImage(out_dir + "optical_depth.pgm");
        std::cout << "Images and a top view of the dust's optical depth saved to " << out_dir << std::endl;
    }
    std::cout << "Dust trail: " << extent_share * 100 << "% of its cross section below " << trail_height << " m and within " << trail_length << " m behind the wheel" << std::endl;
    return 0;
}
