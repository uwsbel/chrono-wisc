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
// Authors: Patrick Chen
// =============================================================================
//
// Wave-domain radar on a highway.
//
// The scene is arranged so that each of the failure modes a radar has, and a ray-cast point cloud
// sensor does not, shows up in one run:
//
//   - a metal guardrail down the right side reflects the truck into a mirrored bearing, so a ghost
//     track appears where there is no vehicle;
//   - a motorcycle riding alongside a truck is a weak target next to a strong one, and drops in
//     and out as the difference between its return and the local detection threshold changes;
//   - a car cuts in from the right, and is reported several cycles after it is first illuminated,
//     because a track has to be confirmed before it is reported;
//   - an oncoming car closes faster than the waveform can measure, so its velocity folds.
//
// None of these is scripted. They follow from the signal, and the visualization window shows the
// stage each one comes from.
//
// =============================================================================

#include <cstdio>
#include <iostream>
#include <string>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChConstants.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterPhysRadarAccess.h"
#include "chrono_sensor/filters/ChFilterPhysRadarVisualize.h"
#include "chrono_sensor/sensors/ChPhysRadarSensor.h"

using namespace chrono;
using namespace chrono::sensor;

// Visual material class ids, which are also the keys the radar material registry is built on.
enum SurfaceClass : unsigned short {
    SURFACE_ROAD = 1,
    SURFACE_GUARDRAIL = 2,
    SURFACE_VEHICLE = 3,
    SURFACE_MOTORCYCLE = 4
};

const double step_size = 2e-3;
const double end_time = 14.0;
const float update_rate = 14.0f;  // one coherent cycle every 71 ms
const double ego_speed = 22.0;    // [m/s]

// Add a box that coasts at a constant velocity. Gravity is off, so a body given a velocity keeps
// it, and the per-body kinematics the radar reads are exact.
std::shared_ptr<ChBody> AddBox(ChSystem& sys,
                               const ChVector3d& size,
                               const ChVector3d& position,
                               const ChVector3d& velocity,
                               unsigned short surface_class,
                               const ChColor& color) {
    auto body = chrono_types::make_shared<ChBodyEasyBox>(size.x(), size.y(), size.z(), 1000.0, true, false);
    body->SetPos(position);
    body->SetPosDt(velocity);
    body->SetFixed(false);

    auto material = chrono_types::make_shared<ChVisualMaterial>();
    material->SetDiffuseColor(color);
    material->SetClassID(surface_class);
    material->SetMetallic(surface_class == SURFACE_ROAD ? 0.f : 1.f);
    material->SetRoughness(surface_class == SURFACE_ROAD ? 0.9f : 0.3f);
    body->GetVisualModel()->GetShapeInstances()[0].shape->AddMaterial(material);

    sys.Add(body);
    return body;
}

int main(int argc, char* argv[]) {
    std::string output_directory;
    bool print_detections = false;
    for (int i = 1; i < argc; i++) {
        const std::string arg = argv[i];
        if (arg == "--save" && i + 1 < argc)
            output_directory = argv[++i];
        else if (arg == "--detections")
            print_detections = true;
        else if (arg == "--help") {
            std::cout << "usage: demo_SEN_phys_radar [--save <directory>] [--detections]\n"
                      << "  --save        also write every visualization frame to <directory> as a PNG\n"
                      << "  --detections  print the detection list, not just the object list\n";
            return 0;
        }
    }

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Road surface. Wide enough that the beam always lands on it, which is where ground clutter and
    // the two-ray interference between the direct and the reflected path come from.
    AddBox(sys, ChVector3d(400, 24, 0.2), ChVector3d(100, 0, -0.1), VNULL, SURFACE_ROAD, ChColor(0.25f, 0.25f, 0.27f));

    // Guardrail down the right-hand shoulder: a long metal plate, and the classic source of the
    // indirect paths that put a detection where no vehicle is.
    AddBox(sys, ChVector3d(300, 0.1, 1.0), ChVector3d(100, -5.5, 0.5), VNULL, SURFACE_GUARDRAIL,
           ChColor(0.7f, 0.7f, 0.75f));

    // Ego vehicle, carrying the radar behind its front bumper.
    auto ego = AddBox(sys, ChVector3d(4.5, 1.9, 1.5), ChVector3d(0, 0, 0.75), ChVector3d(ego_speed, 0, 0),
                      SURFACE_VEHICLE, ChColor(0.2f, 0.4f, 0.8f));

    // Truck ahead in lane, closing slowly: a large target that dominates its neighbourhood.
    AddBox(sys, ChVector3d(12, 2.5, 3.6), ChVector3d(95, 0.2, 1.8), ChVector3d(20, 0, 0), SURFACE_VEHICLE,
           ChColor(0.75f, 0.7f, 0.2f));

    // Motorcycle in the next lane, well clear of the truck in range but inside the skirt the truck
    // spreads along it: a weak target whose detection comes and goes.
    AddBox(sys, ChVector3d(2.1, 0.6, 1.3), ChVector3d(75, -3.2, 0.65), ChVector3d(20.3, 0, 0), SURFACE_MOTORCYCLE,
           ChColor(0.8f, 0.3f, 0.2f));

    // Car cutting in from beyond the right edge of the beam, entering it part-way through the run.
    auto cut_in = AddBox(sys, ChVector3d(4.4, 1.9, 1.5), ChVector3d(25, -18.0, 0.75), ChVector3d(24, 1.5, 0),
                         SURFACE_VEHICLE, ChColor(0.2f, 0.7f, 0.3f));

    // Oncoming car in the opposite carriageway, closing faster than the waveform can measure.
    AddBox(sys, ChVector3d(4.4, 1.9, 1.5), ChVector3d(260, 5.2, 0.75), ChVector3d(-30, 0, 0), SURFACE_VEHICLE,
           ChColor(0.8f, 0.8f, 0.8f));

    // ----------------------------------------------------------------------------------------
    // Radar
    // ----------------------------------------------------------------------------------------

    ChRadarModelConfig config = MakeDefaultFrontRadarConfig();
    config.name = "highway_front_radar";
    config.dsp.cfar_type = ChRadarCfarType::CA;
    // One false alarm per cycle would be 1e-4 across a map this size, which is more than a
    // tracker can absorb; automotive detectors run several orders tighter.
    config.dsp.false_alarm_rate = 1e-6;
    config.ray_tracing.max_bounces = 3;
    // The road and the guardrail return as strongly as the vehicles do, all of it at the Doppler
    // the ego motion puts it at. Rejecting that ridge is what an automotive radar does; the price
    // is that a stopped vehicle is rejected with it, which is a real failure mode and not a
    // shortcut taken here.
    config.dsp.suppress_stationary = true;
    std::cout << config.GetDescription() << std::endl;

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->SetRayRecursions(config.ray_tracing.max_bounces + 1);
    manager->scene->SetAmbientLight({0.2f, 0.2f, 0.2f});

    auto radar = chrono_types::make_shared<ChPhysRadarSensor>(
        ego, update_rate, ChFrame<double>({2.4, 0, -0.25}, QuatFromAngleZ(0)), config);
    radar->SetName("front radar");

    // Surfaces the radar knows about. Everything else falls back to a response derived from the
    // visual material, which is why an untagged scene still produces a plausible signal.
    auto& materials = radar->GetMaterialRegistry();
    materials.Assign(SURFACE_ROAD, "asphalt");
    materials.Assign(SURFACE_GUARDRAIL, "metal");
    materials.Assign(SURFACE_VEHICLE, "vehicle_body");
    materials.Assign(SURFACE_MOTORCYCLE, "pedestrian");

    auto visualize = chrono_types::make_shared<ChFilterPhysRadarVisualize>(1280, 800, "Radar signal chain");
    if (!output_directory.empty())
        visualize->SetSaveDirectory(output_directory);
    radar->PushFilter(visualize);

    auto access = chrono_types::make_shared<ChFilterPhysRadarAccess>(ChRadarFrameContents::DETECTIONS);
    radar->PushFilter(access);

    manager->AddSensor(radar);

    // ----------------------------------------------------------------------------------------
    // Run
    // ----------------------------------------------------------------------------------------

    unsigned int reported_cycle = 0;
    while (sys.GetChTime() < end_time) {
        // Steer the cut-in car back into lane once it has crossed the lane line.
        if (cut_in->GetPos().y() > -0.3)
            cut_in->SetPosDt(ChVector3d(19, 0, 0));

        manager->Update();
        sys.DoStepDynamics(step_size);

        auto frame = access->GetBuffer();
        if (frame && frame->LaunchedCount > reported_cycle) {
            reported_cycle = frame->LaunchedCount;
            std::printf("t=%6.2f s  paths=%7u  dropped=%6u  detections=%4u  tracks=%2u\n", frame->TimeStamp,
                        frame->NumPaths, frame->DroppedPaths, frame->NumDetections,
                        (unsigned int)frame->Objects.size());
            if (print_detections) {
                for (unsigned int i = 0; i < frame->NumDetections; i++) {
                    const RadarDetection& d = frame->Detections[i];
                    std::printf("    detection  r=%7.2f m  rate=%7.2f m/s  az=%6.1f deg  snr=%5.1f dB  "
                                "rcs=%6.1f dBsm  truth id=%u%s\n",
                                d.range, d.range_rate, d.azimuth * CH_RAD_TO_DEG, d.snr_db, d.rcs_dbsm, d.object_id,
                                (d.flags & RADAR_DET_MULTIPATH) ? "  [multipath]" : "");
                }
            }
            for (const RadarObject& object : frame->Objects) {
                std::printf("    track %3u  x=%7.2f m  y=%7.2f m  vx=%6.2f m/s  range rate=%7.2f m/s  "
                            "rcs=%6.1f dBsm  p=%.2f  truth id=%u\n",
                            object.id, object.x, object.y, object.vx, object.range_rate, object.rcs_dbsm,
                            object.existence_probability, object.object_id);
            }
        }
    }

    return 0;
}
