# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2026 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Patrick Chen
# =============================================================================
#
# Wave-domain radar from Python.
#
# Builds the same highway as demo_SEN_phys_radar.cpp, runs the radar over it, and reads the
# signal chain back as numpy arrays: the range-Doppler power map, the detection threshold the
# constant false alarm rate stage derived from it, and the detections and tracks that survived.
#
# =============================================================================

import math

import numpy as np

import pychrono as chrono
import pychrono.sensor as sens

# Visual material class ids, also the keys of the radar material registry.
SURFACE_ROAD = 1
SURFACE_GUARDRAIL = 2
SURFACE_VEHICLE = 3
SURFACE_MOTORCYCLE = 4

STEP_SIZE = 2e-3
END_TIME = 6.0
UPDATE_RATE = 14.0
EGO_SPEED = 22.0


def add_box(system, size, position, velocity, surface_class, color):
    """Add a box that coasts at a constant velocity; gravity is off, so it keeps it."""
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000.0, True, False)
    body.SetPos(chrono.ChVector3d(*position))
    body.SetPosDt(chrono.ChVector3d(*velocity))
    body.SetFixed(False)

    material = chrono.ChVisualMaterial()
    material.SetDiffuseColor(chrono.ChColor(*color))
    material.SetClassID(surface_class)
    material.SetMetallic(0.0 if surface_class == SURFACE_ROAD else 1.0)
    material.SetRoughness(0.9 if surface_class == SURFACE_ROAD else 0.3)
    body.GetVisualModel().GetShapeInstances()[0].shape.AddMaterial(material)

    system.Add(body)
    return body


def main():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    add_box(system, (400, 24, 0.2), (100, 0, -0.1), (0, 0, 0), SURFACE_ROAD, (0.25, 0.25, 0.27))
    add_box(system, (300, 0.1, 1.0), (100, -5.5, 0.5), (0, 0, 0), SURFACE_GUARDRAIL, (0.7, 0.7, 0.75))
    ego = add_box(system, (4.5, 1.9, 1.5), (0, 0, 0.75), (EGO_SPEED, 0, 0), SURFACE_VEHICLE, (0.2, 0.4, 0.8))
    add_box(system, (12, 2.5, 3.6), (95, 0.2, 1.8), (20, 0, 0), SURFACE_VEHICLE, (0.75, 0.7, 0.2))
    add_box(system, (2.1, 0.6, 1.3), (75, -3.2, 0.65), (20.3, 0, 0), SURFACE_MOTORCYCLE, (0.8, 0.3, 0.2))
    add_box(system, (4.4, 1.9, 1.5), (25, -18.0, 0.75), (24, 1.5, 0), SURFACE_VEHICLE, (0.2, 0.7, 0.3))
    add_box(system, (4.4, 1.9, 1.5), (260, 5.2, 0.75), (-30, 0, 0), SURFACE_VEHICLE, (0.8, 0.8, 0.8))

    config = sens.MakeDefaultFrontRadarConfig()
    config.name = "highway_front_radar"
    config.dsp.false_alarm_rate = 1e-6
    config.dsp.suppress_stationary = True
    print(config.GetDescription())

    manager = sens.ChSensorManager(system)
    manager.SetRayRecursions(config.ray_tracing.max_bounces + 1)
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.2, 0.2, 0.2))

    radar = sens.ChPhysRadarSensor(
        ego, UPDATE_RATE, chrono.ChFramed(chrono.ChVector3d(2.4, 0, -0.25), chrono.QuatFromAngleZ(0)), config
    )
    radar.SetName("front radar")

    materials = radar.GetMaterialRegistry()
    materials.Assign(SURFACE_ROAD, "asphalt")
    materials.Assign(SURFACE_GUARDRAIL, "metal")
    materials.Assign(SURFACE_VEHICLE, "vehicle_body")
    materials.Assign(SURFACE_MOTORCYCLE, "pedestrian")

    radar.PushFilter(sens.ChFilterPhysRadarVisualize(1280, 800, "Radar signal chain"))

    # MAPS also brings back the power map, the threshold map and the cell provenance.
    access = sens.ChFilterPhysRadarAccess(sens.ChRadarFrameContents_MAPS)
    radar.PushFilter(access)

    manager.AddSensor(radar)

    reported_cycle = 0
    while system.GetChTime() < END_TIME:
        manager.Update()
        system.DoStepDynamics(STEP_SIZE)

        frame = access.GetBuffer()
        if frame is None or frame.LaunchedCount <= reported_cycle:
            continue
        reported_cycle = frame.LaunchedCount

        power = frame.GetPowerMapData()
        threshold = frame.GetThresholdMapData()
        if power.size == 0:
            continue
        power = power[:, :, 0]
        threshold = threshold[:, :, 0]

        # How far the strongest cell sits above the threshold it had to cross.
        margin_db = 10.0 * np.log10(np.maximum(power, 1e-30) / np.maximum(threshold, 1e-30))
        floor_db = 10.0 * math.log10(frame.NoisePowerPerCell)
        peak_db = 10.0 * math.log10(max(power.max(), 1e-30))

        print(
            "t={:6.2f} s  paths={:7d}  cube={}x{}  peak={:5.1f} dB over noise  "
            "best margin={:5.1f} dB  detections={:3d}  tracks={:2d}".format(
                frame.TimeStamp,
                frame.NumPaths,
                frame.NumDopplerBins,
                frame.NumRangeBins,
                peak_db - floor_db,
                margin_db.max(),
                frame.NumDetections,
                len(frame.Objects),
            )
        )

        for i in range(frame.NumDetections):
            d = frame.GetDetection(i)
            print(
                "    detection  r={:7.2f} m  rate={:7.2f} m/s  az={:6.1f} deg  snr={:5.1f} dB  "
                "rcs={:6.1f} dBsm  truth id={}{}".format(
                    d.range,
                    d.range_rate,
                    d.azimuth * chrono.CH_RAD_TO_DEG,
                    d.snr_db,
                    d.rcs_dbsm,
                    d.object_id,
                    "  [multipath]" if d.flags & sens.RADAR_DET_MULTIPATH else "",
                )
            )

        for obj in frame.Objects:
            print(
                "    track {:3d}  x={:7.2f} m  y={:7.2f} m  vx={:6.2f} m/s  range rate={:7.2f} m/s  "
                "rcs={:6.1f} dBsm  p={:.2f}  truth id={}".format(
                    obj.id, obj.x, obj.y, obj.vx, obj.range_rate, obj.rcs_dbsm,
                    obj.existence_probability, obj.object_id,
                )
            )


if __name__ == "__main__":
    main()
