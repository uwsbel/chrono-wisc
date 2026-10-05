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
// VIPER rover fording a small crater in CRM regolith at a lunar landing site,
// seen from the side by a Chrono::Sensor camera that follows it. The wheels
// turn at 4 rad/s (about 0.8 m/s); about 5 s after they start, the front wheel
// drops over the crater's rim, the rover pitches down into it, slows, and
// climbs back out.
//
// A 14 s run: with --save --no-window, the camera's frames (30 per second) go
// to DEMO_OUTPUT/PLANET_Viper_CRM_CraterFording/side, for example for
//   ffmpeg -framerate 30 -i side/frame_%d.png -pix_fmt yuv420p crater_fording.mp4
// (real speed). See PlanetViperCRMSensor.h for the flags.
//
// =============================================================================

#include "PlanetViperCRMSensor.h"

int main(int argc, char* argv[]) {
    ViperCRMScenario scenario;
    scenario.name = "PLANET_Viper_CRM_CraterFording";
    scenario.start_x = -27, scenario.start_y = 22, scenario.start_heading = 60;  // heading for the crater
    scenario.speed = 4;
    scenario.end_time = 14;
    scenario.rate = 30;
    scenario.camera = "side";
    return RunViperCRMSensor(argc, argv, scenario);
}
