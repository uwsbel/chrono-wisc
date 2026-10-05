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
// VIPER rover driving up a hill of CRM regolith at a lunar landing site, seen
// from the side by a Chrono::Sensor camera that follows it. The wheels turn at
// 0.8 rad/s; on the slope they slip more and more, and the rover slows to a
// crawl, digging its wheels in, about 4 m up. The Sun is 30 degrees up, off
// the rover's left, lighting the side the camera sees.
//
// A 125 s run: with --save --no-window, the camera's frames (10 per second) go
// to DEMO_OUTPUT/PLANET_Viper_CRM_HillClimb/side, for example for
//   ffmpeg -framerate 30 -i side/frame_%d.png -pix_fmt yuv420p hill_climb.mp4
// (three times real speed). See PlanetViperCRMSensor.h for the flags.
//
// =============================================================================

#include "PlanetViperCRMSensor.h"

int main(int argc, char* argv[]) {
    ViperCRMScenario scenario;
    scenario.name = "PLANET_Viper_CRM_HillClimb";
    scenario.start_x = 6, scenario.start_y = -8, scenario.start_heading = 285;  // at the foot of the hill, facing up it
    scenario.speed = 0.8;
    scenario.end_time = 125;
    scenario.rate = 10;
    scenario.camera = "side";
    scenario.sun_elevation = 30;
    scenario.sun_azimuth = 165;
    return RunViperCRMSensor(argc, argv, scenario);
}
