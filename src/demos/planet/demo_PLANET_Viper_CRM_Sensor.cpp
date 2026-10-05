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
// VIPER rover on CRM regolith at a lunar landing site, seen through
// Chrono::Sensor: a mast camera and a chase camera on the smoothest drive near
// the site, at VIPER's traverse pace (about 0.1 m/s), under a low Sun behind
// the rover. See PlanetViperCRMSensor.h for the scene and its flags.
//
// =============================================================================

#include "PlanetViperCRMSensor.h"

int main(int argc, char* argv[]) {
    return RunViperCRMSensor(argc, argv, ViperCRMScenario());
}
