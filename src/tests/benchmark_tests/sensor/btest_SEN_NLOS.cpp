// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nevindu M. Batagoda
// =============================================================================
//
// Chrono demonstration of a camera sensor.
// Generates a mesh object and rotates camera sensor around the mesh.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChTransientSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

using namespace chrono;
using namespace chrono::sensor;

// -----------------------------------------------------------------------------
// Camera parameters
// -----------------------------------------------------------------------------

// Noise model attached to the sensor
enum NoiseModel {
    CONST_NORMAL,     // Gaussian noise with constant mean and standard deviation
    PIXEL_DEPENDENT,  // Pixel dependent gaussian noise
    NONE              // No noise model
};
NoiseModel noise_model = PIXEL_DEPENDENT;

// Camera lens model
// Either PINHOLE or SPHERICAL
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;

// Update rate in Hz
float update_rate = 30;

// Image width and height
unsigned int image_width = 1280;
unsigned int image_height = 720;

// Camera's horizontal field of view
float fov = (float)CH_PI / 2.;

// Lag (in seconds) between sensing and when data becomes accessible
float lag = .05f;

// Exposure (in seconds) of each image
float exposure_time = 0.02f;

int alias_factor = 2;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-2;

// Simulation end time
float end_time = 20.0f;

// Save camera images
bool save = true;

// Render camera images
bool vis = true;

// Output directory
const std::string out_dir = "SENSOR_OUTPUT/";

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;

    // ---------------------------------------
    // add set of boxes to be visualized by camera
    // ---------------------------------------

    
    
    //auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    //floor->SetPos({0, 0, 0});
    //floor->SetFixed(true);
    //sys.Add(floor);

    auto red = chrono_types::make_shared<ChVisualMaterial>();
    red->SetDiffuseColor({1, 0, 0});
    red->SetSpecularColor({1.f, 1.f, 1.f});

    auto green = chrono_types::make_shared<ChVisualMaterial>();
    green->SetDiffuseColor({0, 1, 0});
    green->SetSpecularColor({1.f, 1.f, 1.f});

    auto grey = chrono_types::make_shared<ChVisualMaterial>();
    grey->SetDiffuseColor({.5, .5, .5});
    grey->SetSpecularColor({.5f, .5f, .5f});

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(10, 10, .1, 1000, true, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sys.Add(floor);
    floor->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(grey);

    auto ceiling = chrono_types::make_shared<ChBodyEasyBox>(10, 10, .1, 1000, true, false);
    ceiling->SetPos({0, 0, 5});
    ceiling->SetFixed(true);
    sys.Add(ceiling);
    ceiling->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(grey);

    auto left_wall = chrono_types::make_shared<ChBodyEasyBox>(10, .1, 5, 1000, true, false);
    left_wall->SetPos({0, 5, 2.5});
    left_wall->SetFixed(true);
    sys.Add(left_wall);
    left_wall->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(red);

    auto right_wall = chrono_types::make_shared<ChBodyEasyBox>(10, .1, 5, 1000, true, false);
    right_wall->SetPos({0, -5, 2.5});
    right_wall->SetFixed(true);
    sys.Add(right_wall);
    right_wall->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(green);

    auto back_wall = chrono_types::make_shared<ChBodyEasyBox>(.1, 10, 5, 1000, true, false);
    back_wall->SetPos({5, 0, 2.5});
    back_wall->SetFixed(true);
    sys.Add(back_wall);
    back_wall->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(grey);

    auto mid_wall = chrono_types::make_shared<ChBodyEasyBox>(8, .1, 5, 1000, true, false);
    mid_wall->SetPos({-1, 0, 2.5});
    mid_wall->SetFixed(true);
    sys.Add(mid_wall);
    mid_wall->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(grey);

    auto box_body = chrono_types::make_shared<ChBodyEasySphere>(.5, 1000, true, false);
    box_body->SetPos({0, 2.5, .5});
    box_body->SetFixed(true);
    sys.Add(box_body);
    box_body->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(green);

  /*  auto rod = chrono_types::make_shared<ChBodyEasyBox>(2, .1, .2, 1000, true, false);
    rod->SetPos({4, -.5, .1});
    rod->SetFixed(true);
    sys.Add(rod);
    rod->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(red);*/


    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    //manager->scene->AddPointLight({0.0f, 0.0f, 3.8f}, {2.0f / 2, 1.8902f / 2, 1.7568f / 2}, 5.0f);
    manager->scene->AddAreaLight({0.0f, -2.5, 4.8f}, {2.0f/2, 1.8902f/2, 1.7568f/2}, 5.0f, {1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f});
    // -------------------------------------------------------
    // Create a camera and add it to the sensor manager
    // -------------------------------------------------------
    chrono::ChFrame<double> offset_pose2({-5, -2.5, 2}, QUNIT);
    //chrono::ChFrame<double> offset_pose2({0, -10, 2}, QuatFromAngleZ(CH_PI_2));
    auto cam = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose2,  // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // supersample factor for antialiasing
                                                         lens_model,    // FOV
                                                         true);         // use global illumination or not
    cam->SetName("NLOS Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "NLOS Camera"));
    if (save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "NLOS/"));
    manager->AddSensor(cam);

 

    // ---------------
    // Simulate system
    // ---------------
    // Demonstration shows cameras panning around a stationary mesh.
    // Each camera begins on opposite sides of the object, but rotate at the same speed
    // float orbit_radius = 10.f;
    // float orbit_rate = 2.5;
    float ch_time = 0.0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    while (ch_time < end_time) {
        // Update sensor manager
        // Will render/save/filter automatically
        manager->Update();

        // Perform step of dynamics
        sys.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float)sys.GetChTime();
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}