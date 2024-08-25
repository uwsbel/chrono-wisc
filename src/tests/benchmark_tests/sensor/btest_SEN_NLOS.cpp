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
unsigned int image_width = 500;
unsigned int image_height = 500;

// Camera's horizontal field of view
float fov = (45)* (CH_PI/180) ; //(float)CH_PI / 2.;

// Lag (in seconds) between sensing and when data becomes accessible
float lag = .05f;

// Exposure (in seconds) of each image
float exposure_time = 0.02f;

int alias_factor = 64;

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

    unsigned int shader = (unsigned int)BSDFType::DIFFUSE;
    auto red = chrono_types::make_shared<ChVisualMaterial>();
    red->SetDiffuseColor({1, 0, 0});
    red->SetSpecularColor({1.f, 1.f, 1.f});
    red->SetBSDF(shader);

    auto green = chrono_types::make_shared<ChVisualMaterial>();
    green->SetDiffuseColor({0, 1, 0});
    green->SetSpecularColor({1.f, 1.f, 1.f});
    green->SetBSDF(shader);

    auto grey = chrono_types::make_shared<ChVisualMaterial>();
    grey->SetDiffuseColor({.5, .5, .5});
    grey->SetSpecularColor({.5f, .5f, .5f});
    grey->SetBSDF(shader);

    auto white = chrono_types::make_shared<ChVisualMaterial>();
    white->SetDiffuseColor({1, 1, 1});
    white->SetSpecularColor({0, 0, 0});
    white->SetBSDF(shader);

    auto relay_wall_mat = chrono_types::make_shared<ChVisualMaterial>();
    relay_wall_mat->SetDiffuseColor({1, 1, 1});
    relay_wall_mat->SetSpecularColor({0, 0, 0});
    relay_wall_mat->SetIsHiddenObject(false);
    relay_wall_mat->SetBSDF(shader);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(10, 10, .1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sys.Add(floor);
    //floor->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(grey);

    auto ceiling = chrono_types::make_shared<ChBodyEasyBox>(10, 10, .1, 1000, true, false);
    ceiling->SetPos({0, 0, 5});
    ceiling->SetFixed(true);
    //sys.Add(ceiling);
    ceiling->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(grey);

    auto left_wall = chrono_types::make_shared<ChBodyEasyBox>(10, .1, 5, 1000, true, false);
    left_wall->SetPos({0, 5, 2.5});
    left_wall->SetFixed(true);
    //sys.Add(left_wall);
    left_wall->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(green);

    auto right_wall = chrono_types::make_shared<ChBodyEasyBox>(10, .1, 5, 1000, true, false);
    right_wall->SetPos({0, -5, 2.5});
    right_wall->SetFixed(true);
    //sys.Add(right_wall);
    right_wall->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(green);

    auto relay_wall = chrono_types::make_shared<ChBodyEasyBox>(.1, 10, 5, 1000, true, false);
    //back_wall->SetPos({5, 0, 2.5});
    relay_wall->SetPos({0,0,0});
    relay_wall->SetFixed(true);
    sys.Add(relay_wall);
    relay_wall->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(relay_wall_mat);

    auto mid_wall = chrono_types::make_shared<ChBodyEasyBox>(5, .1, 5, 1000, true, false);
    mid_wall->SetPos({-2.5, 0, 2.5});
    mid_wall->SetFixed(true);
    //sys.Add(mid_wall);
    mid_wall->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(red);

    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, false); 
    //auto box_body = chrono_types::make_shared<ChBodyEasySphere>(.5, 1000, true, false);
    box_body->SetPos({-1, 0, 0});
    box_body->SetFixed(true);
    //sys.Add(box_body);
    box_body->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(green);

    auto sphere_body = chrono_types::make_shared<ChBodyEasySphere>(.5, 1000, true, false);
    sphere_body->SetPos({-1, 0, 0});
    sphere_body->SetFixed(true);
    sys.Add(sphere_body);
    sphere_body->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(red);

  /*  auto rod = chrono_types::make_shared<ChBodyEasyBox>(2, .1, .2, 1000, true, false);
    rod->SetPos({4, -.5, .1});
    rod->SetFixed(true);
    sys.Add(rod);
    rod->GetVisualModel()->GetShapeInstances()[0].first->AddMaterial(red);*/


    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->SetRayRecursions(4);
    Background b;
    b.mode = BackgroundMode::SOLID_COLOR;
    b.color_zenith = {0,0,0};
    manager->scene->SetBackground(b);
    manager->SetVerbose(false);
    float intensity = 1e0f;
    //manager->scene->AddPointLight({-1,0,0}, {1,1,1}, 5.0f); // 0.0f, -2.5f, 4.8f// 2.0f / 2, 1.8902f / 2, 1.7568f / 2
    //manager->scene->AddAreaLight({0.0f, -2.5, 4.8f}, {2.0f/2, 1.8902f/2, 1.7568f/2}, 5.0f, {1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f});
    float rotX = 0 * (CH_PI / 180);
    //manager->scene->AddSpotLight({0, -2.5, 2}, {5, 1.5, 2.5}, {1e4,1e4,1e4}, 5.0f, 1*(CH_PI/180), .5*(CH_PI/180));
    manager->scene->AddSpotLight({-0.25,0,0.5}, {0,0,0.5}, {intensity,intensity,intensity}, 5.0f, fov, fov*0.5);
    // -------------------------------------------------------
    // Create a camera and add it to the sensor manager
    // -------------------------------------------------------
   // chrono::ChFrame<double> offset_pose2({0, -2.5, 2}, QuatFromAngleZ(rotX));
    chrono::ChFrame<double> offset_pose2({-0.25,0,0.5}, QuatFromAngleZ(rotX));
    //chrono::ChFrame<double> offset_pose2({0, -10, 2}, QuatFromAngleZ(CH_PI_2));
    auto cam = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose2,  // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov,           // camera's horizontal field of view
                                                         4,  // supersample factor for antialiasing
                                                         lens_model,    // FOV
                                                         true);         // use global illumination or not
    cam->SetName("Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);
    cam->SetIntegrator(Integrator::PATH);
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "NLOS Camera"));
    if (save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "NLOS_SteadyState/"));
    //manager->AddSensor(cam);

    float tmin = 0.2;
    float tmax = 4;
    float tBins = 128;
    auto tcam = chrono_types::make_shared<ChTransientSensor>(floor,         // body camera is attached to
                                                            update_rate,   // update rate in Hz
                                                            offset_pose2,  // offset pose
                                                            image_width,   // image width
                                                            image_height,  // image height
                                                            fov,           // camera's horizontal field of view
                                                            alias_factor,  // supersample factor for antialiasing
                                                            lens_model,    // FOV
                                                            true,
                                                            Integrator::TRANSIENT,
                                                            tmin,
                                                            tmax,
                                                            tBins
                                                            );         // use global illumination or not
    tcam->SetName("Transient Camera");
    tcam->SetLag(lag);
    tcam->SetCollectionWindow(exposure_time);
    tcam->SetNLOSaserSamples(true);
    tcam->SetDiscardDirectPaths(false);
    tcam->SetFilterBounces(-1);
    tcam->SetNLOSHiddenGeometrySampling(false);
    tcam->SetGamma(1);
    /* if (vis)
         cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Transient Camera"));
     */
    if (save)
        tcam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "NLOS/"));
    manager->AddSensor(tcam);

 
    UserFloat4BufferPtr buffPtr;
    // ---------------
    // Simulate system
    // ---------------
    // Demonstration shows cameras panning around a stationary mesh.
    // Each camera begins on opposite sides of the object, but rotate at the same speed
    // float orbit_radius = 10.f;
    // float orbit_rate = 2.5;
    float ch_time = 0.0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    int i = 0;
    while (i < 6) { //ch_time < end_time
        // Update sensor manager
        // Will render/save/filter automatically
        manager->Update();

        // Perform step of dynamics
        sys.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float)sys.GetChTime();
        i++;
    }

    //buffPtr = tcam->GetMostRecentBuffer<UserFloat4BufferPtr>();
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}