// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist, Yan Xiao
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
float fov = (float)CH_PI / 2.;

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
    auto mmesh =
        ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("sensor/geometries/box.obj"), true, true);

    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("Box");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({0, 0, 0});
    mesh_body->AddVisualShape(trimesh_shape);
    mesh_body->SetFixed(true);
    sys.Add(mesh_body);

    for (auto mat : trimesh_shape->GetMaterials()) {
        mat->SetBSDF((int)BSDFType::DIFFUSE);
    }


    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sys.Add(floor);


    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    Background b;
    b.mode = BackgroundMode::SOLID_COLOR;
    b.color_zenith = {0, 0, 0};
    manager->scene->SetBackground(b);
    manager->scene->AddPointLight({0.0f, 0.0f, 3.8f}, {1.f,1.f,1.f}, 5.0f); //{2.0f / 2, 1.8902f / 2, 1.7568f / 2}
    manager->SetRayRecursions(4);
    //manager->scene->SetAmbientLight({.1,.1,.1});
    //manager->scene->AddAreaLight({0.0f, 0.0f, 3.8f}, {2.0f/2, 1.8902f/2, 1.7568f/2}, 5.0f, {1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f});
    // -------------------------------------------------------
    // Create a camera and add it to the sensor manager
    // -------------------------------------------------------
       chrono::ChFrame<double> offset_pose2({-7, 0, 2}, QUNIT);
     //chrono::ChFrame<double> offset_pose2({-3, 0, 0}, QUNIT);
        auto cam1 = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                             update_rate,   // update rate in Hz
                                                             offset_pose2,  // offset pose
                                                             image_width,   // image width
                                                             image_height,  // image height
                                                             fov,           // camera's horizontal field of view
                                                             alias_factor,  // supersample factor for antialiasing
                                                             lens_model,    // FOV
                                                             true,
                                                             Integrator::PATH);         // use global illumination or not
        cam1->SetName("Camera");
        cam1->SetLag(lag);
        cam1->SetCollectionWindow(exposure_time);
        if (vis)
            cam1->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Camera"));
        if (save)
            cam1->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "transience_steady_state/"));
        //manager->AddSensor(cam1);

   
       float tmin = 6;
       float tmax = 25;
       float tBins = 128;
       auto cam = chrono_types::make_shared<ChTransientSensor>(floor,         // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose2,  // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,
                                                         lens_model,
                                                         false,
                                                         Integrator::TRANSIENT,
                                                         tmin,
                                                         tmax,
                                                         tBins
                                                         ); 
        cam->SetName("Transient Camera");
        cam->SetLag(lag);
        cam->SetCollectionWindow(exposure_time);
        cam->SetNLOSHiddenGeometrySampling(false);
        cam->SetNLOSaserSamples(false);
       /*if (vis)
            cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Transient Camera")); */
        if (save)
            cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "transience/"));
       manager->AddSensor(cam);

       auto cam2 = chrono_types::make_shared<ChTransientSensor>(floor,         // body camera is attached to
                                                                update_rate,   // update rate in Hz
                                                                offset_pose2,  // offset pose
                                                                image_width,   // image width
                                                                image_height,  // image height
                                                                fov,           // camera's horizontal field of view
                                                                alias_factor,
                                                                lens_model,
                                                                false,
                                                                Integrator::TIMEGATED
                                                                );
        cam2->SetName("Time Gated Camera");
        cam2->SetLag(lag);
        cam2->SetCollectionWindow(exposure_time);
        cam2->SetWindowSize(.1f);
        cam2->SetTimeGatedMode(TIMEGATED_MODE::BOX);
        cam2->SetSampleFactor(4);
        cam2->SetTargetDist(10.f);
        if (vis)
             cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "TimeGated Camera")); 
        if (save)
            cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "timegated/"));
        //manager->AddSensor(cam2);




    // ---------------
    // Simulate system
    // ---------------
    // Demonstration shows cameras panning around a stationary mesh.
    // Each camera begins on opposite sides of the object, but rotate at the same speed
    // float orbit_radius = 10.f;
    // float orbit_rate = 2.5;
    float ch_time = 0.0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();


    ch_time = (float)sys.GetChTime();
    int i = 0;
    while (i < 6) {
         // Update sensor manager
         // Will render/save/filter automatically
         manager->Update();

         // Perform step of dynamics
         sys.DoStepDynamics(step_size);

         // Get the current time of the simulation
         ch_time = (float)sys.GetChTime();
         i++;
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}