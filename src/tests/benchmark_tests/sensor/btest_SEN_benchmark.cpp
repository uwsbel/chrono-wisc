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
float update_rate = 2;

// Image width and height
unsigned int image_width = 1920;
unsigned int image_height = 760;

// Camera's horizontal field of view
float fov = (float)CH_PI / 2.;

// Lag (in seconds) between sensing and when data becomes accessible
float lag = .05f;

// Exposure (in seconds) of each image
float exposure_time = 0.02f;

int alias_factor = 1;

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

enum BenchmarkModel {
    STANFORD_BUNNY,
    STANFORD_DRAGON,
    BLENDER_SUZANNE,
};

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;

    // ---------------------------------------
    // add set of boxes to be visualized by camera
    // ---------------------------------------

    BenchmarkModel model = STANFORD_DRAGON;

    std::string obj_path = "";
    switch (model) {
        case STANFORD_BUNNY:
            obj_path = GetChronoDataFile("sensor/geometries/bunny.obj");
            break;
        case STANFORD_DRAGON:
            obj_path = GetChronoDataFile("sensor/geometries/dragon.obj");
            break;
        case BLENDER_SUZANNE:
            obj_path = GetChronoDataFile("sensor/geometries/suzanne.obj");
            break;
    }
    auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(obj_path), true, true);

    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("Box");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({0, 0, 0});
    mesh_body->AddVisualShape(trimesh_shape);
    mesh_body->SetFixed(true);
    mesh_body->SetRot(QuatFromAngleAxis(CH_PI_2, {0, 0, 1}) * QuatFromAngleAxis(CH_PI_2, {1, 0, 0}));
    //sys.Add(mesh_body);

    for (auto mat : trimesh_shape->GetMaterials()) {
        mat->SetBSDF((unsigned int)BSDFType::DIFFUSE);
    }

    auto floor_mat = chrono_types::make_shared<ChVisualMaterial>();
    floor_mat->SetAmbientColor({0.f, 0.f, 0.f});
    floor_mat->SetDiffuseColor({.5, .5, .5});
    floor_mat->SetSpecularColor({.0f, .0f, .0f});
    floor_mat->SetUseSpecularWorkflow(true);
    floor_mat->SetBSDF((unsigned int)BSDFType::DIFFUSE);

    auto box_mat = chrono_types::make_shared<ChVisualMaterial>();
    box_mat->SetAmbientColor({0.f, 0.f, 0.f});
    box_mat->SetDiffuseColor({1,0,0});
    box_mat->SetSpecularColor({.0f, .0f, .0f});
    box_mat->SetUseSpecularWorkflow(true);
    box_mat->SetBSDF((unsigned int)BSDFType::DIFFUSE);
    
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(200, 200, .1, 1000, true, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sys.Add(floor);
    {
        auto shape = floor->GetVisualModel()->GetShapeInstances()[0].first;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(floor_mat);
        } else {
            shape->GetMaterials()[0] = floor_mat;
        }
    }

    auto box = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, false);
    box->SetPos({0, 0, 0.5});
    box->SetFixed(true);
    sys.Add(box);
    {
        auto shape = box->GetVisualModel()->GetShapeInstances()[0].first;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(box_mat);
        } else {
            shape->GetMaterials()[0] = box_mat;
        }
    }



    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->SetRayRecursions(4);
    Background b;
    b.mode = BackgroundMode::SOLID_COLOR;
    b.color_horizon = {1,1,1};
    manager->scene->SetBackground(b);
    manager->scene->AddPointLight({0.0f, 0.0f, 10.f}, {1,1,1}, 1000.0f);
    //manager->scene->AddSpotLight({0.f, 0.0f, 2.f},{0,0,-1}, {10.f,10.f,10.f}, 1000.0f, CH_PI/3, CH_PI/6);
    //manager->scene->AddSpotLight(box, ChFramed({0, 0, 2.f}, QuatFromAngleY(90*CH_PI/180)), {10,10,10}, 5.f, CH_PI/3, CH_PI/6);
    ChVector3f light_pos = {0, 0, 2.f};
    unsigned int spot_id  = manager->scene->AddSpotLight(ChFramed(light_pos, QuatFromAngleY(90*CH_PI/180)), {10,10,10}, 5.f, CH_PI/3, CH_PI/6);
    //manager->scene->AddAreaLight({0.0f, 0.0f, 200.f}, {2.0f/2, 1.8902f/2, 1.7568f/2}, 1000.0f, {1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f});
    // -------------------------------------------------------
    // Create a camera and add it to the sensor manager
    // -------------------------------------------------------
    chrono::ChFrame<double> offset_pose2({-10, 0, 2}, QuatFromAngleY(0.f));
    // chrono::ChFrame<double> offset_pose2({-3, 0, 0}, QUNIT);
    auto cam = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose2,  // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // supersample factor for antialiasing
                                                         lens_model,    // FOV
                                                         true);         // use global illumination or not
    cam->SetName("Global Illum Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);
    cam->SetIntegrator(Integrator::PATH);
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Global Illumination"));
    if (save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "globalillum/"));
    manager->AddSensor(cam);

    auto cam2 = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                          update_rate,   // update rate in Hz
                                                          offset_pose2,  // offset pose
                                                          image_width,   // image width
                                                          image_height,  // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // supersample factor for antialiasing
                                                          lens_model,    // FOV
                                                          false);        // use global illumination or not
    cam2->SetName("Whitted Camera");
    cam2->SetIntegrator(Integrator::PATH);
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);
    if (vis)
        cam2->PushFilter(
            chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Whitted Ray Tracing"));
    if (save)
        cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "whitted/"));
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
    int step = 0;
    float end_dist = 5;
    float move_rate = (end_dist/end_time) * step_size;
    while (ch_time < end_time) {
        // Update sensor manager
        // Will render/save/filter automatically

        ChVector3f new_pos = box->GetPos() + ChVector3f(move_rate, 0, 0);
        light_pos = light_pos + ChVector3f(move_rate, 0, 0);
        box->SetPos(new_pos);
        manager->scene->UpdateLight(spot_id, ChFramed(light_pos, QuatFromAngleY(90 * CH_PI / 180)));
      /*  ChQuaternionf new_rot = box->GetRot() * QuatFromAngleAxis(0.1, ChVector3f(0, 0, 1));
        box->SetRot(new_rot);*/
        
        manager->Update();

        // Perform step of dynamics
        sys.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float)sys.GetChTime();
        step++;
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}