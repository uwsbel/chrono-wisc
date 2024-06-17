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
// Authors: Nevi
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

#include "chrono/assets/ChVisualShapeModelFile.h"

#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

#include <openvdb/openvdb.h>

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
NoiseModel noise_model = NONE;

// Camera lens model
// Either PINHOLE or FOV_LENS
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;

// Update rate in Hz
float update_rate = 30.f;

// Image width and height
unsigned int image_width = 1;// 1920;
unsigned int image_height = 1;//780;

// Camera's horizontal field of view
float fov = (float)CH_PI / 3.;

// Lag (in seconds) between sensing and when data becomes accessible
float lag = 0.f;

// Exposure (in seconds) of each image
float exposure_time = 0.f;

int alias_factor = 1;

bool use_gi = false;  // whether cameras should use global illumination

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-2;

// Simulation end time
float end_time = 200.0f;

// Save camera images
bool save = false;

// Render camera images
bool vis = true;

// Output directory
const std::string out_dir = "SENSOR_OUTPUT/VDB_DUST/";
openvdb::Vec3d volDims(0);

std::shared_ptr<openvdb::FloatGrid> readVDBGrid(std::string fileName) {
    openvdb::initialize();
    openvdb::io::File file(fileName);
    // Open the file.  This reads the file header, but not any grids.
    openvdb::GridBase::Ptr baseGrid;
    file.open();
    baseGrid = file.readGrid("density");

    openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
    openvdb::math::Transform::Ptr linearTransform = openvdb::math::Transform::createLinearTransform(0.1);
    grid->setTransform(linearTransform);
    // Print Grid information
    openvdb::Vec3d minBBox = grid->evalActiveVoxelBoundingBox().min().asVec3d();
    openvdb::Vec3d maxBBox = grid->evalActiveVoxelBoundingBox().max().asVec3d();

    openvdb::Vec3d minBBoxWS = grid->indexToWorld(minBBox);
    openvdb::Vec3d maxBBoxWS = grid->indexToWorld(maxBBox);

    volDims = openvdb::Vec3d(maxBBoxWS[0] - minBBoxWS[0], maxBBoxWS[1] - minBBoxWS[1], maxBBoxWS[2] - minBBoxWS[2]);
    int activeVoxels = grid->activeVoxelCount();

    // Save Grid
    openvdb::io::File("dustGrid.vdb").write({ grid });
    std::cout << "VDB Grid Written\n" << std::endl;

    // Print Grid information
    printf("############### VDB POINT GRID INFORMATION ################\n");
    printf("Voxel Size: %f %f %f\n", grid->voxelSize()[0], grid->voxelSize()[1], grid->voxelSize()[2]);
    printf("Grid Class: %d\n", grid->getGridClass());
    printf("Grid Type: %s\n", grid->gridType().c_str());
    printf("Upper Internal Nodes: %d\n", grid->tree().nodeCount()[2]);
    printf("Lower Internal Nodes: %d\n", grid->tree().nodeCount()[1]);
    printf("Leaf Nodes: %d\n", grid->tree().nodeCount()[0]);
    printf("Active Voxels: %d\n", grid->activeVoxelCount());
    printf("Min BBox: %f %f %f\n", minBBox[0], minBBox[1], minBBox[2]);
    printf("Max BBox: %f %f %f\n", maxBBox[0], maxBBox[1], maxBBox[2]);
    printf("Min BBox WorldSpace: %f %f %f\n", minBBoxWS[0], minBBoxWS[1], minBBoxWS[2]);
    printf("Max BBox WorldSpace: %f %f %f\n", maxBBoxWS[0], maxBBoxWS[1], maxBBoxWS[2]);
    printf("Volume Dimensions: %f %f %f\n", volDims[0], volDims[1], volDims[2]);
    printf("############### END #############\n");

    return grid;

}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;


    auto vis_mat3 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat3->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat3->SetDiffuseColor({.5, .5, .5});
    vis_mat3->SetSpecularColor({.0f, .0f, .0f});
    vis_mat3->SetUseSpecularWorkflow(true);
    vis_mat3->SetClassID(30000);
    vis_mat3->SetInstanceID(30000);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(20, 20, .1, 1000, true, false);
    floor->SetPos({0, 0, -1});
    floor->SetFixed(true);
    //sys.Add(floor);
    {
        auto shape = floor->GetVisualModel()->GetShapes()[0].first;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat3);
        } else {
            shape->GetMaterials()[0] = vis_mat3;
        }
    }

    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat->SetDiffuseColor({0.0, 1.0, 0.0});
    vis_mat->SetSpecularColor({1.f, 1.f, 1.f});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetShader(3);
    vis_mat->SetRoughness(.5f);
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(50000);

    auto ground_body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    ground_body->SetPos({0, 0, 0});
    ground_body->SetFixed(true);
    sys.Add(ground_body);

    // Make VDB Grid
    std::string vdbGridPath = "C:\\workspace\\data\\VDBFIles\\DustImpactVDB\\dust_impact_0010.vdb";
    std::shared_ptr<openvdb::FloatGrid> grid = readVDBGrid(vdbGridPath);


    //(float)volDims[0], (float)volDims[1], (float)volDims[2]
    std::cout << "Vol Dims: " << (float)volDims[0] << "," << (float)volDims[1] << "," << (float)volDims[2] << std::endl;
     auto vol_bbox = chrono_types::make_shared<ChNVDBVolume>((float)volDims[0], (float)volDims[1], (float)volDims[2], 1000,true);
     vol_bbox->SetPos({0, 0, 0});
     vol_bbox->SetFixed(true);
     sys.Add(vol_bbox);
    {
        auto shape = vol_bbox->GetVisualModel()->GetShapes()[0].first;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat);
        } else {
            shape->GetMaterials()[0] = vis_mat;
        }
    }

    // -----------------------
    // Create a sensor manager
    // -----------------------
    float intensity = 1.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100, 100, 100}, {intensity, intensity, intensity}, 500);
    manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    manager->scene->SetBackground(b);

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    //-800, 0, 50
    chrono::ChFrame<double> offset_pose1({-80, 1, 1}, QuatFromAngleAxis(0, {0, 1, 0}));
    auto cam = chrono_types::make_shared<ChCameraSensor>(ground_body,   // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose1,  // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // super sampling factor
                                                         lens_model,    // lens model type
                                                         use_gi, 
                                                          2.2);
    cam->SetName("Camera Sensor");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);

    // --------------------------------------------------------------------
    // Create a filter graph for post-processing the images from the camera
    // --------------------------------------------------------------------

    // Add a noise model filter to the camera sensor
    switch (noise_model) {
        case CONST_NORMAL:
            cam->PushFilter(chrono_types::make_shared<ChFilterCameraNoiseConstNormal>(0.f, .0004f));
            break;
        case PIXEL_DEPENDENT:
            cam->PushFilter(chrono_types::make_shared<ChFilterCameraNoisePixDep>(.0004f, .0004f));
            break;
        case NONE:
            // Don't add any noise models
            break;
    }

    // Renders the image at current point in the filter graph
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Global Illumination"));

    if (save)
        // Save the current image to a png file at the specified path
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "rgb/"));


    // add sensor to the manager
    manager->AddSensor(cam);

    manager->Update();


    // ---------------
    // Simulate system
    // ---------------
    // Demonstration shows cameras panning around a stationary mesh.
    // Each camera begins on opposite sides of the object, but rotate at the same speed
    float orbit_radius = 10.f;
    float orbit_rate = 0.1f;
    float ch_time = 0.0f;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    UserRGBA8BufferPtr rgba8_ptr;
    UserR8BufferPtr r8_ptr;
    UserDepthBufferPtr depth_ptr;

    int current_step = 0;
    unsigned int render_steps = (unsigned int)round(1 / (update_rate * step_size));

    while (ch_time < end_time) {
        // Rotate the cameras around the mesh at a fixed rate
       /* cam->SetOffsetPose(chrono::ChFrame<double>(
            {orbit_radius * cos(ch_time * orbit_rate), orbit_radius * sin(ch_time * orbit_rate), 2},
            QuatFromAngleAxis(ch_time * orbit_rate + CH_PI, {0, 0, 1})));*/

         
        if (current_step % render_steps == 0) {
            //timerNVDB.start();
            manager->scene->AddVDBGrid(grid);
            //timerNVDB.stop();
            //std::cout << "Making VDB Voxel Grid:" << timerNVDB.GetTimeMilliseconds() << "ms" << std::endl;
            //timerNVDB.reset();
        }
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
