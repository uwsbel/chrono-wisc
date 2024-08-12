#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/optix/ChNVDBVolume.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>

#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/ParticlesToLevelSet.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>

std::vector<float> readBinaryFile(const std::string &binaryFilePath) {
    std::ifstream binaryFile(binaryFilePath, std::ios::binary);
    if (!binaryFile.is_open()) {
        std::cerr << "Could not open the file: " << binaryFilePath << std::endl;
        return {};
    }

    // Get the file size
    binaryFile.seekg(0, std::ios::end);
    std::streampos fileSize = binaryFile.tellg();
    binaryFile.seekg(0, std::ios::beg);

    // Calculate the number of floats
    size_t numFloats = fileSize / sizeof(float);

    // Read the data into a buffer
    std::vector<float> buffer(numFloats);
    binaryFile.read(reinterpret_cast<char*>(buffer.data()), fileSize);

    binaryFile.close();
    return buffer;
}



// Function to extract the numerical part from the filename for sorting
int extractNumber(const std::string &filename) {
    std::string numberStr;
    for (char c : filename) {
        if (std::isdigit(c)) {
            numberStr += c;
        }
    }
    return std::stoi(numberStr);
}

using namespace chrono::sensor;
using namespace chrono;
// Sensor params
// Noise model attached to the sensor
enum NoiseModel {
    CONST_NORMAL,     // Gaussian noise with constant mean and standard deviation
    PIXEL_DEPENDENT,  // Pixel dependent gaussian noise
    NONE              // No noise model
};
NoiseModel noise_model = NONE;

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
float lag = 0.0f;
// Exposure (in seconds) of each image
float exposure_time = 0.00f;
int alias_factor = 1;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------
// Save camera images
bool save = true;
// Render camera images
bool vis = true;
// Output directory
const std::string sensor_out_dir = "SENSOR_OUTPUT/NVDBTest/";

bool use_gi = false;

bool firstInst = true;
std::vector<int> idList;
int prevActiveVoxels = 0;

// forward declarations
void createVoxelGrid(std::vector<openvdb::Vec3R>& points, ChSystemNSC& sys);
std::vector<openvdb::Vec3R> floatToVec3R(float* floatBuffer, int npts);

int main() {
    std::string binaryDir = "D:\\workspace\\chrono-wisc-build\\bin\\Release\\DEMO_OUTPUT\\FSI_Viper_RealSlope\\particles\\binary";  // Change this to your directory containing binary files
    std::vector<std::string> filePaths;

    for (const auto &entry : std::filesystem::directory_iterator(binaryDir)) {
        if (entry.path().extension() == ".bin") {
            filePaths.push_back(entry.path().string());
        }
    }

    // Sort the file paths numerically
    std::sort(filePaths.begin(), filePaths.end(), [](const std::string &a, const std::string &b) {
        return extractNumber(a) < extractNumber(b);
    });

    std::vector<std::vector<float>> allBuffers;
   
     int fileCounter = 0;
     int fromRange = 190;
     int toRange = 500;
    for (const auto &binaryFilePath : filePaths) {
        if (fileCounter++ < fromRange)
            continue;
        std::vector<float> buffer = readBinaryFile(binaryFilePath);
        if (!buffer.empty()) {
            allBuffers.push_back(buffer);
            std::cout << "Read " << buffer.size()/6 << " entries " << binaryFilePath << std::endl;
        }
        if (fileCounter > toRange)
           break;
    }

    // Create a Chrono physical system
    ChSystemNSC sysMBS;
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
    vis_mat->SetDiffuseColor({1,1,1}); //0.29f, 0.29f, 0.235f
    vis_mat->SetSpecularColor({1, 1, 1});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(1.0f);
    vis_mat->SetBSDF(1);
    vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(20000);

    float scal = 10.f;
    // auto vol_bbox = chrono_types::make_shared<ChNVDBVolume>(6,4,1, 1000,
    //                                                        true);  
    // vol_bbox->SetPos({0, 0, 0});                                   
    // vol_bbox->SetFixed(true);
    // sysMBS.Add(vol_bbox);
    // {
    //    auto shape = vol_bbox->GetVisualModel()->GetShapes()[0].first;
    //    if (shape->GetNumMaterials() == 0) {
    //        shape->AddMaterial(vis_mat);
    //    } else {
    //        shape->GetMaterials()[0] = vis_mat;
    //    }
    // }

    // Add spheres for each particle in the buffer
    //int counter = 0;
    //for (const auto &buffer : allBuffers) {
    //    for (size_t i = 0; i < buffer.size(); i += 6) {
    //        if (counter % 100 == 0) {
    //            auto sphere = chrono_types::make_shared<ChBodyEasySphere>(0.01, 1000, true, false);
    //            sphere->SetPos({buffer[i], buffer[i + 1], buffer[i + 2]});
    //            sphere->SetFixed(true);
    //            sysMBS.Add(sphere);
    //            {
    //                auto shape = sphere->GetVisualModel()->GetShapes()[0].first;
    //                if (shape->GetNumMaterials() == 0) {
    //                    shape->AddMaterial(vis_mat);
    //                } else {
    //                    shape->GetMaterials()[0] = vis_mat;
    //                }
    //            }
    //        }
    //        counter++;
    //    }
    //    break;
    //}

    // Make VDB Grid
    

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetFixed(true);
    sysMBS.Add(floor);

    // Create a Sensor manager
    float intensity = 1.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sysMBS);
    manager->scene->AddPointLight({0, 0, 10}, {intensity, intensity, intensity}, 500);
    manager->scene->SetAmbientLight({.1, .1, .1});
    Background b;
    b.mode = BackgroundMode::SOLID_COLOR;
    b.color_zenith = ChVector3d(0.1f, 0.2f, 0.4f);  // 0.1f, 0.2f, 0.4f
    manager->scene->SetBackground(b);
    manager->SetVerbose(false);

    chrono::ChFrame<double> offset_pose1({0,2,1}, QuatFromAngleAxis(-CH_PI_2, {0, 0, 1}));  // Q_from_AngAxis(CH_PI_4, {0, 1, 0})  //-1200, -252, 100
    auto cam = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose1,  // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // super sampling factor
                                                         lens_model,    // lens model type
                                                         false, 2.2);
    cam->SetName("Third Person Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Third Person Camera"));

    if (save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "CRM_DEMO_THIRD_PERSON_VIEW_RealSlope/"));
   

    chrono::ChFrame<double> offset_pose2({-0.f, -1.7, 0.5}, QuatFromAngleAxis(.2, {-2, 3, 9.75}));
    auto cam2 = chrono_types::make_shared<ChCameraSensor>(floor,  // body camera is attached to
                                                          update_rate,                     // update rate in Hz
                                                          offset_pose2,                    // offset pose
                                                          image_width,                     // image width
                                                          image_height,                    // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // super sampling factor
                                                          lens_model,    // lens model type
                                                          false, 2.2);
    cam2->SetName("Wheel Camera");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);
    if (vis)
        cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Wheel Camera"));

    if (save)
        cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Wheel_Cam_RealSlope/"));
   

    chrono::ChFrame<double> offset_pose3({-2.f, 0, .5f}, QuatFromAngleAxis(.2, {0, 1, 0}));
    // chrono::ChFrame<double> offset_pose3({0, 0, 5.f}, Q_from_AngAxis(CH_PI_2, {0, 1, 0}));
    auto cam3 = chrono_types::make_shared<ChCameraSensor>(floor,  // body camera is attached to
                                                          update_rate,                     // update rate in Hz
                                                          offset_pose3,                    // offset pose
                                                          image_width,                     // image width
                                                          image_height,                    // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // super sampling factor
                                                          lens_model,    // lens model type
                                                          false, 2.2);
    cam3->SetName("Rear Camera");
    cam3->SetLag(lag);
    cam3->SetCollectionWindow(exposure_time);
    if (vis)
        cam3->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Rear Camera"));

    if (save)
        cam3->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Rear_Cam_RealSlope/"));
    

    // chrono::ChFrame<double> offset_pose4({-2.f, 0, .5f}, Q_from_AngAxis(.2, {0, 1, 0}));
    chrono::ChFrame<double> offset_pose4({0, 0, 8.f}, QuatFromAngleAxis(CH_PI_2, {0, 1, 0}));
    auto cam4 = chrono_types::make_shared<ChCameraSensor>(floor,  // body camera is attached to
                                                          update_rate,                     // update rate in Hz
                                                          offset_pose4,                    // offset pose
                                                          image_width,                     // image width
                                                          image_height,                    // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // super sampling factor
                                                          lens_model,    // lens model type
                                                          false, 2.2);
    cam4->SetName("Top Camera");
    cam4->SetLag(lag);
    cam4->SetCollectionWindow(exposure_time);
    if (vis)
        cam4->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Top Camera"));

    if (save)
        cam4->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_out_dir + "Top_Cam_RealSensor/"));
    
    manager->AddSensor(cam);
    manager->AddSensor(cam2);
    manager->AddSensor(cam3);
    manager->AddSensor(cam4);

    int totalBuffers = allBuffers.size();
    int currentBuffer = 0;
    float* h_points;
    int n_pts;
    float step_size = 1e-2;
    std::vector<openvdb::Vec3R> vdbBuffer;

    unsigned int render_steps = (unsigned int)round(1 / (update_rate * step_size));
    int current_step = 0;
    while(true) {
        std::cout << "Frame: " << currentBuffer << std::endl;

         if (current_step % render_steps == 0) {
            h_points = allBuffers[currentBuffer].data();
            n_pts = allBuffers[currentBuffer].size() / 6;
            manager->scene->SetFSIParticles(h_points);
            manager->scene->SetFSINumFSIParticles(n_pts);

            vdbBuffer = floatToVec3R(allBuffers[currentBuffer].data(), n_pts);
            createVoxelGrid(vdbBuffer, sysMBS);
            vdbBuffer.clear();
            currentBuffer++;
        }

        manager->Update();

        sysMBS.DoStepDynamics(step_size);

        current_step++;
        if (currentBuffer >= totalBuffers)
           currentBuffer = 0;
    }

}


std::vector<openvdb::Vec3R> floatToVec3R(float* floatBuffer, int npts) {
    std::vector<openvdb::Vec3R> vec3RBuffer;
    for (size_t i = 0; i < npts; i++) {
        vec3RBuffer.emplace_back(floatBuffer[6*i], floatBuffer[6*i + 1], floatBuffer[6*i + 2]);
    }
    return vec3RBuffer;
}


void createVoxelGrid(std::vector<openvdb::Vec3R>& points, ChSystemNSC& sys) {
    std::cout << "Creating OpenVDB Voxel Grid" << std::endl;
    openvdb::initialize();
    openvdb::points::PointAttributeVector<openvdb::Vec3R> positionsWrapper(points);
    float r = 0.05f;
    int pointsPerVoxel = 10;
    std::vector<float> radius(points.size(), r);
    float voxelSize = openvdb::points::computeVoxelSize(positionsWrapper, pointsPerVoxel);
    // Print the voxel-size to cout
    std::cout << "VoxelSize=" << voxelSize << std::endl;
    // Create a transform using this voxel-size.
    openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(voxelSize);

    openvdb::tools::PointIndexGrid::Ptr pointIndexGrid =
        openvdb::tools::createPointIndexGrid<openvdb::tools::PointIndexGrid>(positionsWrapper, *transform);

    openvdb::points::PointDataGrid::Ptr grid =
        openvdb::points::createPointDataGrid<openvdb::points::NullCodec, openvdb::points::PointDataGrid>(points,
                                                                                                         *transform);

    using Codec = openvdb::points::FixedPointCodec</*1-byte=*/false, openvdb::points::UnitRange>;
    openvdb::points::TypedAttributeArray<float, Codec>::registerType();
    openvdb::NamePair radiusAttribute = openvdb::points::TypedAttributeArray<float, Codec>::attributeType();
    openvdb::points::appendAttribute(grid->tree(), "pscale", radiusAttribute);
    // Create a wrapper around the radius vector.
    openvdb::points::PointAttributeVector<float> radiusWrapper(radius);
    // Populate the "pscale" attribute on the points
    openvdb::points::populateAttribute<openvdb::points::PointDataTree, openvdb::tools::PointIndexTree,
                                       openvdb::points::PointAttributeVector<float>>(
        grid->tree(), pointIndexGrid->tree(), "pscale", radiusWrapper);

    // Set the name of the grid
    grid->setName("FSIPointData");
    openvdb::Vec3d minBBox = grid->evalActiveVoxelBoundingBox().min().asVec3d();
    openvdb::Vec3d maxBBox = grid->evalActiveVoxelBoundingBox().max().asVec3d();

    int activeVoxels = grid->activeVoxelCount();
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
    printf("############### END #############\n");
    
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
    vis_mat->SetDiffuseColor({1, 1, 1});  // 0.29f, 0.29f, 0.235f
    vis_mat->SetSpecularColor({1, 1, 1});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(1.0f);
    vis_mat->SetBSDF(1);
    vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(20000); 

    // remove all existing spheres
    //if (!idList.empty()) {
    //    std::cout << "Num Sphere IDs: " << idList.size() << std::endl;
    //    for (int id : idList) {
    //        auto sphere = sys.GetBodies()[id];
    //        sys.RemoveBody(sphere);
    //        std::cout << "Num Bodies: " << sys.GetBodies().size() << std::endl;
    //    }
    //    std::cout << "Spheres Removed" << std::endl;
    //    idList.clear();
    //    std::cout << "Num Sphere IDs after clearing: " << idList.size() << std::endl;
    //    prevActiveVoxels = 0;
    //}
    int voxelCount = 0;
    int numVoxelsToAdd = activeVoxels - prevActiveVoxels;
    int numAdds = 0;
    int numUpdates = 0;
    for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf) {
        for (auto iter(leaf->cbeginValueOn()); iter; ++iter) {
            //const float value = iter.getValue();
            const openvdb::Coord coord = iter.getCoord();
            openvdb::Vec3d voxelPos = grid->indexToWorld(coord);
            //std::wcout << "Voxel Pos: (" << voxelPos[0] << "," << voxelPos[1] << "," << voxelPos[2] << std::endl;
            // Update existing spheres
           /* if (!firstInst && voxelCount >= 500000)
                std::cout << numVoxelsToAdd << " " << prevActiveVoxels << " " << voxelCount << " " << idList.size()
                          << " " << numAdds << " " << numUpdates << std::endl;*/
            if (!idList.empty() && voxelCount < prevActiveVoxels) {
               /* if (voxelCount >= 500000)
                    std::cout << "Updating sphere: " << idList[voxelCount] << " Total Bodies: "<< sys.GetBodies().size() << std::endl;*/
                numUpdates++;
                //std::cout << voxelCount << "," << idList.size() << std::endl;
                auto voxelBody = sys.GetBodies()[idList[voxelCount]];
                voxelBody->SetPos({voxelPos.x(), voxelPos.y(), voxelPos.z()});
            }
            //Create a sphere for each point
            if (numVoxelsToAdd > 0 && voxelCount >= prevActiveVoxels) {
               /* if (!firstInst && voxelCount >= 500000) {
                    std::cout << "Adding Sphere" << std::endl;
                }*/
                numAdds++;
                auto voxelBody = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);
                //auto voxelBody = chrono_types::make_shared<ChBodyEasyBox>(r, r, r, 1000, true, false);
                voxelBody->SetPos({voxelPos.x(), voxelPos.y(), voxelPos.z()});
                voxelBody->SetFixed(true);
                sys.Add(voxelBody);
                {
                    auto shape = voxelBody->GetVisualModel()->GetShapes()[0].first;
                    if (shape->GetNumMaterials() == 0) {
                        shape->AddMaterial(vis_mat);
                    } else {
                        shape->GetMaterials()[0] = vis_mat;
                    }
                }
                
               /* if (voxelCount > 500000)
                    std::cout << "BodyList: "<< sys.GetBodies().size() <<  " Adding Sphere " << sphere->GetIndex() << std::endl;*/
                idList.emplace_back(voxelBody->GetIndex());
            } 
            voxelCount++;
        }
    }
    prevActiveVoxels = voxelCount;
    std::wcout << "Num Voxels: " << voxelCount << std::endl;
    // Iterate over points
    // Iterate over all the leaf nodes in the grid.
    //int sphereCount = 0;
    //for (auto leafIter = grid->tree().cbeginLeaf(); leafIter; ++leafIter) {
    //    // Verify the leaf origin.
    //    // std::cout << "Leaf" << leafIter->origin() << std::endl;
    //    // Extract the position attribute from the leaf by name (P is position).
    //    const openvdb::points::AttributeArray& positionArray = leafIter->constAttributeArray("P");
    //    // Extract the radius attribute from the leaf by name (pscale is radius).
    //    const openvdb::points::AttributeArray& radiusArray = leafIter->constAttributeArray("pscale");
    //    // Create read-only handles for position and radius.
    //    openvdb::points::AttributeHandle<openvdb::Vec3f> positionHandle(positionArray);
    //    openvdb::points::AttributeHandle<float> radiusHandle(radiusArray);
    //    // Iterate over the point indices in the leaf.
    //    int i = 0;
    //    for (auto indexIter = leafIter->beginIndexOn(); indexIter; ++indexIter) {
    //        i++;
    //        // Extract the voxel-space position of the point.
    //        openvdb::Vec3f voxelPosition = positionHandle.get(*indexIter);
    //        // Extract the world-space position of the voxel.
    //        openvdb::Vec3d xyz = indexIter.getCoord().asVec3d();
    //        // Compute the world-space position of the point.
    //        openvdb::Vec3f worldPosition = grid->transform().indexToWorld(voxelPosition + xyz);
    //        // Extract the radius of the point.
    //        float radius = radiusHandle.get(*indexIter);
    //        // Verify the index, world-space position and radius of the point.
    //        //std::cout << "* PointIndex=[" << *indexIter << "] ";
    //        //std::cout << "WorldPosition=" << worldPosition << " ";
    //        //std::cout << "Radius=" << radius << std::endl;

    //        // Create a sphere for each point
    //        auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    //        vis_mat->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
    //        vis_mat->SetDiffuseColor({1, 1, 1});  // 0.29f, 0.29f, 0.235f
    //        vis_mat->SetSpecularColor({1, 1, 1});
    //        vis_mat->SetUseSpecularWorkflow(true);
    //        vis_mat->SetRoughness(1.0f);
    //        vis_mat->SetBSDF(1);
    //        vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f, 23.4f * (CH_PI / 180));
    //        vis_mat->SetClassID(30000);
    //        vis_mat->SetInstanceID(20000);
    //        if (firstInst) {
    //            //std::cout << "Adding Sphere" << std::endl;
    //            auto sphere = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);
    //            sphere->SetPos({worldPosition.x(), worldPosition.y(), worldPosition.z()});
    //            sphere->SetFixed(true);
    //            sys.Add(sphere);
    //            {
    //                auto shape = sphere->GetVisualModel()->GetShapes()[0].first;
    //                if (shape->GetNumMaterials() == 0) {
    //                    shape->AddMaterial(vis_mat);
    //                } else {
    //                    shape->GetMaterials()[0] = vis_mat;
    //                }
    //            }
    //            sphereCount++;
    //            idList.emplace_back(sphere->GetIdentifier());
    //        } else {
    //            for (int id : idList) {
    //                auto sphere = sys.GetBodies()[id];
    //                sphere->SetPos({worldPosition.x(), worldPosition.y(), worldPosition.z()});
    //            }
    //            sphereCount++;
    //        }
    //    }
    //    std::cout << "Num Leaf point indices: " << i << std::endl;
    //}
    firstInst = false;
   // std::cout << "Added/Updated " << sphereCount << "spheres" << std::endl;
}