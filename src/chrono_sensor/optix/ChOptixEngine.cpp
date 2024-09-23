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
// Authors: Asher Elmquist, Han Wang
// =============================================================================
//
// OptiX rendering engine for processing jobs for sensing. Jobs are defined on
// each sensor as a graph.
//
// =============================================================================

#define PROFILE false

#include "chrono_sensor/optix/ChOptixEngine.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <optix_stubs.h>
#include <optix_function_table_definition.h>

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChTransientSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChTransientSensor.h"
#include "chrono_sensor/optix/ChOptixUtils.h"

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/assets/ChVisualShapePath.h"
#include "chrono/assets/ChVisualShapeRoundedBox.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_sensor/optix/ChNVDBVolume.h"
#include <random>

#include "chrono_sensor/cuda/cuda_utils.cuh"

#ifdef USE_SENSOR_NVDB
#include <openvdb/openvdb.h>
#endif

namespace chrono {
namespace sensor {


// using namespace optix;
ChOptixEngine::ChOptixEngine(ChSystem* sys, int device_id, int max_scene_reflections, bool verbose)
    : m_verbose(verbose), m_deviceId(device_id), m_recursions(max_scene_reflections), m_sceneThread() {
    m_sceneThread.start = false;
    m_sceneThread.terminate = false;
    m_sceneThread.done = true;  // thread is done to begin with (no work to complete)
    m_system = sys;
    Initialize();
}
ChOptixEngine::~ChOptixEngine() {
    StopAllThreads();  // if it hasn't been stopped yet, stop it ourselves
    // cleanup ChOptixGeometry and ChOptixPipeline before destroying the context
    cudaDeviceSynchronize();
    m_geometry->Cleanup();
    m_pipeline->Cleanup();
    // cleanup lights
    cudaFree(reinterpret_cast<void*>(md_params));
    // cleanup device context parameters
    cudaFree(reinterpret_cast<void*>(m_params.lights));
    optixDeviceContextDestroy(m_context);
}

void ChOptixEngine::Initialize() {
    cudaFree(0);
    OptixDeviceContext context;
    CUcontext cuCtx = 0;  // zero means take the current context, TODO: enable multigpu
    OPTIX_ERROR_CHECK(optixInit());
    OptixDeviceContextOptions options = {};
    options.logCallbackFunction = &optix_log_callback;

    if (m_verbose) {
        options.validationMode = OPTIX_DEVICE_CONTEXT_VALIDATION_MODE_ALL;
        options.logCallbackLevel = 4;
    } else {
        options.validationMode = OPTIX_DEVICE_CONTEXT_VALIDATION_MODE_OFF;
        options.logCallbackLevel = 2;
    }

    OPTIX_ERROR_CHECK(optixDeviceContextCreate(cuCtx, &options, &context));
    m_context = context;

    // defaults to no lights
    m_params.lights = {};
    /*m_params.arealights = {};
    m_params.arealights = 0;*/
    m_params.num_lights = 0;
    m_params.ambient_light_color = make_float3(0.0f, 0.0f, 0.0f);  // make_float3(0.1f, 0.1f, 0.1f);  // default value
    m_params.max_depth = m_recursions;
    m_params.scene_epsilon = 1.e-3f;    // TODO: determine a good value for this
    m_params.importance_cutoff = .01f;  /// TODO: determine a good value for this

    m_params.transient_buffer = {};
    //m_params.integrator = Integrator::PATH;
    m_params.window_size = 1.f;
    m_params.timegated_mode = TIMEGATED_MODE::BOX;
    m_params.target_dist = 1.f;
    #ifdef USE_SENSOR_NVDB
        m_params.handle_ptr = nullptr;
        m_params.normal_handle_ptr = nullptr;
    #else
    m_params.handle_ptr = 0;
    m_params.normal_handle_ptr = 0;
    #endif  // USE_SENSOR_NVDB
    
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_params), sizeof(ContextParameters)));
    m_params.root = {};

    m_geometry = chrono_types::make_shared<ChOptixGeometry>(m_context);
    m_pipeline = chrono_types::make_shared<ChOptixPipeline>(m_context, m_recursions, m_verbose);

    // TODO: enable multigpu
}

void ChOptixEngine::AssignSensor(std::shared_ptr<ChOptixSensor> sensor) {
    // all optix sensors should be treated the same. Up to the sensor to specify a difference by using a unique launch
    // kernel and set of filters
    {
        // std::cout << "Assigning a sensor\n";
        std::unique_lock<std::mutex> lck(m_sceneThread.mutex);
        // std::cout << "going to wait for lock. done=" << m_sceneThread.done << "\n";
        while (!m_sceneThread.done) {
            m_sceneThread.cv.wait(lck);
        }
        // std::cout << "Done waiting for lock\n";

        if (std::find(m_assignedSensor.begin(), m_assignedSensor.end(), sensor) != m_assignedSensor.end()) {
            std::cerr << "WARNING: This sensor already exists in manager. Ignoring this addition\n";
            return;
        }

        m_assignedSensor.push_back(sensor);
        m_cameraStartFrames.push_back(sensor->GetParent()->GetVisualModelFrame());
        m_cameraStartFrames_set.push_back(false);
        m_pipeline->SpawnPipeline(sensor->GetPipelineType());
        // create a ChFilterOptixRender and push to front of filter list
        auto opx_filter = chrono_types::make_shared<ChFilterOptixRender>();
        unsigned int id = static_cast<unsigned int>(m_assignedSensor.size() - 1);
        opx_filter->m_optix_pipeline = m_pipeline->GetPipeline(id);
        opx_filter->m_optix_params = md_params;
        opx_filter->m_optix_sbt = m_pipeline->GetSBT(id);
        opx_filter->m_raygen_record = m_pipeline->GetRayGenRecord(id);

        //m_params.integrator = sensor->GetIntegrator();

        // add a denoiser to the optix render filter if its a camera and global illumination is enabled
        if (auto cam = std::dynamic_pointer_cast<ChCameraSensor>(sensor)) { // TODO: Maybe set this up for TransientSensor as well?
            if (cam->GetUseDenoiser()) {
                std::cout << "Sensor: " << cam->GetName() << " requested global illumination\n";
                opx_filter->m_denoiser = chrono_types::make_shared<ChOptixDenoiser>(m_context);
                //opx_filter->m_denoiser = nullptr;
            }
            
        }

        // if transient cameram, populate the transient buffer
        if (auto trans_sensor = std::dynamic_pointer_cast<ChTransientSensor>(sensor)) {
              // allocate memory for transient samples
            size_t size = trans_sensor->GetWidth() * trans_sensor->GetHeight()* m_params.max_depth * sizeof(TransientSample); // Possible numerical error here
            std::cout << "Allocating " << size << " bytes for transient buffer\n";
            cudaMalloc(reinterpret_cast<void**>(&m_params.transient_buffer),size);
            cudaMemcpy(reinterpret_cast<void*>(md_params), &m_params, sizeof(ContextParameters), cudaMemcpyHostToDevice);

            m_params.window_size = trans_sensor->GetWindowSize();
            m_params.timegated_mode = trans_sensor->GetTimeGatedMode();
            m_params.target_dist = trans_sensor->GetTargetDist();
            m_params.nlos_hidden_geometry_sampling = trans_sensor->GetNLOSHiddenGeometrySampling();
            m_params.nlos_laser_sampling = trans_sensor->GetNLOSLaserSamples();
            m_params.filter_bounces = trans_sensor->GetFilterBounces();
            m_params.discard_direct_paths = trans_sensor->GetDiscardDirectPaths();
        }

        m_assignedRenderers.push_back(opx_filter);
        sensor->PushFilterFront(opx_filter);
        sensor->LockFilterList();

        std::shared_ptr<SensorBuffer> buffer;
        for (auto f : sensor->GetFilterList()) {
            f->Initialize(sensor, buffer);  // master thread should always be the one to initialize
        }
        // create the thread that will be in charge of this sensor (must be consistent thread for visualization reasons)
        m_renderThreads.emplace_back();
        id = static_cast<unsigned int>(m_renderThreads.size() - 1);
        m_renderThreads[id].done = true;
        m_renderThreads[id].start = false;
        m_renderThreads[id].terminate = false;
        m_renderThreads[id].thread =
            std::move(std::thread(&ChOptixEngine::RenderProcess, this, std::ref(m_renderThreads[id]), sensor));
    }
    if (!m_started) {
        Start();
    }
}

void ChOptixEngine::UpdateSensors(std::shared_ptr<ChScene> scene) {
    if (!m_params.root) {
        ConstructScene(scene);
    }
    std::vector<int> to_be_updated;
    std::vector<int> to_be_waited_on;

    // check if any of the sensors would be collecting data right now, if so, pack a tmp start keyframe
    for (int i = 0; i < m_assignedSensor.size(); i++) {
        auto sensor = m_assignedSensor[i];
        if (m_system->GetChTime() > sensor->GetNumLaunches() / sensor->GetUpdateRate() - 1e-7 &&
            !m_cameraStartFrames_set[i]) {
            // do this once per sensor because we don't know if they will be updated at the same time
            m_geometry->UpdateBodyTransformsStart((float)m_system->GetChTime(),
                                                  (float)m_system->GetChTime() + sensor->GetCollectionWindow());
            m_cameraStartFrames[i] = sensor->GetParent()->GetVisualModelFrame();
            m_cameraStartFrames_set[i] = true;
        }
    }

    // check which sensors need to be updated this step
    for (int i = 0; i < m_assignedSensor.size(); i++) {
        auto sensor = m_assignedSensor[i];
        if (m_system->GetChTime() >
            sensor->GetNumLaunches() / sensor->GetUpdateRate() + sensor->GetCollectionWindow() - 1e-7) {
            to_be_updated.push_back(i);
        }
    }
    

    if (to_be_updated.size() > 0) {
        {
            // lock the render queue to make sure previous rendering has completed
            std::unique_lock<std::mutex> lck(m_sceneThread.mutex);
            while (!m_sceneThread.done) {
                m_sceneThread.cv.wait(lck);
            }
            // m_sceneThread.cv.wait(lck);  // wait for the scene thread to tell us it is done
            cudaDeviceSynchronize();  // TODO: do we need to synchronize here?

            // update the scene for the optix context
            UpdateCameraTransforms(to_be_updated, scene);

            m_geometry->UpdateBodyTransformsEnd((float)m_system->GetChTime());
            
            // m_renderThreads
            UpdateSceneDescription(scene);
            UpdateDeformableMeshes();

            float t = (float)m_system->GetChTime();
            // push the sensors that need updating to the render queue
            for (auto i : to_be_updated) {
                m_renderQueue.push_back(i);
                m_assignedSensor[i]->IncrementNumLaunches();
                m_assignedRenderers[i]->m_time_stamp = t;
                m_renderThreads[i].done =
                    false;  // this render thread must not be done now given we have prepped some data for it
            }
        }
        // we only notify the worker thread when there is a sensor to launch and filters to process
        m_sceneThread.done = false;     // tell the thread it is not done
        m_sceneThread.start = true;     // tell the thread it should start
        m_sceneThread.cv.notify_all();  // tell the scene thread it should proceed
    }

    // wait for any sensors whose lag times would mean the data should be available before the next ones start rendering
    // bool data_complete = false;

    for (int i = 0; i < m_assignedSensor.size(); i++) {
        auto sensor = m_assignedSensor[i];
        if (m_system->GetChTime() > (sensor->GetNumLaunches() - 1) / sensor->GetUpdateRate() +
                                        sensor->GetCollectionWindow() + sensor->GetLag() - 1e-7) {
            // wait for the sensor thread i which will notify everyone when done

            // if (!m_mainLock.owns_lock())
            //     m_mainLock.lock();  // will wait for lock to come back from scene thread - TODO: make this check the
            // render threads instead
            // m_renderThreads.cv.wait()
            // if any sensors need to have their data returned, we must wait until optix is done rendering their data
            // TODO: allow waiting for the specific sensor rather than all of them
            // bool data_complete = false;
            // while (!data_complete) {
            //     std::lock_guard<std::mutex> lck(m_sceneThread.mutex);
            //     if (m_renderQueue.empty())
            //         data_complete = true;
            // }

            // see if this specific thread is done
            std::unique_lock<std::mutex> lck(m_renderThreads[i].mutex);
            while (!m_renderThreads[i].done) {
                m_renderThreads[i].cv.wait(lck);
            }
        }
    }
}  // namespace sensor

void ChOptixEngine::StopAllThreads() {
    // stop the scene building thread
    {
        // wait for last processing to be done
        std::unique_lock<std::mutex> lck(m_sceneThread.mutex);
        while (!m_sceneThread.done) {
            m_sceneThread.cv.wait(lck);
        }
        m_sceneThread.terminate = true;
        m_sceneThread.start = true;
        m_sceneThread.done = false;
    }
    m_sceneThread.cv.notify_all();

    // wait for it to finish the terminate proces
    {
        std::unique_lock<std::mutex> lck(m_sceneThread.mutex);
        while (!m_sceneThread.done) {
            m_sceneThread.cv.wait(lck);
        }
    }

    if (m_sceneThread.thread.joinable()) {
        m_sceneThread.thread.join();
    }

    m_started = false;

    // stop all the render threads
    for (int i = 0; i < m_renderThreads.size(); i++) {
        {
            // wait for previous processing to be done
            std::unique_lock<std::mutex> lck(m_renderThreads[i].mutex);
            while (!m_renderThreads[i].done) {
                m_renderThreads[i].cv.wait(lck);
            }
            m_renderThreads[i].terminate = true;
            m_renderThreads[i].start = true;
            m_renderThreads[i].done = false;
        }
        m_renderThreads[i].cv.notify_all();

        // wait for it to finish the terminate proces
        std::unique_lock<std::mutex> lck(m_renderThreads[i].mutex);
        while (!m_renderThreads[i].done) {
            m_renderThreads[i].cv.wait(lck);
        }
        if (m_renderThreads[i].thread.joinable()) {
            m_renderThreads[i].thread.join();
        }
    }

    // clear the assigned sensors as their corresponding threads have been stopped
    m_assignedSensor.clear();
    m_assignedRenderers.clear();
    m_renderThreads.clear();
}

void ChOptixEngine::Start() {
    if (!m_started) {
        m_sceneThread.start = false;
        m_sceneThread.terminate = false;
        m_sceneThread.done = true;  // thread is done to begin with (no work to complete)
        m_sceneThread.thread = std::move(std::thread(&ChOptixEngine::SceneProcess, this, std::ref(m_sceneThread)));
        m_started = true;
    }
}

void ChOptixEngine::RenderProcess(RenderThread& tself, std::shared_ptr<ChOptixSensor> sensor) {
    bool terminate = false;

    // keep the thread running until we submit a terminate job or equivalent
    while (!terminate) {
        std::unique_lock<std::mutex> tmp_lock(tself.mutex);
        while (!tself.start) {
            tself.cv.wait(tmp_lock);
        }
        tself.start = false;
        terminate = tself.terminate;

#if PROFILE
        auto start = std::chrono::high_resolution_clock::now();
#endif
        if (!terminate) {
            // run through the filter graph of our sensor
            for (auto f : sensor->GetFilterList()) {
                f->Apply();
            }
        }

        // wait for stream to synchronize
        cudaStreamSynchronize(sensor->GetCudaStream());
#if PROFILE
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        auto milli = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        std::cout << "Sensor = " << sensor->GetName() << ", Process time = " << milli << "ms" << std::endl;
#endif
        tself.done = true;
        tmp_lock.unlock();  // explicitely release the lock on this render thread
        tself.cv.notify_all();
    }
}

void ChOptixEngine::SceneProcess(RenderThread& tself) {
    // continually loop and perform two functions: add filters from sensor to job queue, empty the job queue

    bool terminate = false;

    // keep the thread running until we submit a terminate job or equivalent
    while (!terminate) {
        std::unique_lock<std::mutex> tmp_lock(tself.mutex);
        // wait for a notification from the main thread
        while (!tself.start) {
            tself.cv.wait(tmp_lock);
        }
        tself.start = false;  // reset start variable
        terminate = tself.terminate;

        if (!terminate) {
            // wait for the previous render threads before starting this rebuild

            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            m_geometry->RebuildRootStructure();
            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> wall_time =
                std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            // launch the render threads
            for (auto i : m_renderQueue) {
                m_renderThreads[i].done = false;
                m_renderThreads[i].start = true;
                m_renderThreads[i].cv.notify_all();  // notify render thread it should proceed

                // for (auto f : m_assignedSensor[i]->GetFilterList()) {
                //     f->Apply();
                // }
                // cudaStreamSynchronize(m_assignedSensor[i]->GetCudaStream());
            }

            // wait for each of the thread to be done before we say we are done
            for (auto i : m_renderQueue) {
                std::unique_lock<std::mutex> lck(m_renderThreads[i].mutex);
                while (!m_renderThreads[i].done) {
                    m_renderThreads[i].cv.wait(lck);
                }
            }
        }
        m_renderQueue.clear();  // empty list of sensor when everything is processed
        tself.done = true;
        tmp_lock.unlock();      // explicitely release the lock on the job queue
        tself.cv.notify_all();  // wake up anyone waiting for us
    }
}

void ChOptixEngine::boxVisualization(std::shared_ptr<ChBody> body,
                                     std::shared_ptr<ChVisualShapeBox> box_shape,
                                     ChFrame<> asset_frame) {
    ChVector3d size = box_shape->GetLengths();
    unsigned int mat_id;
    if (box_shape->GetNumMaterials() == 0) {
        mat_id = m_pipeline->GetBoxMaterial(body, box_shape);
    } else {
        mat_id = m_pipeline->GetBoxMaterial(body , box_shape, box_shape->GetMaterials());
    }
    m_geometry->AddBox(body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

void ChOptixEngine::sphereVisualization(std::shared_ptr<ChBody> body,
                                        std::shared_ptr<ChVisualShapeSphere> sphere_shape,
                                        ChFrame<> asset_frame) {
    ChVector3d size(sphere_shape->GetRadius());
    unsigned int mat_id;
    if (sphere_shape->GetNumMaterials() == 0) {
        mat_id = m_pipeline->GetSphereMaterial(body, sphere_shape);
    } else {
        mat_id = m_pipeline->GetSphereMaterial(body, sphere_shape, sphere_shape->GetMaterials());
    }
    m_geometry->AddSphere(body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

void ChOptixEngine::cylinderVisualization(std::shared_ptr<ChBody> body,
                                          std::shared_ptr<ChVisualShapeCylinder> cyl_shape,
                                          ChFrame<> asset_frame) {
    double radius = cyl_shape->GetRadius();
    double height = cyl_shape->GetHeight();

    ChVector3d size = {radius, radius, height};

    unsigned int mat_id;
    if (cyl_shape->GetNumMaterials() == 0) {
        mat_id = m_pipeline->GetCylinderMaterial();
    } else {
        mat_id = m_pipeline->GetCylinderMaterial(cyl_shape->GetMaterials());
    }
    m_geometry->AddCylinder(body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

void ChOptixEngine::rigidMeshVisualization(std::shared_ptr<ChBody> body,
                                           std::shared_ptr<ChVisualShapeTriangleMesh> mesh_shape,
                                           ChFrame<> asset_frame) {
    if (mesh_shape->IsWireframe()) {
        std::cerr << "WARNING: Chrono::Sensor does not support wireframe meshes. Defaulting back to solid mesh, please "
                     "check for visual issues.\n";
    }
    ChVector3d size = mesh_shape->GetScale();

    //printf("Rigid Mesh: %s!\n", body->GetName());
    unsigned int mat_id;
    CUdeviceptr d_vertex_buffer;  // handle will go to m_geometry
    CUdeviceptr d_index_buffer;   // handle will go to m_geometry

    mat_id = m_pipeline->GetRigidMeshMaterial(d_vertex_buffer, d_index_buffer, mesh_shape, mesh_shape->GetMaterials());
    m_geometry->AddRigidMesh(d_vertex_buffer, d_index_buffer, mesh_shape, body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

void ChOptixEngine::deformableMeshVisualization(std::shared_ptr<ChBody> body,
                                                std::shared_ptr<ChVisualShapeTriangleMesh> mesh_shape,
                                                ChFrame<> asset_frame) {
    if (mesh_shape->IsWireframe()) {
        std::cerr << "WARNING: Chrono::Sensor does not support wireframe meshes. Defaulting back to solid mesh, please "
                     "check for visual issues.\n";
    }
    ChVector3d size = mesh_shape->GetScale();

    unsigned int mat_id;
    CUdeviceptr d_vertex_buffer;  // handle will go to m_geometry
    CUdeviceptr d_index_buffer;   // handle will go to m_geometry
    mat_id =
        m_pipeline->GetDeformableMeshMaterial(d_vertex_buffer, d_index_buffer, mesh_shape, mesh_shape->GetMaterials());
    m_geometry->AddDeformableMesh(d_vertex_buffer, d_index_buffer, mesh_shape, body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

#ifdef USE_SENSOR_NVDB
void ChOptixEngine::nvdbVisualization(std::shared_ptr<ChBody> body,
                                      std::shared_ptr<ChNVDBShape> box_shape,
                                      ChFrame<> asset_frame) {
    ChVector3d size = box_shape->GetBoxGeometry().GetLengths();

    unsigned int mat_id;
    if (box_shape->GetNumMaterials() == 0) {
        mat_id = m_pipeline->GetNVDBMaterial();
    } else {
        mat_id = m_pipeline->GetNVDBMaterial(box_shape->GetMaterials());
    }
    m_geometry->AddNVDBVolume(body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}
#endif  // USE_SENSOR_NVDB

void ChOptixEngine::ConstructScene(std::shared_ptr<ChScene> scene) {
    // need to lock before touching any optix stuff
    // std::lock_guard<std::mutex> lck(
    //     m_sceneThread.mutex);  /// here we should not wait for a notify, it is good enough to get a lock
    std::unique_lock<std::mutex> lck(m_sceneThread.mutex);
    while (!m_sceneThread.done) {
        m_sceneThread.cv.wait(lck);
    }
    cudaDeviceSynchronize();
    // wipeout all of old scene
    m_geometry->Cleanup();         // remove all geometry
    m_pipeline->CleanMaterials();  // remove all programs and materials

    // iterate through all bodies in Chrono and add a subnode for each body in Chrono
    for (auto body : m_system->GetBodies()) {
        if (body->GetVisualModel()) {
            for (auto& shape_instance : body->GetVisualModel()->GetShapeInstances()) {
                const auto& shape = shape_instance.first;
                const auto& shape_frame = shape_instance.second;
                // check if the asset is a ChVisualShape

                // if (std::shared_ptr<ChVisualShape> visual_asset = std::dynamic_pointer_cast<ChVisualShape>(asset)) {

                // collect relative position and orientation of the asset
                // ChVector3d asset_pos = visual_asset->Pos;
                // ChMatrix33<double> asset_rot_mat = visual_asset->Rot;

                // const ChFrame<float> asset_frame = ChFrame<float>(asset_pos,asset_rot_mat);

                if (!shape->IsVisible()) {
                    // std::cout << "Ignoring an asset that is set to invisible\n";
                    printf("Body %s is invisible\n", body->GetName().c_str());
                } else if (auto box_shape = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
                    boxVisualization(body, box_shape, shape_frame);
                } 
                #ifdef USE_SENSOR_NVDB
                else if (std::shared_ptr<ChNVDBShape> nvdb_shape = std::dynamic_pointer_cast<ChNVDBShape>(shape)) {
                    nvdbVisualization(body, nvdb_shape, shape_frame);
                    printf("Added NVDB Shape!");
                }
                #endif
                else if (auto sphere_shape = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
                    sphereVisualization(body, sphere_shape, shape_frame);

                } else if (auto cylinder_shape = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
                    cylinderVisualization(body, cylinder_shape, shape_frame);

                } else if (auto trimesh_shape = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape)) {
                    if (!trimesh_shape->IsMutable()) {
                        //printf("Trimesh Shape: %s, Pos: %f,%f,%f\n", body->GetName().c_str(),body->GetPos().x(), body->GetPos().y(), body->GetPos().z());
                        rigidMeshVisualization(body, trimesh_shape, shape_frame);

                        // added_asset_for_body = true;
                    } else {
                        deformableMeshVisualization(body, trimesh_shape, shape_frame);
                    }

                } else if (auto ellipsoid_shape = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(shape)) {
                } else if (auto cone_shape = std::dynamic_pointer_cast<ChVisualShapeCone>(shape)) {
                } else if (auto rbox_shape = std::dynamic_pointer_cast<ChVisualShapeRoundedBox>(shape)) {
                } else if (auto capsule_shape = std::dynamic_pointer_cast<ChVisualShapeCapsule>(shape)) {
                } else if (auto path_shape = std::dynamic_pointer_cast<ChVisualShapePath>(shape)) {
                } else if (auto line_shape = std::dynamic_pointer_cast<ChVisualShapeLine>(shape)) {
                }

                // TODO: Add NVDB Vis condition
                // }
                // }
            }
        }
    }

    // // Assumption made here that other physics items don't have a transform -> not always true!!!
    for (auto item : m_system->GetOtherPhysicsItems()) {
        if (item->GetVisualModel()) {
            for (auto& shape_instance : item->GetVisualModel()->GetShapeInstances()) {
                const auto& shape = shape_instance.first;
                const auto& shape_frame = shape_instance.second;

                auto dummy_body = chrono_types::make_shared<ChBody>();

                if (!shape->IsVisible()) {
                    // std::cout << "Ignoring an asset that is set to invisible\n";
                } else if (auto box_shape = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
                    boxVisualization(dummy_body, box_shape, shape_frame);

                } else if (auto sphere_shape = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
                    sphereVisualization(dummy_body, sphere_shape, shape_frame);

                } else if (auto cylinder_shape = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
                    cylinderVisualization(dummy_body, cylinder_shape, shape_frame);

                } else if (auto trimesh_shape = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape)) {
                    if (!trimesh_shape->IsMutable()) {
                        rigidMeshVisualization(dummy_body, trimesh_shape, shape_frame);
                    } else {
                        deformableMeshVisualization(dummy_body, trimesh_shape, shape_frame);
                    }

                } else if (auto ellipsoid_shape = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(shape)) {
                } else if (auto cone_shape = std::dynamic_pointer_cast<ChVisualShapeCone>(shape)) {
                } else if (auto rbox_shape = std::dynamic_pointer_cast<ChVisualShapeRoundedBox>(shape)) {
                } else if (auto capsule_shape = std::dynamic_pointer_cast<ChVisualShapeCapsule>(shape)) {
                } else if (auto path_shape = std::dynamic_pointer_cast<ChVisualShapePath>(shape)) {
                } else if (auto line_shape = std::dynamic_pointer_cast<ChVisualShapeLine>(shape)) {
                }
            }
        }
    }

    // Add sprites to the scene
    for (auto sprite : scene->GetSprites()) {
        if (sprite->GetVisualModel()) {
            for (auto& shape_instance : sprite->GetVisualModel()->GetShapeInstances()) {
                const auto& shape = shape_instance.first;
                const auto& shape_frame = shape_instance.second;
                // check if the asset is a ChVisualShape

                // if (std::shared_ptr<ChVisualShape> visual_asset = std::dynamic_pointer_cast<ChVisualShape>(asset)) {

                // collect relative position and orientation of the asset
                // ChVector3d asset_pos = visual_asset->Pos;
                // ChMatrix33<double> asset_rot_mat = visual_asset->Rot;

                // const ChFrame<float> asset_frame = ChFrame<float>(asset_pos,asset_rot_mat);

                if (!shape->IsVisible()) {
                    // std::cout << "Ignoring an asset that is set to invisible\n";
                    printf("Body %s is invisible\n", sprite->GetName().c_str());
                } else if (auto box_shape = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
                    boxVisualization(sprite, box_shape, shape_frame);
                } 
                #ifdef USE_SENSOR_NVDB
                else if (std::shared_ptr<ChNVDBShape> nvdb_shape = std::dynamic_pointer_cast<ChNVDBShape>(shape)) {
                    nvdbVisualization(sprite, nvdb_shape, shape_frame);
                    printf("Added NVDB Shape!");
                }
                #endif
                else if (auto sphere_shape = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
                    sphereVisualization(sprite, sphere_shape, shape_frame);

                } else if (auto cylinder_shape = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
                    cylinderVisualization(sprite, cylinder_shape, shape_frame);

                } else if (auto trimesh_shape = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape)) {
                    if (!trimesh_shape->IsMutable()) {
                        // printf("Trimesh Shape: %s, Pos: %f,%f,%f\n", sprite->GetName().c_str(), sprite->GetPos().x(),
                        //        sprite->GetPos().y(), sprite->GetPos().z());
                        rigidMeshVisualization(sprite, trimesh_shape, shape_frame);

                        // added_asset_for_body = true;
                    } else {
                        deformableMeshVisualization(sprite, trimesh_shape, shape_frame);
                    }

                } else if (auto ellipsoid_shape = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(shape)) {
                } else if (auto cone_shape = std::dynamic_pointer_cast<ChVisualShapeCone>(shape)) {
                } else if (auto rbox_shape = std::dynamic_pointer_cast<ChVisualShapeRoundedBox>(shape)) {
                } else if (auto capsule_shape = std::dynamic_pointer_cast<ChVisualShapeCapsule>(shape)) {
                } else if (auto path_shape = std::dynamic_pointer_cast<ChVisualShapePath>(shape)) {
                } else if (auto line_shape = std::dynamic_pointer_cast<ChVisualShapeLine>(shape)) {
                }

                // TODO: Add NVDB Vis condition
                // }
                // }
            }
        }
    }
    
    // Process shapes and surface area distribution
    m_params.total_object_area = m_pipeline->GetTotalSurfaceArea();
    m_params.num_hidden_geometry = m_pipeline->GetNumHidddenShapes();

    m_params.root = m_geometry->CreateRootStructure();
    m_pipeline->UpdateAllSBTs();
    m_pipeline->UpdateAllPipelines();
    m_params.mesh_pool = reinterpret_cast<MeshParameters*>(m_pipeline->GetMeshPool());
    m_params.material_pool = reinterpret_cast<MaterialParameters*>(m_pipeline->GetMaterialPool());
    m_params.shape_info = reinterpret_cast<Shape*>(m_pipeline->GetShapeList());
    m_params.sphere_data = reinterpret_cast<SphereParameters*>(m_pipeline->GetSphereList());
    m_params.box_data = reinterpret_cast<BoxParameters*>(m_pipeline->GetBoxList());
    m_params.surface_area_buffer = reinterpret_cast<float*>(m_pipeline->GetSurfaceAreaPMF());
    m_params.surface_area_cdf_buffer = reinterpret_cast<float*>(m_pipeline->GetSurfaceAreaCDF());

    cudaMemcpy(reinterpret_cast<void*>(md_params), &m_params, sizeof(ContextParameters), cudaMemcpyHostToDevice);
}

void ChOptixEngine::UpdateCameraTransforms(std::vector<int>& to_be_updated, std::shared_ptr<ChScene> scene) {
    // go through the sensors to be updated and see if we need to move the scene origin
    for (unsigned int i = 0; i < to_be_updated.size(); i++) {
        int id = to_be_updated[i];
        ChFrame<double> f_offset = m_assignedSensor[id]->GetOffsetPose();
        ChFrame<double> f_body_0 = m_cameraStartFrames[id];
        ChFrame<double> global_loc_0 = f_body_0 * f_offset;
        scene->UpdateOriginOffset(global_loc_0.GetPos());
    }

    // go through all the sensors to be updated
    for (unsigned int i = 0; i < to_be_updated.size(); i++) {
        int id = to_be_updated[i];
        auto sensor = m_assignedSensor[id];

        // update radar velocity
        if (auto radar = std::dynamic_pointer_cast<ChRadarSensor>(sensor)) {
            ChVector3f origin(0, 0, 0);
            auto r = radar->GetOffsetPose().GetPos() - origin;
            auto ang_vel = radar->GetAngularVelocity() % r;
            auto vel_abs =
                radar->GetOffsetPose().TransformDirectionLocalToParent(ang_vel) + radar->GetTranslationalVelocity();
            m_assignedRenderers[id]->m_raygen_record->data.specific.radar.velocity.x = vel_abs.x();
            m_assignedRenderers[id]->m_raygen_record->data.specific.radar.velocity.y = vel_abs.y();
            m_assignedRenderers[id]->m_raygen_record->data.specific.radar.velocity.z = vel_abs.z();
            m_pipeline->UpdateObjectVelocity();
        }

        // Clear transient sensor frame buffer
        if (auto transientSensor = std::dynamic_pointer_cast<ChTransientSensor>(sensor)) {
            int w = transientSensor->GetWidth();
            int h = transientSensor->GetHeight();
             initializeBuffer(m_assignedRenderers[id]->m_raygen_record->data.specific.transientCamera.frame_buffer,
                                w * transientSensor->GetNumBins(), h);
            //if (transientSensor->GetUseGI()) {  
            //    /*initializeBuffer(m_assignedRenderers[id]->m_raygen_record->data.specific.transientCamera.frame_buffer,
            //                    m_assignedRenderers[id]->m_raygen_record->data.specific.transientCamera.albedo_buffer,
            //                    m_assignedRenderers[id]->m_raygen_record->data.specific.transientCamera.normal_buffer,
            //                     w, h);*/ // GI is enabled for transient camera but not the denoiser

            // } else {
            //    initializeBuffer(m_assignedRenderers[id]->m_raygen_record->data.specific.transientCamera.frame_buffer,
            //                     w, h);
            // }
        }

 
           
        ChFrame<double> f_offset = sensor->GetOffsetPose();
        ChFrame<double> f_body_0 = m_cameraStartFrames[i];
        m_cameraStartFrames_set[i] = false;  // reset this camera frame so that we know it should be packed again
        ChFrame<double> f_body_1 = sensor->GetParent()->GetVisualModelFrame();
        ChFrame<double> global_loc_0 = f_body_0 * f_offset;
        ChFrame<double> global_loc_1 = f_body_1 * f_offset;

        ChVector3f pos_0 = global_loc_0.GetPos() - scene->GetOriginOffset();
        ChVector3f pos_1 = global_loc_1.GetPos() - scene->GetOriginOffset();

        m_assignedRenderers[id]->m_raygen_record->data.t0 =
            (float)(m_system->GetChTime() - sensor->GetCollectionWindow());
        m_assignedRenderers[id]->m_raygen_record->data.t1 = (float)(m_system->GetChTime());
        m_assignedRenderers[id]->m_raygen_record->data.pos0 = make_float3(pos_0.x(), pos_0.y(), pos_0.z());
        m_assignedRenderers[id]->m_raygen_record->data.rot0 =
            make_float4((float)global_loc_0.GetRot().e0(), (float)global_loc_0.GetRot().e1(),
                        (float)global_loc_0.GetRot().e2(), (float)global_loc_0.GetRot().e3());
        m_assignedRenderers[id]->m_raygen_record->data.pos1 = make_float3(pos_1.x(), pos_1.y(), pos_1.z());
        m_assignedRenderers[id]->m_raygen_record->data.rot1 =
            make_float4((float)global_loc_1.GetRot().e0(), (float)global_loc_1.GetRot().e1(),
                        (float)global_loc_1.GetRot().e2(), (float)global_loc_1.GetRot().e3());
        m_assignedRenderers[id]->m_time_stamp = (float)m_system->GetChTime();
    }
}

void ChOptixEngine::UpdateDeformableMeshes() {
    // update the mesh in the pipeline
    m_pipeline->UpdateDeformableMeshes();
    // update the meshes in the geometric scene
    m_geometry->UpdateDeformableMeshes();
}

void ChOptixEngine::UpdateSceneDescription(std::shared_ptr<ChScene> scene) {

    if (scene->GetBackgroundChanged()) {
        m_pipeline->UpdateBackground(scene->GetBackground());

        m_params.scene_epsilon = scene->GetSceneEpsilon();
        m_params.fog_color = {scene->GetFogColor().x(), scene->GetFogColor().y(), scene->GetFogColor().z()};
        m_params.fog_scattering = scene->GetFogScattering();

        cudaMemcpy(reinterpret_cast<void*>(md_params), &m_params, sizeof(ContextParameters), cudaMemcpyHostToDevice);
        scene->ResetBackgroundChanged();
    }

    std::vector<Light> l = scene->GetLights();
    // Handle moving lights
    bool light_moved = false;
    for (int i = 0; i < l.size(); i++) {
        Light light = l[i];
        if (light.type == LightType::SPOT_LIGHT && light.parent_id >= 0) {
            std::cout << "Moving Spot Light" << std::endl;
            scene->UpdateLight(i);
        }
    }

    if (true && scene->GetLightsChanged() || scene->GetOriginChanged()) {

        // Handling changes to all lights
        if (l.size() != m_params.num_lights) {  // need new memory in this case
            if (m_params.lights)
                CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(m_params.lights)));

            cudaMalloc(reinterpret_cast<void**>(&m_params.lights), l.size() * sizeof(Light));
        }

        for (unsigned int i = 0; i < l.size(); i++) {
            l[i].pos = make_float3(l[i].pos.x - scene->GetOriginOffset().x(), l[i].pos.y - scene->GetOriginOffset().y(),
                                   l[i].pos.z - scene->GetOriginOffset().z());
        }

        cudaMemcpy(reinterpret_cast<void*>(m_params.lights), l.data(), l.size() * sizeof(Light), cudaMemcpyHostToDevice);
        m_params.num_lights = static_cast<int>(l.size());
      
        
        // Handling changes in origin

        m_params.ambient_light_color = {scene->GetAmbientLight().x(), scene->GetAmbientLight().y(),
                                        scene->GetAmbientLight().z()};

        cudaMemcpy(reinterpret_cast<void*>(md_params), &m_params, sizeof(ContextParameters), cudaMemcpyHostToDevice);

        m_geometry->SetOriginOffset(scene->GetOriginOffset());
        scene->ResetLightsChanged();
        //scene->ResetAreaLightsChanged();
        scene->ResetOriginChanged();
    }



    #ifdef USE_SENSOR_NVDB
    if (float* d_pts = scene->GetFSIParticles()) {
        int n = scene->GetNumFSIParticles();
        
        printf("Creatinng NanoVDB Handle...\n");
        //using buildType = nanovdb::Point;
        //nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> volHandle;
        //nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> normalHandle;
        //nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> densityHandle;
        //createNanoVDBGridHandle(d_pts, n, scene->GetVoxelSize(), scene->GetWindowSize(), volHandle, normalHandle,densityHandle);
        ////nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> handle = addVDBVolume(nullptr);
        //nanovdb::NanoGrid<buildType>* grid = volHandle.deviceGrid<buildType>();
        //nanovdb::NanoGrid<nanovdb::Vec3f>* normal_grid = normalHandle.deviceGrid<nanovdb::Vec3f>();
        //nanovdb::NanoGrid<float>* density_grid = densityHandle.deviceGrid<float>();

        //if (m_params.handle_ptr)
        //    cudaFree(m_params.handle_ptr);
        //cudaMalloc((void**)&m_params.handle_ptr, volHandle.gridSize());
        //cudaMemcpy((void*)m_params.handle_ptr, grid, volHandle.gridSize(), cudaMemcpyDeviceToDevice);

        //if (m_params.normal_handle_ptr)
        //    cudaFree(m_params.normal_handle_ptr);
        //cudaMalloc((void**)&m_params.normal_handle_ptr, normalHandle.gridSize());
        //cudaMemcpy((void*)m_params.normal_handle_ptr, normal_grid, normalHandle.gridSize(), cudaMemcpyDeviceToDevice);


        // Make dust grid
        if (m_system->GetChTime() > 1.f) {
            nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> dustHandle;
            createDustGrid(dustHandle, d_pts, n, scene->GetThresholdVelocity(), &m_dust_particles, m_num_dust_particles,
                           m_system->GetChTime(), m_frame, scene->GetZThreshold());
            if (m_num_dust_particles > 0) {
                 nanovdb::NanoGrid<float>* dust_grid = dustHandle.deviceGrid<float>();
                if (m_params.density_grid_ptr)
                    cudaFree(m_params.density_grid_ptr);
                cudaMalloc((void**)&m_params.density_grid_ptr, dustHandle.gridSize());
                cudaMemcpy((void*)m_params.density_grid_ptr, dust_grid, dustHandle.gridSize(), cudaMemcpyDeviceToDevice);
            } else {
                nanovdb::build::Grid<float> density_grid(0.f);
                density_grid.setTransform(0.001);
                density_grid.setName("density0");
                dustHandle = nanovdb::createNanoGrid<nanovdb::build::Grid<float>, float, nanovdb::CudaDeviceBuffer>(density_grid);
                dustHandle.deviceUpload();
                nanovdb::NanoGrid<float>* dust_grid = dustHandle.deviceGrid<float>();
                if (m_params.density_grid_ptr)
                    cudaFree(m_params.density_grid_ptr);
                cudaMalloc((void**)&m_params.density_grid_ptr, dustHandle.gridSize());
                cudaMemcpy((void*)m_params.density_grid_ptr, dust_grid, dustHandle.gridSize(), cudaMemcpyDeviceToDevice);
            }
        } else { // TODO: Ugly code! Refactor!
            nanovdb::build::Grid<float> density_grid(0.f);
            density_grid.setTransform(0.001);
            density_grid.setName("density0");
            nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> dustHandle = nanovdb::createNanoGrid<nanovdb::build::Grid<float>, float, nanovdb::CudaDeviceBuffer>(density_grid);
            dustHandle.deviceUpload();
            nanovdb::NanoGrid<float>* dust_grid = dustHandle.deviceGrid<float>();
            if (m_params.density_grid_ptr)
                cudaFree(m_params.density_grid_ptr);
            cudaMalloc((void**)&m_params.density_grid_ptr, dustHandle.gridSize());
            cudaMemcpy((void*)m_params.density_grid_ptr, dust_grid, dustHandle.gridSize(), cudaMemcpyDeviceToDevice);
            
        }
     
       
        /*DustParticle* h_dust = new DustParticle[m_num_dust_particles];
        DustParticle* d_dust = (DustParticle*)m_dust_particles;
        cudaMemcpy(h_dust, d_dust, m_num_dust_particles * sizeof(DustParticle), cudaMemcpyDeviceToHost);*
        for (int i = 0; i < m_num_dust_particles; i++) {
            float3 pos = h_dust[i].pos;
            printf("i: %d | p: (%f,%f,%f) | Active: %d\n", i, pos.x,pos.y,pos.z, h_dust[i].isActive);
        }*/
        cudaMemcpy(reinterpret_cast<void*>(md_params), &m_params, sizeof(ContextParameters), cudaMemcpyHostToDevice);
    }

    if (std::shared_ptr<openvdb::FloatGrid> grid = scene->GetVDBGrid()) {
        printf("Adding VDB Volume Grid to Scene\n");
        nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> handle = addVDBVolume(grid);
        //nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> handle = createNanoVDBGridHandle(nullptr, 10);
        nanovdb::NanoGrid<float>* nanoGrid = handle.deviceGrid<float>();
        handle.deviceDownload();
        auto* nanoGrid_h = handle.grid<float>();

        cudaMalloc((void**)&m_params.density_grid_ptr, handle.gridSize());
        cudaMemcpy((void*)m_params.density_grid_ptr, nanoGrid, handle.gridSize(), cudaMemcpyDeviceToDevice);
      
        nanovdb::DefaultReadAccessor<float> acc = nanoGrid_h->tree().getAccessor();
        nanovdb::CoordBBox bbox = acc.root().bbox();
        
    printf("############### NanoVDB GRID INFORMATION ################\n");
        printf("Grid Size: %d\n", nanoGrid_h->gridSize());
        printf("Grid Class: %s\n", nanovdb::toStr(handle.gridMetaData()->gridClass()));
        printf("Grid Type: %s\n", nanovdb::toStr(handle.gridType(0)));
        printf("Upper Internal Nodes: %d\n", nanoGrid_h->tree().nodeCount(2));
        printf("Lower Internal Nodes: %d\n", nanoGrid_h->tree().nodeCount(1));
        printf("Leaf Nodes: %d\n", nanoGrid_h->tree().nodeCount(0));
        printf("Voxel Size: %f\n", float(nanoGrid_h->voxelSize()[0]));
        printf("Active Voxels: %d\n", nanoGrid_h->activeVoxelCount());
        printf("World BBox Dims: %f %f %f\n", nanoGrid_h->worldBBox().dim()[0], nanoGrid_h->worldBBox().dim()[1],
               nanoGrid_h->worldBBox().dim()[2]);
        printf("World Bbox Max: %f %f %f | Min: %f %f %f\n", nanoGrid_h->worldBBox().max()[0],
               nanoGrid_h->worldBBox().max()[1], nanoGrid_h->worldBBox().max()[2], nanoGrid_h->worldBBox().min()[0],
               nanoGrid_h->worldBBox().min()[1], nanoGrid_h->worldBBox().min()[2]);
        printf("BBox: min:(%d,%d,%d)| max:(%d,%d,%d)\n", bbox.min()[0], bbox.min()[1], bbox.min()[2], bbox.max()[0],
               bbox.max()[1], bbox.max()[2]);
        printf("############### END #############\n");

        cudaMemcpy(reinterpret_cast<void*>(md_params), &m_params, sizeof(ContextParameters), cudaMemcpyHostToDevice);
    }
    #endif
    }

}  // namespace sensor
}  // namespace chrono
