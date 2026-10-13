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
// depth camera ray launch kernels
//
// =============================================================================

#ifndef DEPTH_CAM_RAYGEN_CU
#define DEPTH_CAM_RAYGEN_CU

#include "chrono_sensor/optix/shaders/device_utils.cuh"

// Default of depth camera per ray data (PRD)
__device__ __inline__ PerRayData_depthCamera DefaultDepthCameraPRD(float maxDepth) {
    PerRayData_depthCamera prd = {};
    prd.depth = 0.f;
    prd.max_depth = maxDepth;
    return prd;
};

// Ray generation program for depth camera
extern "C" __global__ void __raygen__depth_camera() {
    const RaygenParameters* raygen = (RaygenParameters*) optixGetSbtDataPointer();
    const DepthCameraParameters& depth_cam = raygen->specific.depthCamera;

    const uint3 px_2D_idx = optixGetLaunchIndex();
    const uint3 img_size = optixGetLaunchDimensions();
    const unsigned int pixel_idx = img_size.x * px_2D_idx.y + px_2D_idx.x;
    float t_frac = 0.f;
    float2 jitter = make_float2(0.5f, 0.5f);
    curandState_t rng = depth_cam.rng_buffer[pixel_idx];
    float3 ray_origin, ray_direction;
    float3 cam_forward, cam_left, cam_up;
    t_frac = curand_uniform(&rng);              // 0-1 between start and end time of the camera (chosen here)
    const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent

    get_cam_ray_direction(ray_origin, ray_direction, cam_forward, cam_left, cam_up, raygen, depth_cam.lens_model,
                          depth_cam.lens_parameters, depth_cam.hFOV, px_2D_idx, img_size, t_frac, jitter);

    
    // Create per-ray data for depth camera ray
    PerRayData_depthCamera prd = DefaultDepthCameraPRD(depth_cam.max_depth);
    unsigned int opt1;
    unsigned int opt2;
    pointer_as_ints(&prd, opt1, opt2);
    unsigned int raytype = (unsigned int)RayType::DEPTH_RAY_TYPE;
    optixTrace(
        params.root, // The scene traversable handle (OptixTraversableHandle)
        ray_origin, // Ray origin
        ray_direction, // Ray direction
        params.scene_epsilon, // Min intersection distance
        1e16f, // Max intersection distance
        t_traverse, // Ray time
        OptixVisibilityMask(1), // Specify always visible
        OPTIX_RAY_FLAG_NONE, // Specify no special flags
        0, // SBT offset
        1, // SBT stride
        0, // missSBTIndex
        opt1, // PRD pointer as two 32-bit values
        opt2, // PRD pointer as two 32-bit values
        raytype // Ray type
    );

    depth_cam.frame_buffer[pixel_idx] = prd.depth;
}

#endif // DEPTH_CAM_RAYGEN_CU