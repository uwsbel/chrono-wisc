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
// Authors: Asher Elmquist
// =============================================================================
//
// camera ray launch kernels
//
// =============================================================================

#ifndef CAMERA_RAYGEN_CU
#define CAMERA_RAYGEN_CU

#include "chrono_sensor/optix/shaders/device_utils.cuh"
#include "chrono_sensor/optix/shaders/camera_utils.cuh"

/// Camera ray generation program with using a lens distortion model
extern "C" __global__ void __raygen__camera() {
    
    const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
    const CameraParameters& camera = raygen->specific.camera;

    const uint3 px_2D_idx = optixGetLaunchIndex();
    const uint3 img_size = optixGetLaunchDimensions();
    unsigned int num_spp = camera.super_sample_factor * camera.super_sample_factor; // number of samples per pixel (spp)
    float recip_num_spp = 1 / static_cast<float>(num_spp);
    const unsigned int pixel_idx = img_size.x * px_2D_idx.y + px_2D_idx.x;

    float3 color_result = make_float3(0.f);
    float3 albedo_result = make_float3(0.f);
    float opacity_result = 0.f;
    float3 prd_normal = make_float3(0.f);
    float gamma = camera.gamma;
    camera.frame_buffer[pixel_idx] = make_half4(0.f, 0.f, 0.f, 0.f);

    if (camera.use_denoiser) {
        camera.albedo_buffer[pixel_idx] = make_half4(0.f, 0.f, 0.f, 0.f);
        camera.normal_buffer[pixel_idx] = make_half4(0.f, 0.f, 0.f, 0.f);
    }   
    
    curandState_t rng = camera.rng_buffer[pixel_idx];
    float3 ray_origin, ray_direction;
    float3 cam_forward, cam_left, cam_up;
    for (unsigned int sample_idx = 0; sample_idx < num_spp; sample_idx++) {
    
        //// Get camera's pose (origin of the ray to be launched) ////
        
        // Add motion-blur effect
        float t_frac = (camera.rng_buffer) ? curand_uniform(&rng) : 0.f;
        // float t_frac = static_cast<float>((sample_idx + 1)) * recip_num_spp;  // uniformly-spaced midpoint samples over span to compose motion-blur
        
        // Last jitter must be at the center of the pixel
        float2 jitter = (sample_idx == num_spp - 1) ? make_float2(0.5f, 0.5f) : make_float2(curand_uniform(&rng), curand_uniform(&rng));
        
        get_cam_ray_direction(ray_origin, ray_direction, cam_forward, cam_left, cam_up, raygen, camera.lens_model, camera.lens_parameters,
                              camera.hFOV, px_2D_idx, img_size, t_frac, jitter);

        // Create per-ray data for camera ray
        PerRayData_camera prd = DefaultCameraPRD();
        prd.integrator = camera.integrator;
        prd.use_gi = camera.use_gi;
        prd.use_fog = camera.use_fog;
        prd.rng = camera.rng_buffer[pixel_idx];
        unsigned int opt1;
        unsigned int opt2;
        pointer_as_ints(&prd, opt1, opt2);
        unsigned int raytype = (unsigned int)RayType::CAMERA_RAY_TYPE;
        const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent during the frame
        // printf("CameraRayGen: orig: (%f,%f,%f), dir:(%f,%f,%f)\n", ray_origin.x,ray_origin.y,ray_origin.z, ray_direction.x, ray_direction.y, ray_direction.z);
        optixTrace(
            params.root,            // The scene traversable handle (OptixTraversableHandle); basically the top-level acceleration structure (TLAS).
            ray_origin,             // origin of the traced ray
            ray_direction,          // direction of the traced ray
            params.scene_epsilon,   // minimum intersection distance to avoid self-intersection (“shadow acne”)
            1e16f,                  // A very large max distance (effectively “infinite” for the scene scale)
            t_traverse,             // time value for motion blur (0.0 to 1.0)
            OptixVisibilityMask(1), // Only intersects geometry whose instance mask matches 1
            OPTIX_RAY_FLAG_NONE,    // No special flags (e.g., no disable-anyhit, no terminate-on-first-hit, etc.)
            0,                      // SBT offset (used when you have multiple SBT records for the same ray type). It selects the first “ray type slot”
            1,                      // SBT stride (used when you have multiple SBT records for the same ray type). It usually means “one ray type stride”
            0,                      // missSBTIndex. It selects the first miss program
            opt1,                   // Final payloads; the per-ray data pointer (first 32 bits); optixGetPayload_0() = opt1
            opt2,                   // Final payloads; the per-ray data pointer (second 32 bits); optixGetPayload_1() = opt2
            raytype                 // The ray type index (used when you have multiple ray types, e.g., radiance rays, shadow rays, etc.)
        );
        
        // Aggregate results from this sample
        color_result += prd.color;
        albedo_result += prd.albedo;
        opacity_result += prd.transparency;

        if ((sample_idx == num_spp - 1) && camera.use_denoiser) {
            prd_normal = prd.normal;
        }
    }

    // Average results over all samples of each pixel
    color_result = color_result * recip_num_spp;
    albedo_result = albedo_result * recip_num_spp;
    opacity_result = opacity_result * recip_num_spp;
    if (camera.use_denoiser) {
        camera.albedo_buffer[pixel_idx] = make_half4(albedo_result.x, albedo_result.y, albedo_result.z, 0.f);
        
        // Transform to screen coordinates (x: right, y: up, z: backward)
        camera.normal_buffer[pixel_idx] = make_half4(-Dot(cam_left, prd_normal), Dot(cam_up, prd_normal), -Dot(cam_forward, prd_normal), 0.f);
    }

    bool is_transparent = (opacity_result < 1e-6) ? true : false;

    // Gamma correction
    camera.frame_buffer[pixel_idx] = make_half4(
        pow(color_result.x, 1.0f / gamma), pow(color_result.y, 1.0f / gamma), pow(color_result.z, 1.0f / gamma), opacity_result
    );
}

#endif // CAMERA_RAYGEN_CU
