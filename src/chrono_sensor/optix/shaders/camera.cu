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

#include "chrono_sensor/optix/shaders/device_utils.h"


__device__ __inline__ float radial_function(const float& rd2, const LensParams& params){
    // Drap, P., & Lefevre, J. (2016). 
    // An Exact Formula for Calculating Inverse Radial Lens Distortions. 
    // Sensors (Basel, Switzerland), 16(6), 807. https://doi.org/10.3390/s16060807
    double rd4 = rd2 * rd2;
    double rd6 = rd4 * rd2;
    double rd8 = rd4 * rd4;
    double rd10 = rd6 * rd4;
    double rd12 = rd6 * rd6;
    double rd14 = rd8 * rd6;
    double rd16 = rd8 * rd8;
    double rd18 = rd10 * rd8;

    float ru = (float)(1.0 + params.a0 * rd2 + 
        params.a1 * rd4 +
        params.a2 * rd6 + 
        params.a3 * rd8 +
        params.a4 * rd10 +
        params.a5 * rd12 +
        params.a6 * rd14 +
        params.a7 * rd16 +
        params.a8 * rd18);
    return ru;
}

/// Camera ray generation program using an FOV lens model
extern "C" __global__ void __raygen__camera() {
    const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
    const CameraParameters& camera = raygen->specific.camera;

    const uint3 idx = optixGetLaunchIndex();
    const uint3 screen = optixGetLaunchDimensions();
    const unsigned int image_index = screen.x * idx.y + idx.x;

    float2 d =
        (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f - make_float2(1.f);
    d.y *= (float)(screen.y) / (float)(screen.x);  // correct for the aspect ratio

    if (camera.lens_model == FOV_LENS && ((d.x) > 1e-5 || abs(d.y) > 1e-5)) {
        float focal = 1.f / tanf(camera.hFOV / 2.0);
        float2 d_normalized = d / focal;
        float rd = sqrtf(d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y);
        float ru = tanf(rd * camera.hFOV) / (2 * tanf(camera.hFOV / 2.0));
        d = d_normalized * (ru / rd) * focal;

    } else if (camera.lens_model == RADIAL) {
        float focal = 1.f / tanf(camera.hFOV / 2.0);
        float recip_focal = tanf(camera.hFOV / 2.0);
        float2 d_normalized = d * recip_focal;
        float rd2 = d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y;
        float distortion_ratio = radial_function(rd2,camera.lens_parameters);
        d = d_normalized * distortion_ratio * focal;
    }

    if (camera.super_sample_factor > 1) {
        unsigned int local_idx = idx.x % camera.super_sample_factor;
        unsigned int local_idy = idx.y % camera.super_sample_factor;

        float d_local_x = (local_idx + .5) / camera.super_sample_factor - (camera.super_sample_factor / 2);
        float d_local_y = (local_idy + .5) / camera.super_sample_factor - (camera.super_sample_factor / 2);

        float2 dir_change = make_float2(-d_local_y, d_local_x);
        float2 pixel_dist =
            make_float2(2 * camera.super_sample_factor / screen.x, 2 * camera.super_sample_factor / screen.y);
        float2 dist_change =
            pixel_dist *
            sin(.4636);  // * sin(.4636);  // approximately a 26.6 degree roation about the center of the pixel

        d = d + make_float2(dist_change.x * dir_change.x, dist_change.y * dir_change.y);
    }

    float t_frac = 0.f;
    if (camera.rng_buffer)
        t_frac = curand_uniform(
            &camera.rng_buffer[image_index]);  // 0-1 between start and end time of the camera (chosen here)
    const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent

    float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
    float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
    // float3 ray_origin = raygen->pos0;
    // float4 ray_quat = raygen->rot0;
    const float h_factor = camera.hFOV / CUDART_PI_F * 2.0;
    float3 forward;
    float3 left;
    float3 up;

    basis_from_quaternion(ray_quat, forward, left, up);
    float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);

    PerRayData_camera prd = default_camera_prd();
    prd.use_gi = camera.use_gi;
    if (camera.use_gi) {
        prd.rng = camera.rng_buffer[image_index];
    }
    unsigned int opt1;
    unsigned int opt2;
    pointer_as_ints(&prd, opt1, opt2);
    unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
    //printf("CameraRayGen: orig: (%f,%f,%f), dir:(%f,%f,%f)\n", ray_origin.x,ray_origin.y,ray_origin.z, ray_direction.x, ray_direction.y, ray_direction.z);
    optixTrace(params.root, ray_origin, ray_direction, params.scene_epsilon, 1e16f, t_traverse, OptixVisibilityMask(1),
               OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

    // Gamma correct the output color into sRGB color space
    float gamma = camera.gamma;
    camera.frame_buffer[image_index] =
        make_half4(pow(prd.color.x, 1.0f / gamma), pow(prd.color.y, 1.0f / gamma), pow(prd.color.z, 1.0f / gamma), prd.transparency);
    if (camera.use_gi) {
        camera.albedo_buffer[image_index] = make_half4(prd.albedo.x, prd.albedo.y, prd.albedo.z, 0.f);
        float screen_n_x = -Dot(left, prd.normal);     // screen space (x right)
        float screen_n_y = Dot(up, prd.normal);        // screen space (y up)
        float screen_n_z = -Dot(forward, prd.normal);  // screen space (-z forward)
        camera.normal_buffer[image_index] = make_half4(screen_n_x, screen_n_y, screen_n_z, 0.f);
    }
}


extern "C" __global__ void __raygen__transientcamera() {
    //printf("Transient Raygen\n");
    const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
    const TransientCameraParameters& camera = raygen->specific.transientCamera;

    const uint3 idx = optixGetLaunchIndex();
    const uint3 screen = optixGetLaunchDimensions();
    unsigned int nsamples = camera.super_sample_factor * camera.super_sample_factor;

    float inv_nsamples = 1 / static_cast<float>(nsamples);

    for (unsigned int sample = 0; sample < nsamples; sample++) {
        const unsigned int image_index = screen.x * idx.y + idx.x;

        float2 jitter = make_float2(curand_uniform(&camera.rng_buffer[image_index]),
                                    curand_uniform(&camera.rng_buffer[image_index]));
        // float2 d = (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f -
        // make_float2(1.f);
        float2 d = (make_float2(idx.x, idx.y) + jitter) / make_float2(screen.x, screen.y) * 2.f - make_float2(1.f);
        d.y *= (float)(screen.y) / (float)(screen.x);  // correct for the aspect ratio

        if (camera.lens_model == FOV_LENS && ((d.x) > 1e-5 || abs(d.y) > 1e-5)) {
            float focal = 1.f / tanf(camera.hFOV / 2.0);
            float2 d_normalized = d / focal;
            float rd = sqrtf(d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y);
            float ru = tanf(rd * camera.hFOV) / (2 * tanf(camera.hFOV / 2.0));
            d = d_normalized * (ru / rd) * focal;

        } else if (camera.lens_model == RADIAL) {
            float focal = 1.f / tanf(camera.hFOV / 2.0);
            float recip_focal = tanf(camera.hFOV / 2.0);
            float2 d_normalized = d * recip_focal;
            float rd2 = d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y;
            float distortion_ratio = radial_function(rd2, camera.lens_parameters);
            d = d_normalized * distortion_ratio * focal;
        }

        // if (camera.super_sample_factor > 1) {
        //    unsigned int local_idx = idx.x % camera.super_sample_factor;
        //    unsigned int local_idy = idx.y % camera.super_sample_factor;

        //    float d_local_x = (local_idx + .5) / camera.super_sample_factor - (camera.super_sample_factor / 2);
        //    float d_local_y = (local_idy + .5) / camera.super_sample_factor - (camera.super_sample_factor / 2);

        //    float2 dir_change = make_float2(-d_local_y, d_local_x);
        //    float2 pixel_dist =
        //        make_float2(2 * camera.super_sample_factor / screen.x, 2 * camera.super_sample_factor / screen.y);
        //    float2 dist_change =
        //        pixel_dist *
        //        sin(.4636);  // * sin(.4636);  // approximately a 26.6 degree roation about the center of the pixel

        //    d = d + make_float2(dist_change.x * dir_change.x, dist_change.y * dir_change.y);
        //}

        float t_frac = 0.f;
        if (camera.rng_buffer)
            t_frac = curand_uniform(&camera.rng_buffer[image_index]);  // 0-1 between start and end time of the camera (chosen here)
        const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent

        float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
        float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
        // float3 ray_origin = raygen->pos0;
        // float4 ray_quat = raygen->rot0;
        const float h_factor = camera.hFOV / CUDART_PI_F * 2.0;
        float3 forward;
        float3 left;
        float3 up;

        basis_from_quaternion(ray_quat, forward, left, up);
        float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);

        PerRayData_transientCamera prd = default_transientCamera_prd(image_index);
        prd.use_gi = camera.use_gi;
        if (camera.use_gi) {
            prd.rng = camera.rng_buffer[image_index];
        }
        unsigned int opt1;
        unsigned int opt2;
        pointer_as_ints(&prd, opt1, opt2);
        unsigned int raytype = (unsigned int)TRANSIENT_RAY_TYPE;
        /*  printf("TransCameraRayGen: orig: (%f,%f,%f), dir:(%f,%f,%f)\n", ray_origin.x,ray_origin.y,ray_origin.z,
          ray_direction.x, ray_direction.y, ray_direction.z);*/
        optixTrace(params.root, ray_origin, ray_direction, params.scene_epsilon, 1e16f, t_traverse,
                   OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
        // printf("Tracing don!\n");
        // Gamma correct the output color into sRGB color space
        float gamma = camera.gamma;
       /* half4 corrected_color = make_half4(pow(prd.color.x, 1.0f / gamma), pow(prd.color.y, 1.0f / gamma),
                                           pow(prd.color.z, 1.0f / gamma), prd.transparency);*/

        // loop over all transient sample
        int stride = screen.x * screen.y;  // Does not account for multiple spp! (super sampling factor)

        for (int i = 0; i < prd.depth_reached; i++) {
            TransientSample sample = params.transient_buffer[params.max_depth * image_index + i];

            float r = ((sample.pathlength - camera.tmin) / (camera.tmax - camera.tmin)) * camera.tbins;

            unsigned int idx = static_cast<unsigned int>(r);

            // Check if index is valid
            float idx_valid = (sample.pathlength >= camera.tmin) && (sample.pathlength < camera.tmax) ? 1.0f : 0.0f;

         /*   if (idx == 0 && prd.depth_reached > 0 && idx_valid) {
                printf("path length: %f, r: %f, idx: %d, dr: %d, tmin: %f, tmax: %f, tbins:%f, c: (%f,%f,%f)\n",
                       sample.pathlength, r, idx, prd.depth_reached, camera.tmin, camera.tmax, camera.tbins,
                       sample.color.x, sample.color.y, sample.color.z);
            }*/
            // Clamp indices
            unsigned int curr_idx = clamp(idx, 0, camera.tbins - 1);
            unsigned int next_idx = clamp(idx + 1, 0, camera.tbins - 1);

            // Calculate remainder
            float remainder = r - curr_idx;

            float3 L1 = (sample.color * (1 - remainder) * idx_valid) * inv_nsamples;
            float3 L2 = (sample.color * remainder * idx_valid) * inv_nsamples;

            half4 L1_corrected =
                make_half4(pow(L1.x, 1.0f / gamma), pow(L1.y, 1.0f / gamma), pow(L1.z, 1.0f / gamma), prd.transparency);
            half4 L2_corrected =
                make_half4(pow(L2.x, 1.0f / gamma), pow(L2.y, 1.0f / gamma), pow(L2.z, 1.0f / gamma), prd.transparency);

            /*   if (sample.pathlength > 0.f) {
                   printf("path length: %f, color: (%f,%f,%f), tmin:%f, tmax:%f, tbins:%f, r: %f , curridx: %d, nextidx:
               %d, remainer: %f, idxvalid: %f| Adding to buffer idx: %d, L1: (%f,%f,%f)\n", sample.pathlength,
               sample.color.x, sample.color.y, sample.color.z, camera.tmin, camera.tmax, camera.tbins, r, curr_idx,
               next_idx, remainder, idx_valid, stride * curr_idx + image_index, L1.x, L1.y, L1.z);
               }*/

            camera.frame_buffer[stride * curr_idx + image_index].x += L1_corrected.x;
            camera.frame_buffer[stride * curr_idx + image_index].y += L1_corrected.y;
            camera.frame_buffer[stride * curr_idx + image_index].z += L1_corrected.z;

            camera.frame_buffer[stride * next_idx + image_index].x += L2_corrected.x;
            camera.frame_buffer[stride * next_idx + image_index].y += L2_corrected.y;
            camera.frame_buffer[stride * next_idx + image_index].z += L2_corrected.z;
        }
    }
    

}


extern "C" __global__ void __raygen__depthcamera() {
    const RaygenParameters* raygen = (RaygenParameters*) optixGetSbtDataPointer();
    const DepthCameraParameters& camera = raygen->specific.depthCamera;

    const uint3 idx = optixGetLaunchIndex();
    const uint3 screen = optixGetLaunchDimensions();
    const unsigned int image_index = screen.x * idx.y + idx.x;

    float2 d =
        (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f - make_float2(1.f);
    d.y *= (float)(screen.y) / (float)(screen.x);  // correct for the aspect ratio

    if (camera.lens_model == FOV_LENS && ((d.x) > 1e-5 || abs(d.y) > 1e-5)) {
        float focal = 1.f / tanf(camera.hFOV / 2.0);
        float2 d_normalized = d / focal;
        float rd = sqrtf(d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y);
        float ru = tanf(rd * camera.hFOV) / (2 * tanf(camera.hFOV / 2.0));
        d = d_normalized * (ru / rd) * focal;

    } else if (camera.lens_model == RADIAL) {
        float focal = 1.f / tanf(camera.hFOV / 2.0);
        float recip_focal = tanf(camera.hFOV / 2.0);
        float2 d_normalized = d * recip_focal;
        float rd2 = d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y;
        float distortion_ratio = radial_function(rd2,camera.lens_parameters);
        d = d_normalized * distortion_ratio * focal;
    }

    float t_frac = 0.f;
    if (camera.rng_buffer)
        t_frac = curand_uniform(
            &camera.rng_buffer[image_index]);  // 0-1 between start and end time of the camera (chosen here)
    const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent

    float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
    float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
    // float3 ray_origin = raygen->pos0;
    // float4 ray_quat = raygen->rot0;
    const float h_factor = camera.hFOV / CUDART_PI_F * 2.0;
    float3 forward;
    float3 left;
    float3 up;

    basis_from_quaternion(ray_quat, forward, left, up);
    float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);

    PerRayData_depthCamera prd = default_depthCamera_prd();
    
    unsigned int opt1;
    unsigned int opt2;
    pointer_as_ints(&prd, opt1, opt2);
    unsigned int raytype = (unsigned int)DEPTH_RAY_TYPE;
    optixTrace(params.root, ray_origin, ray_direction, params.scene_epsilon, 1e16f, t_traverse, OptixVisibilityMask(1),
               OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

    camera.frame_buffer[image_index] = prd.depth;
}

/// Camera ray generation program using an FOV lens model
extern "C" __global__ void __raygen__segmentation() {
    const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
    const SemanticCameraParameters& camera = raygen->specific.segmentation;

    const uint3 idx = optixGetLaunchIndex();
    const uint3 screen = optixGetLaunchDimensions();
    const unsigned int image_index = screen.x * idx.y + idx.x;

    float2 d =
        (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f - make_float2(1.f);
    d.y *= (float)(screen.y) / (float)(screen.x);  // correct for the aspect ratio

    if (camera.lens_model == FOV_LENS && ((d.x) > 1e-5 || abs(d.y) > 1e-5)) {
        float focal = 1.f / tanf(camera.hFOV / 2.0);
        float2 d_normalized = d / focal;
        float rd = sqrtf(d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y);
        float ru = tanf(rd * camera.hFOV) / (2 * tanf(camera.hFOV / 2.0));
        d = d_normalized * (ru / rd) * focal;

    } else if (camera.lens_model == RADIAL) {
        float focal = 1.f / tanf(camera.hFOV / 2.0);
        float recip_focal = tanf(camera.hFOV / 2.0);
        float2 d_normalized = d * recip_focal;
        float rd2 = d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y;
        float distortion_ratio = radial_function(rd2,camera.lens_parameters);
        d = d_normalized * distortion_ratio * focal;
    }

    // const float t_frac = 0;  // 0-1 between start and end time of the camera (chosen here)
    float t_frac = 0.f;
    if (camera.rng_buffer)
        t_frac = curand_uniform(
            &camera.rng_buffer[image_index]);  // 0-1 between start and end time of the camera (chosen here)

    const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent
    float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
    float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
    const float h_factor = camera.hFOV / CUDART_PI_F * 2.0;
    float3 forward;
    float3 left;
    float3 up;

    basis_from_quaternion(ray_quat, forward, left, up);
    float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);

    PerRayData_semantic prd = default_semantic_prd();
    unsigned int opt1;
    unsigned int opt2;
    pointer_as_ints(&prd, opt1, opt2);
    unsigned int raytype = (unsigned int)SEGMENTATION_RAY_TYPE;
    optixTrace(params.root, ray_origin, ray_direction, params.scene_epsilon, 1e16f, t_traverse, OptixVisibilityMask(1),
               OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
    camera.frame_buffer[image_index].x = prd.class_id;
    camera.frame_buffer[image_index].y = prd.instance_id;
}
