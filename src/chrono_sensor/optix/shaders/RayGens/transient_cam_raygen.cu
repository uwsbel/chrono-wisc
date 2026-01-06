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
// camera ray launch kernels
//
// =============================================================================

#include "chrono_sensor/optix/shaders/device_utils.h"


/// Transient camera ray generation program with using a lens distortion model
extern "C" __global__ void __raygen__transientcamera() {
    //printf("Transient Raygen\n");
    const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
    const TransientCameraParameters& camera = raygen->specific.transientCamera;

    const uint3 idx = optixGetLaunchIndex();
    const uint3 screen = optixGetLaunchDimensions();
    unsigned int nsamples = camera.super_sample_factor * camera.super_sample_factor;

    float inv_nsamples = 1 / static_cast<float>(nsamples);
    float gamma = camera.gamma;
    const unsigned int image_index = screen.x * idx.y + idx.x;

    float3 accum_color = make_float3(0.f);
    float3 laser_focus_point;
    float sigma = .5f;


    if (camera.integrator == Integrator::MITRANSIENT) {
        // trace laser ray to find laster focusing point
        assert(params.num_lights > 0);
        Light l = params.lights[0];
        assert(l.type == LightType::SPOT_LIGHT); // MITRANSIENT assumes only one light source and it should be a Spot Light/Projector (TODO)


        PerRayData_laserSampleRay prd = default_laserSampleRay_prd();
        unsigned int opt1;
        unsigned int opt2;
        pointer_as_ints(&prd, opt1, opt2);
        unsigned int raytype = (unsigned int)LASER_SAMPLE_RAY_TYPE;

        optixTrace(params.root, l.pos, l.spot_dir, params.scene_epsilon, 1e16f, 0,
                   OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype); // Should a ray time be set?
        assert(prd.laser_hitpoint.x != 1e10 && prd.laser_hitpoint.y != 1e10 && prd.laser_hitpoint.z != 1e10);
        laser_focus_point = prd.laser_hitpoint;
        /*printf("Lp: (%f,%f,%f) | Ld: (%f,%f,%f) | Lt: (%f,%f,%f)\n", 
               l.pos.x, l.pos.y, l.pos.z,
               l.spot_dir.x, l.spot_dir.y, l.spot_dir.z,
               laser_focus_point.x, laser_focus_point.y, laser_focus_point.z);*/
    }

    for (unsigned int sample = 0; sample < nsamples; sample++) {
        float2 jitter = make_float2(curand_uniform(&camera.rng_buffer[image_index]),
                                    curand_uniform(&camera.rng_buffer[image_index]));
         //float2 d = (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f -
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

       
        float t_frac = 0.f;
        //if (camera.rng_buffer)
        //    t_frac = curand_uniform(&camera.rng_buffer[image_index]);  // 0-1 between start and end time of the camera (chosen here)
        const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent

        float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
        float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);

        const float h_factor = camera.hFOV / CUDART_PI_F * 2.0;
        float3 forward;
        float3 left;
        float3 up;
        
        basis_from_quaternion(ray_quat, forward, left, up);
        float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);
        
        //float2 jitter = make_float2(curand_uniform(&camera.rng_buffer[image_index]), curand_uniform(&camera.rng_buffer[image_index]));

        //float phi_jitter = (idx.y + jitter.y) / (float)(max(1, screen.y - 1));
        //float phi = phi_jitter * (camera.maxVFOV - camera.minVFOV) + camera.minVFOV;

        //float theta_jitter = (idx.x + jitter.x) / (float)(max(1, screen.x - 1));
        //float theta = theta_jitter * camera.hFOV - camera.hFOV / 2.;

        //float xy_proj = cos(phi);
        //float z = sin(phi);
        //float y = xy_proj * sin(theta);
        //float x = xy_proj * cos(theta);

        //const float t_frac = idx.x / (float)screen.x;
        //const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent
        //float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
        //float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
        //float3 forward;
        //float3 left;
        //float3 up;
        //basis_from_quaternion(ray_quat, forward, left, up);
        //float3 ray_direction = normalize(forward * x + left * y + up * z);

        PerRayData_transientCamera prd = default_transientCamera_prd(image_index);
        prd.integrator = camera.integrator;
        prd.use_gi = camera.use_gi;
        prd.laser_focus_point = laser_focus_point;
        if (true) {
            prd.rng = camera.rng_buffer[image_index];
        }
        unsigned int opt1;
        unsigned int opt2;
        pointer_as_ints(&prd, opt1, opt2);
        unsigned int raytype = (unsigned int)TRANSIENT_RAY_TYPE;

        optixTrace(params.root, ray_origin, ray_direction, params.scene_epsilon, 1e16f, t_traverse,
                   OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
        accum_color += prd.color;
 

        // loop over all transient sample
        int stride = screen.x * screen.y;  
        if (camera.integrator == Integrator::TRANSIENT || camera.integrator == Integrator::MITRANSIENT) {
        
             for (int i = 0; i < prd.depth_reached; i++) {
                TransientSample sample = params.transient_buffer[params.max_depth * image_index + i];

                float r = ((sample.pathlength - camera.tmin) / (camera.tmax - camera.tmin)) * camera.tbins;

                unsigned int idx_t = static_cast<unsigned int>(r);
           
                // Check if index is valid
                float idx_valid = (sample.pathlength >= camera.tmin) && (sample.pathlength < camera.tmax) ? 1.0f : 0.0f;
                // Clamp indices
                unsigned int curr_idx = clamp(idx_t, 0, camera.tbins - 1);
                unsigned int next_idx = clamp(idx_t + 1, 0, camera.tbins - 1);

                // Calculate remainder
                float remainder = r - curr_idx;


                float3 L1 = (sample.color * (1 - remainder) * idx_valid) * inv_nsamples;
                float3 L2 = (sample.color * remainder * idx_valid) * inv_nsamples;

                float4 L1_corrected  = make_float4(pow(L1.x, 1.0f / gamma), pow(L1.y, 1.0f / gamma), pow(L1.z, 1.0f / gamma), 1.f);
                float4 L2_corrected = make_float4(pow(L2.x, 1.0f / gamma), pow(L2.y, 1.0f / gamma), pow(L2.z, 1.0f / gamma), 1.f);
                
                int kernel_radius = static_cast<int>(2 * sigma);  // Define this based on your sigma
                for (int dx = -kernel_radius; dx <= kernel_radius; ++dx) {
                    for (int dy = -kernel_radius; dy <= kernel_radius; ++dy) {
                        int nx = clamp(idx.x + dx, 0, (int)screen.x - 1);
                        int ny = clamp((int)idx.y + dy, 0, (int)screen.y - 1);
                        float weight = gaussian(dx, dy, sigma);

                        // print L1 corrected
                      
                        camera.frame_buffer[stride * curr_idx + ny * screen.x + nx].x += L1_corrected.x * weight;
                        camera.frame_buffer[stride * curr_idx + ny * screen.x + nx].y += L1_corrected.y * weight;
                        camera.frame_buffer[stride * curr_idx + ny * screen.x + nx].z += L1_corrected.z * weight;
                        camera.frame_buffer[stride * curr_idx + ny * screen.x + nx].w = L1_corrected.w;

                        camera.frame_buffer[stride * next_idx + ny * screen.x + nx].x += L2_corrected.x * weight;
                        camera.frame_buffer[stride * next_idx + ny * screen.x + nx].y += L2_corrected.y * weight;
                        camera.frame_buffer[stride * next_idx + ny * screen.x + nx].z += L2_corrected.z * weight;
                        camera.frame_buffer[stride * next_idx + ny * screen.x + nx].w = L2_corrected.w;
                    }
                }

                // print L1 corrected and value at image index
       

                // if (idx_valid && i == 0 && sample.pathlength > 0.f) {
                //     float4 fb_l1 = camera.frame_buffer[stride * curr_idx + image_index];
                //     float4 fb_l2 = camera.frame_buffer[stride * next_idx + image_index];
                //     printf("color: (%f,%f,%f), L1: (%f,%f,%f), L2: (%f,%f,%f),r: %f, (1-r): %f,\n", sample.color.x,
                //            sample.color.y, sample.color.z, fb_l1.x, fb_l1.y, fb_l1.z, fb_l2.x, fb_l2.y, fb_l2.z,
                //            remainder,
                //            1 - remainder);
                //     float4 outval = camera.frame_buffer[stride * curr_idx + image_index];
                //     printf("Outval: (%f,%f,%f,%f)\n", outval.x, outval.y, outval.z, outval.w);
                // }
            }
        } /*else if (params.integrator == Integrator::TIMEGATED) {
            float3 color = prd.color * inv_nsamples;
            half4 corrected_color = make_half4(pow(color.x, 1.0f / gamma), pow(color.y, 1.0f / gamma),
                                               pow(color.z, 1.0f / gamma), 1.f);

            camera.frame_buffer[image_index].x += corrected_color.x;
            camera.frame_buffer[image_index].y += corrected_color.y;
            camera.frame_buffer[image_index].z += corrected_color.z;
            camera.frame_buffer[image_index].w = 1.f;
        }*/
     }

     if (camera.integrator == Integrator::TIMEGATED) {
         accum_color = accum_color * inv_nsamples;
         //printf("T Color: (%f,%f,%f)\n", accum_color.x, accum_color.y, accum_color.z);
         float4 corrected_color = make_float4(pow(accum_color.x, 1.0f / gamma), pow(accum_color.y, 1.0f / gamma),
                                           pow(accum_color.z, 1.0f / gamma), 1.f);
        camera.frame_buffer[image_index] = corrected_color;
     }

}

