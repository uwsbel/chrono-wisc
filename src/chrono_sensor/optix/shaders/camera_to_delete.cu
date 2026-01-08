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

__device__ float gaussian(int dx, int dy, float sigma) {
    float dist2 = dx * dx + dy * dy;
    return expf(-dist2 / (2 * sigma * sigma)) / (2 * CUDART_PI_F * sigma * sigma);
}

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
    float transparency_result = 0.f;
    float gamma = camera.gamma;
    camera.frame_buffer[pixel_idx] = make_half4(0.f, 0.f, 0.f, 0.f);

    if (camera.use_gi) {
        camera.albedo_buffer[pixel_idx] = make_half4(0.f, 0.f, 0.f, 0.f);
        camera.normal_buffer[pixel_idx] = make_half4(0.f, 0.f, 0.f, 0.f);
    }   
    
    curandState_t rng = camera.rng_buffer[pixel_idx];
    float3 cam_forward, cam_left, cam_up;
    PerRayData_camera prd;
    for (int sample_idx = 0; sample_idx < num_spp; sample_idx++) {
       
        //// Obtain camera's pose (origin of the ray to be launched) ////
        
        // Add motion-blur effect
        float t_frac = (camera.rng_buffer) ? curand_uniform(&rng) : 0.f;
        // float t_frac = static_cast<float>((sample_idx + 1)) * recip_num_spp;  // evenly-spaced midpoint samples over span to compose motion blu
        
        const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent during the frame
        float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
        float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
        
        basis_from_quaternion(ray_quat, cam_forward, cam_left, cam_up);

        //// Obtain (u, v) location on the view plane ////
        
        float2 jitter = (sample_idx == num_spp - 1) ? make_float2(0.5f, 0.5f) : make_float2(curand_uniform(&rng), curand_uniform(&rng)); 
        // UV ~ [{(j + Unif(0, 1)) / img_w * 2 - 1} in range[-1, 1], {(i + Unif(0, 1)) / img_h * 2 - 1} in range[-1, 1]]
        float2 uv = (make_float2(px_2D_idx.x, px_2D_idx.y) + jitter) / make_float2(img_size.x, img_size.y) * 2.f - make_float2(1.f);
        // Correct for the aspect ratio, TODO: This should be added here or after the lens distortion model?
        uv.y *= (float)(img_size.y) / (float)(img_size.x);  

        // Apply lens distortion model
        if (camera.lens_model == FOV_LENS && ((uv.x) > 1e-5 || abs(uv.y) > 1e-5)) {
            float focal = 1.f / tanf(camera.hFOV / 2.0);
            float2 uv_nrmlz = uv / focal;
            float rd = sqrtf(uv_nrmlz.x * uv_nrmlz.x + uv_nrmlz.y * uv_nrmlz.y);
            float ru = tanf(rd * camera.hFOV) / (2 * tanf(camera.hFOV / 2.0));
            uv = uv_nrmlz * (ru / rd) * focal;
        }
        else if (camera.lens_model == RADIAL) {
            float recip_focal = tanf(camera.hFOV / 2.0);
            float2 uv_nrmlz = uv * recip_focal;
            float rd2 = uv_nrmlz.x * uv_nrmlz.x + uv_nrmlz.y * uv_nrmlz.y;
            float distortion_ratio = radial_function(rd2, camera.lens_parameters);
            uv = uv_nrmlz * distortion_ratio / recip_focal;
        }
        
        // Compute ray direction
        // const float h_factor = camera.hFOV / CUDART_PI_F * 2.0; // bug here
        const float h_factor = tanf(camera.hFOV / 2.f);
        float3 ray_direction = normalize(cam_forward - uv.x * cam_left * h_factor + uv.y * cam_up * h_factor);

        // Create per-ray data for camera ray
        prd = default_camera_prd();
        prd.integrator = camera.integrator;
        prd.use_gi = camera.use_gi;
        if (true) { // camera.use_gi
            prd.rng = camera.rng_buffer[pixel_idx];
        }
        unsigned int opt1;
        unsigned int opt2;
        pointer_as_ints(&prd, opt1, opt2);
        unsigned int raytype = RayType::CAMERA_RAY_TYPE;
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
        transparency_result += prd.transparency;
    }

    // Average results over all samples of each pixel
    color_result = color_result * recip_num_spp;
    albedo_result = albedo_result * recip_num_spp;
    transparency_result = transparency_result * recip_num_spp;
    if (camera.use_gi) {
        camera.albedo_buffer[pixel_idx] = make_half4(albedo_result.x, albedo_result.y, albedo_result.z, 0.f);
        // Transform to screen coordinates (x: right, y: up, z: backward)
        camera.normal_buffer[pixel_idx] = make_half4(-Dot(cam_left, prd.normal), Dot(cam_up, prd.normal), -Dot(cam_forward, prd.normal), 0.f);
    }

    bool is_transparent = (transparency_result < 1e-6) ? true : false;

    // Gamma correction
    camera.frame_buffer[pixel_idx] = make_half4(
        pow(color_result.x, 1.0f / gamma), pow(color_result.y, 1.0f / gamma), pow(color_result.z, 1.0f / gamma), transparency_result
    );
}


// extern "C" __global__ void __raygen__transientcamera() {
//     //printf("Transient Raygen\n");
//     const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
//     const TransientCameraParameters& camera = raygen->specific.transientCamera;

//     const uint3 idx = optixGetLaunchIndex();
//     const uint3 screen = optixGetLaunchDimensions();
//     unsigned int nsamples = camera.super_sample_factor * camera.super_sample_factor;

//     float inv_nsamples = 1 / static_cast<float>(nsamples);
//     float gamma = camera.gamma;
//     const unsigned int image_index = screen.x * idx.y + idx.x;

//     float3 accum_color = make_float3(0.f);
//     float3 laser_focus_point;
//     float sigma = .5f;


//     if (camera.integrator == Integrator::MITRANSIENT) {
//         // trace laser ray to find laster focusing point
//         assert(params.num_lights > 0);
//         Light l = params.lights[0];
//         assert(l.type == LightType::SPOT_LIGHT); // MITRANSIENT assumes only one light source and it should be a Spot Light/Projector (TODO)


//         PerRayData_laserSampleRay prd = default_laserSampleRay_prd();
//         unsigned int opt1;
//         unsigned int opt2;
//         pointer_as_ints(&prd, opt1, opt2);
//         unsigned int raytype = (unsigned int)LASER_SAMPLE_RAY_TYPE;

//         optixTrace(params.root, l.pos, l.spot_dir, params.scene_epsilon, 1e16f, 0,
//                    OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype); // Should a ray time be set?
//         assert(prd.laser_hitpoint.x != 1e10 && prd.laser_hitpoint.y != 1e10 && prd.laser_hitpoint.z != 1e10);
//         laser_focus_point = prd.laser_hitpoint;
//         /*printf("Lp: (%f,%f,%f) | Ld: (%f,%f,%f) | Lt: (%f,%f,%f)\n", 
//                l.pos.x, l.pos.y, l.pos.z,
//                l.spot_dir.x, l.spot_dir.y, l.spot_dir.z,
//                laser_focus_point.x, laser_focus_point.y, laser_focus_point.z);*/
//     }

//     for (unsigned int sample = 0; sample < nsamples; sample++) {
//         float2 jitter = make_float2(curand_uniform(&camera.rng_buffer[image_index]),
//                                     curand_uniform(&camera.rng_buffer[image_index]));
//          //float2 d = (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f -
//         // make_float2(1.f);
//         float2 d = (make_float2(idx.x, idx.y) + jitter) / make_float2(screen.x, screen.y) * 2.f - make_float2(1.f);
//         d.y *= (float)(screen.y) / (float)(screen.x);  // correct for the aspect ratio

//         if (camera.lens_model == FOV_LENS && ((d.x) > 1e-5 || abs(d.y) > 1e-5)) {
//             float focal = 1.f / tanf(camera.hFOV / 2.0);
//             float2 d_normalized = d / focal;
//             float rd = sqrtf(d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y);
//             float ru = tanf(rd * camera.hFOV) / (2 * tanf(camera.hFOV / 2.0));
//             d = d_normalized * (ru / rd) * focal;

//         } else if (camera.lens_model == RADIAL) {
//             float focal = 1.f / tanf(camera.hFOV / 2.0);
//             float recip_focal = tanf(camera.hFOV / 2.0);
//             float2 d_normalized = d * recip_focal;
//             float rd2 = d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y;
//             float distortion_ratio = radial_function(rd2, camera.lens_parameters);
//             d = d_normalized * distortion_ratio * focal;
//         }

       
//         float t_frac = 0.f;
//         //if (camera.rng_buffer)
//         //    t_frac = curand_uniform(&camera.rng_buffer[image_index]);  // 0-1 between start and end time of the camera (chosen here)
//         const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent

//         float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
//         float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);

//         const float h_factor = camera.hFOV / CUDART_PI_F * 2.0;
//         float3 forward;
//         float3 left;
//         float3 up;
        
//         basis_from_quaternion(ray_quat, forward, left, up);
//         float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);
        
//         //float2 jitter = make_float2(curand_uniform(&camera.rng_buffer[image_index]), curand_uniform(&camera.rng_buffer[image_index]));

//         //float phi_jitter = (idx.y + jitter.y) / (float)(max(1, screen.y - 1));
//         //float phi = phi_jitter * (camera.maxVFOV - camera.minVFOV) + camera.minVFOV;

//         //float theta_jitter = (idx.x + jitter.x) / (float)(max(1, screen.x - 1));
//         //float theta = theta_jitter * camera.hFOV - camera.hFOV / 2.;

//         //float xy_proj = cos(phi);
//         //float z = sin(phi);
//         //float y = xy_proj * sin(theta);
//         //float x = xy_proj * cos(theta);

//         //const float t_frac = idx.x / (float)screen.x;
//         //const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent
//         //float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
//         //float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
//         //float3 forward;
//         //float3 left;
//         //float3 up;
//         //basis_from_quaternion(ray_quat, forward, left, up);
//         //float3 ray_direction = normalize(forward * x + left * y + up * z);

//         PerRayData_transientCamera prd = default_transientCamera_prd(image_index);
//         prd.integrator = camera.integrator;
//         prd.use_gi = camera.use_gi;
//         prd.laser_focus_point = laser_focus_point;
//         if (true) {
//             prd.rng = camera.rng_buffer[image_index];
//         }
//         unsigned int opt1;
//         unsigned int opt2;
//         pointer_as_ints(&prd, opt1, opt2);
//         unsigned int raytype = (unsigned int)TRANSIENT_RAY_TYPE;

//         optixTrace(params.root, ray_origin, ray_direction, params.scene_epsilon, 1e16f, t_traverse,
//                    OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
//         accum_color += prd.color;
 

//         // loop over all transient sample
//         int stride = screen.x * screen.y;  
//         if (camera.integrator == Integrator::TRANSIENT || camera.integrator == Integrator::MITRANSIENT) {
        
//              for (int i = 0; i < prd.depth_reached; i++) {
//                 TransientSample sample = params.transient_buffer[params.max_depth * image_index + i];

//                 float r = ((sample.pathlength - camera.tmin) / (camera.tmax - camera.tmin)) * camera.tbins;

//                 unsigned int idx_t = static_cast<unsigned int>(r);
           
//                 // Check if index is valid
//                 float idx_valid = (sample.pathlength >= camera.tmin) && (sample.pathlength < camera.tmax) ? 1.0f : 0.0f;
//                 // Clamp indices
//                 unsigned int curr_idx = clamp(idx_t, 0, camera.tbins - 1);
//                 unsigned int next_idx = clamp(idx_t + 1, 0, camera.tbins - 1);

//                 // Calculate remainder
//                 float remainder = r - curr_idx;


//                 float3 L1 = (sample.color * (1 - remainder) * idx_valid) * inv_nsamples;
//                 float3 L2 = (sample.color * remainder * idx_valid) * inv_nsamples;

//                 float4 L1_corrected  = make_float4(pow(L1.x, 1.0f / gamma), pow(L1.y, 1.0f / gamma), pow(L1.z, 1.0f / gamma), 1.f);
//                 float4 L2_corrected = make_float4(pow(L2.x, 1.0f / gamma), pow(L2.y, 1.0f / gamma), pow(L2.z, 1.0f / gamma), 1.f);
                
//                 int kernel_radius = static_cast<int>(2 * sigma);  // Define this based on your sigma
//                 for (int dx = -kernel_radius; dx <= kernel_radius; ++dx) {
//                     for (int dy = -kernel_radius; dy <= kernel_radius; ++dy) {
//                         int nx = clamp(idx.x + dx, 0, (int)screen.x - 1);
//                         int ny = clamp((int)idx.y + dy, 0, (int)screen.y - 1);
//                         float weight = gaussian(dx, dy, sigma);

//                         // print L1 corrected
                      
//                         camera.frame_buffer[stride * curr_idx + ny * screen.x + nx].x += L1_corrected.x * weight;
//                         camera.frame_buffer[stride * curr_idx + ny * screen.x + nx].y += L1_corrected.y * weight;
//                         camera.frame_buffer[stride * curr_idx + ny * screen.x + nx].z += L1_corrected.z * weight;
//                         camera.frame_buffer[stride * curr_idx + ny * screen.x + nx].w = L1_corrected.w;

//                         camera.frame_buffer[stride * next_idx + ny * screen.x + nx].x += L2_corrected.x * weight;
//                         camera.frame_buffer[stride * next_idx + ny * screen.x + nx].y += L2_corrected.y * weight;
//                         camera.frame_buffer[stride * next_idx + ny * screen.x + nx].z += L2_corrected.z * weight;
//                         camera.frame_buffer[stride * next_idx + ny * screen.x + nx].w = L2_corrected.w;
//                     }
//                 }

//                 // print L1 corrected and value at image index
       

//                 // if (idx_valid && i == 0 && sample.pathlength > 0.f) {
//                 //     float4 fb_l1 = camera.frame_buffer[stride * curr_idx + image_index];
//                 //     float4 fb_l2 = camera.frame_buffer[stride * next_idx + image_index];
//                 //     printf("color: (%f,%f,%f), L1: (%f,%f,%f), L2: (%f,%f,%f),r: %f, (1-r): %f,\n", sample.color.x,
//                 //            sample.color.y, sample.color.z, fb_l1.x, fb_l1.y, fb_l1.z, fb_l2.x, fb_l2.y, fb_l2.z,
//                 //            remainder,
//                 //            1 - remainder);
//                 //     float4 outval = camera.frame_buffer[stride * curr_idx + image_index];
//                 //     printf("Outval: (%f,%f,%f,%f)\n", outval.x, outval.y, outval.z, outval.w);
//                 // }
//             }
//         } /*else if (params.integrator == Integrator::TIMEGATED) {
//             float3 color = prd.color * inv_nsamples;
//             half4 corrected_color = make_half4(pow(color.x, 1.0f / gamma), pow(color.y, 1.0f / gamma),
//                                                pow(color.z, 1.0f / gamma), 1.f);

//             camera.frame_buffer[image_index].x += corrected_color.x;
//             camera.frame_buffer[image_index].y += corrected_color.y;
//             camera.frame_buffer[image_index].z += corrected_color.z;
//             camera.frame_buffer[image_index].w = 1.f;
//         }*/
//      }

//      if (camera.integrator == Integrator::TIMEGATED) {
//          accum_color = accum_color * inv_nsamples;
//          //printf("T Color: (%f,%f,%f)\n", accum_color.x, accum_color.y, accum_color.z);
//          float4 corrected_color = make_float4(pow(accum_color.x, 1.0f / gamma), pow(accum_color.y, 1.0f / gamma),
//                                            pow(accum_color.z, 1.0f / gamma), 1.f);
//         camera.frame_buffer[image_index] = corrected_color;
//      }

// }

// /// Physics-based camera ray generation program with different lens distortion model
// extern "C" __global__ void __raygen__phys_camera() {
//     const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
//     const PhysCameraParameters& camera = raygen->specific.phys_camera;

//     const uint3 idx = optixGetLaunchIndex();
//     const uint3 screen = optixGetLaunchDimensions();
//     const unsigned int image_index = screen.x * idx.y + idx.x;

//     float2 d =
//         (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f - make_float2(1.f);
//     d.y *= (float)(screen.y) / (float)(screen.x);  // correct for the aspect ratio

//     if (camera.lens_model == FOV_LENS && ((d.x) > 1e-5 || abs(d.y) > 1e-5)) {
//         float focal = 1.f / tanf(camera.hFOV / 2.0);
//         float2 d_normalized = d / focal;
//         float rd = sqrtf(d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y);
//         float ru = tanf(rd * camera.hFOV) / (2 * tanf(camera.hFOV / 2.0));
//         d = d_normalized * (ru / rd) * focal;
//     }
//     else if (camera.lens_model == RADIAL) {
//         float focal = 1.f / tanf(camera.hFOV / 2.0);
//         float recip_focal = tanf(camera.hFOV / 2.0);
//         float2 d_normalized = d * recip_focal;
//         float rd2 = d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y;
//         float distortion_ratio = radial_function(rd2,camera.lens_parameters);
//         d = d_normalized * distortion_ratio * focal;
//     }

//     if (camera.super_sample_factor > 1) {
//         unsigned int local_idx = idx.x % camera.super_sample_factor;
//         unsigned int local_idy = idx.y % camera.super_sample_factor;

//         float d_local_x = (local_idx + .5) / camera.super_sample_factor - (camera.super_sample_factor / 2);
//         float d_local_y = (local_idy + .5) / camera.super_sample_factor - (camera.super_sample_factor / 2);

//         float2 dir_change = make_float2(-d_local_y, d_local_x);
//         float2 pixel_dist =
//             make_float2(2 * camera.super_sample_factor / screen.x, 2 * camera.super_sample_factor / screen.y);
//         float2 dist_change =
//             pixel_dist *
//             sin(.4636);  // * sin(.4636);  // approximately a 26.6 degree roation about the center of the pixel

//         d = d + make_float2(dist_change.x * dir_change.x, dist_change.y * dir_change.y);
//     }

//     float t_frac = 0.f;
//     if (camera.rng_buffer) {
//         t_frac = curand_uniform(&camera.rng_buffer[image_index]);  // 0-1 between start and end time of the camera (chosen here)
//     }
//     const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent

//     float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
//     float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
//     // float3 ray_origin = raygen->pos0;
//     // float4 ray_quat = raygen->rot0;
//     const float h_factor = camera.hFOV / CUDART_PI_F * 2.0;
//     float3 forward;
//     float3 left;
//     float3 up;

//     basis_from_quaternion(ray_quat, forward, left, up);
//     float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);

//     PerRayData_phys_camera prd = default_phys_camera_prd();
//     prd.use_gi = camera.use_gi;
//     if (camera.use_gi) {
//         prd.rng = camera.rng_buffer[image_index];
//     }
//     unsigned int opt1;
//     unsigned int opt2;
//     pointer_as_ints(&prd, opt1, opt2);
//     unsigned int raytype = (unsigned int)PHYS_CAMERA_RAY_TYPE;
//     optixTrace(params.root, ray_origin, ray_direction, params.scene_epsilon, 1e16f, t_traverse, OptixVisibilityMask(1),
//                OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

//     // Gamma correct the output color into sRGB color space
//     float gamma = camera.gamma;
//     camera.rgbd_buffer[image_index] =
//         make_half4(pow(prd.color.x, 1.0f / gamma), pow(prd.color.y, 1.0f / gamma), pow(prd.color.z, 1.0f / gamma), prd.distance);
    
//     if (camera.use_gi) {
//         camera.albedo_buffer[image_index] = make_half4(prd.albedo.x, prd.albedo.y, prd.albedo.z, 0.f);
//         float screen_n_x = -Dot(left, prd.normal);     // screen space (x right)
//         float screen_n_y = Dot(up, prd.normal);        // screen space (y up)
//         float screen_n_z = -Dot(forward, prd.normal);  // screen space (-z forward)
//         camera.normal_buffer[image_index] = make_half4(screen_n_x, screen_n_y, screen_n_z, 0.f);
//     }
// }

// extern "C" __global__ void __raygen__depthcamera() {
//     const RaygenParameters* raygen = (RaygenParameters*) optixGetSbtDataPointer();
//     const DepthCameraParameters& camera = raygen->specific.depthCamera;

//     const uint3 idx = optixGetLaunchIndex();
//     const uint3 screen = optixGetLaunchDimensions();
//     const unsigned int image_index = screen.x * idx.y + idx.x;

//     float2 d =
//         (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f - make_float2(1.f);
//     d.y *= (float)(screen.y) / (float)(screen.x);  // correct for the aspect ratio

//     if (camera.lens_model == FOV_LENS && ((d.x) > 1e-5 || abs(d.y) > 1e-5)) {
//         float focal = 1.f / tanf(camera.hFOV / 2.0);
//         float2 d_normalized = d / focal;
//         float rd = sqrtf(d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y);
//         float ru = tanf(rd * camera.hFOV) / (2 * tanf(camera.hFOV / 2.0));
//         d = d_normalized * (ru / rd) * focal;

//     } else if (camera.lens_model == RADIAL) {
//         float focal = 1.f / tanf(camera.hFOV / 2.0);
//         float recip_focal = tanf(camera.hFOV / 2.0);
//         float2 d_normalized = d * recip_focal;
//         float rd2 = d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y;
//         float distortion_ratio = radial_function(rd2,camera.lens_parameters);
//         d = d_normalized * distortion_ratio * focal;
//     }

//     float t_frac = 0.f;
//     if (camera.rng_buffer)
//         t_frac = curand_uniform(
//             &camera.rng_buffer[image_index]);  // 0-1 between start and end time of the camera (chosen here)
//     const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent

//     float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
//     float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
//     // float3 ray_origin = raygen->pos0;
//     // float4 ray_quat = raygen->rot0;
//     const float h_factor = camera.hFOV / CUDART_PI_F * 2.0;
//     float3 forward;
//     float3 left;
//     float3 up;

//     basis_from_quaternion(ray_quat, forward, left, up);
//     float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);

//     PerRayData_depthCamera prd = default_depthCamera_prd(camera.max_depth);
    
//     unsigned int opt1;
//     unsigned int opt2;
//     pointer_as_ints(&prd, opt1, opt2);
//     unsigned int raytype = (unsigned int)DEPTH_RAY_TYPE;
//     optixTrace(params.root, ray_origin, ray_direction, params.scene_epsilon, 1e16f, t_traverse, OptixVisibilityMask(1),
//                OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

//     camera.frame_buffer[image_index] = prd.depth;
// }

// /// Camera ray generation program for normal camera
// extern "C" __global__ void __raygen__normalcamera() {
//     const RaygenParameters* raygen = (RaygenParameters*) optixGetSbtDataPointer();
//     const NormalCameraParameters& camera = raygen->specific.normalCamera;

//     const uint3 idx = optixGetLaunchIndex();
//     const uint3 screen = optixGetLaunchDimensions();
//     const unsigned int image_index = screen.x * idx.y + idx.x;

//     float2 d =
//         (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f - make_float2(1.f);
//     d.y *= (float)(screen.y) / (float)(screen.x);  // correct for the aspect ratio

//     // FOV lens model
//     if (camera.lens_model == FOV_LENS && ((d.x) > 1e-5 || abs(d.y) > 1e-5)) {
//         float focal = 1.f / tanf(camera.hFOV / 2.0);
//         float2 d_normalized = d / focal;
//         float rd = sqrtf(d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y);
//         float ru = tanf(rd * camera.hFOV) / (2 * tanf(camera.hFOV / 2.0));
//         d = d_normalized * (ru / rd) * focal;

//     } // radial lens model
//     else if (camera.lens_model == RADIAL) {
//         float focal = 1.f / tanf(camera.hFOV / 2.0);
//         float recip_focal = tanf(camera.hFOV / 2.0);
//         float2 d_normalized = d * recip_focal;
//         float rd2 = d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y;
//         float distortion_ratio = radial_function(rd2,camera.lens_parameters);
//         d = d_normalized * distortion_ratio * focal;
//     }

//     float t_frac = 0.f;
//     if (camera.rng_buffer) {
//         t_frac = curand_uniform(&camera.rng_buffer[image_index]);  // 0-1 between start and end time of the camera (chosen here)
//     }
//     const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent

//     float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
//     float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
//     // float3 ray_origin = raygen->pos0;
//     // float4 ray_quat = raygen->rot0;
//     const float h_factor = camera.hFOV / CUDART_PI_F * 2.0;
//     float3 forward;
//     float3 left;
//     float3 up;

//     basis_from_quaternion(ray_quat, forward, left, up);
//     float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);

//     PerRayData_normalCamera prd = default_normalCamera_prd();
    
//     unsigned int opt1;
//     unsigned int opt2;
//     pointer_as_ints(&prd, opt1, opt2);
//     unsigned int raytype = (unsigned int)NORMAL_RAY_TYPE;
//     optixTrace(params.root, ray_origin, ray_direction, params.scene_epsilon, 1e16f, t_traverse, OptixVisibilityMask(1),
//                OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

//     camera.frame_buffer[image_index] = make_float3(prd.normal.x, prd.normal.y, prd.normal.z);
// }

// /// Camera ray generation program using an FOV lens model
// extern "C" __global__ void __raygen__segmentation() {
//     const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
//     const SemanticCameraParameters& camera = raygen->specific.segmentation;

//     const uint3 idx = optixGetLaunchIndex();
//     const uint3 screen = optixGetLaunchDimensions();
//     const unsigned int image_index = screen.x * idx.y + idx.x;

//     float2 d =
//         (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f - make_float2(1.f);
//     d.y *= (float)(screen.y) / (float)(screen.x);  // correct for the aspect ratio

//     if (camera.lens_model == FOV_LENS && ((d.x) > 1e-5 || abs(d.y) > 1e-5)) {
//         float focal = 1.f / tanf(camera.hFOV / 2.0);
//         float2 d_normalized = d / focal;
//         float rd = sqrtf(d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y);
//         float ru = tanf(rd * camera.hFOV) / (2 * tanf(camera.hFOV / 2.0));
//         d = d_normalized * (ru / rd) * focal;

//     } else if (camera.lens_model == RADIAL) {
//         float focal = 1.f / tanf(camera.hFOV / 2.0);
//         float recip_focal = tanf(camera.hFOV / 2.0);
//         float2 d_normalized = d * recip_focal;
//         float rd2 = d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y;
//         float distortion_ratio = radial_function(rd2,camera.lens_parameters);
//         d = d_normalized * distortion_ratio * focal;
//     }

//     // const float t_frac = 0;  // 0-1 between start and end time of the camera (chosen here)
//     float t_frac = 0.f;
//     if (camera.rng_buffer)
//         t_frac = curand_uniform(
//             &camera.rng_buffer[image_index]);  // 0-1 between start and end time of the camera (chosen here)

//     const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent
//     float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
//     float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
//     const float h_factor = camera.hFOV / CUDART_PI_F * 2.0;
//     float3 forward;
//     float3 left;
//     float3 up;

//     basis_from_quaternion(ray_quat, forward, left, up);
//     float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);

//     PerRayData_semantic prd = default_semantic_prd();
//     unsigned int opt1;
//     unsigned int opt2;
//     pointer_as_ints(&prd, opt1, opt2);
//     unsigned int raytype = (unsigned int)SEGMENTATION_RAY_TYPE;
//     optixTrace(params.root, ray_origin, ray_direction, params.scene_epsilon, 1e16f, t_traverse, OptixVisibilityMask(1),
//                OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
//     camera.frame_buffer[image_index].x = prd.class_id;
//     camera.frame_buffer[image_index].y = prd.instance_id;
// }
