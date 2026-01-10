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
// Path_v1 camera shader
//
// =============================================================================

#ifndef CAMERA_PATHV1_SHADER_CU
#define CAMERA_PATHV1_SHADER_CU

#include "chrono_sensor/optix/shaders/device_utils.h"
#include "chrono_sensor/optix/shaders/shader_utils.cu"


// Camera path integrator
static __device__ __inline__ void CameraPathv1Integrator(PerRayData_camera* prd_camera,
                                                       const MaterialRecordParameters* mat_params,
                                                       unsigned int& material_id,
                                                       const float3& world_normal,
                                                       const float2& uv,
                                                       const float3& tangent,
                                                       const float& ray_dist,
                                                       const float3& hit_point,
                                                       const float3& ray_dir) {
    // if (prd_camera->depth >= 3)
    //     printf("PI| d: %d | contr: (%f,%f,%f)\n", prd_camera->depth, prd_camera->contrib_to_pixel.x,
    //            prd_camera->contrib_to_pixel.y, prd_camera->contrib_to_pixel.z);
    const MaterialParameters& mat = params.material_pool[material_id];
    float3 Kd = (mat.kd_tex) ? Pow(GetTexFloat4ValFloat3(mat.kd_tex, mat.tex_scale, uv), 2.2) : mat.Kd;
    BSDFType bsdf_type = mat.bsdf_type;
    
    float3 L = make_float3(0.0f);

    float3 wo = -ray_dir;

    // Add ambient light
    // prd_camera->color += params.ambient_light_color * prd_camera->contrib_to_pixel;  // ?

    float3 Le = make_float3(0.f);
    // TODO: Add Emisions from Area Lights

    // Direct light contributions
    float3 Ld = make_float3(0.f);

    if (params.num_lights > 0 && bsdf_type != BSDFType::SPECULAR) {
        // printf("Direct Light| BSDF: %d != %d\n", bsdf_type, BSDFType::SPECULAR);
        
        //  Uniformly sample a light source
        unsigned int sample_light_index = (unsigned int)(curand_uniform(&prd_camera->rng) * params.num_lights);
        // TODO: Won't work for whitted as no GI, have a global sampler instead?
        Light l = params.lights[sample_light_index];
        LightSample ls;
        ls.hitpoint = hit_point;
        ls.wo = wo;
        ls.n = world_normal;

        // Compute direct lighting
        // float3 ld = ComputeDirectLight(l, ls, mat, prd_camera->depth);
        // printf("d: %d | DL: (%f,%f,%f) \n", prd_camera->depth, ld.x,ld.y,ld.z);
        Ld = prd_camera->contrib_to_pixel * ComputeDirectLight(l, ls, mat, uv, prd_camera->depth);
    }
    L += Ld;

    if (prd_camera->depth + 1 < params.max_depth) {
        // printf("Next ray!\n");
        BSDFSample sample;
        sample.wo = wo;
        sample.n = world_normal;
        float z1 = curand_uniform(&prd_camera->rng);
        float z2 = curand_uniform(&prd_camera->rng);
        SampleBSDF(bsdf_type, sample, mat, uv, false, z1, z2);
        float NdL = Dot(sample.n, sample.wi);
        float3 next_contrib_to_pixel = prd_camera->contrib_to_pixel * sample.f * NdL / sample.pdf;
        if (luminance(sample.f) > params.importance_cutoff && sample.pdf > 0 && fmaxf(next_contrib_to_pixel) > 0) {
            // Check possible rr termination
            float rr_thresh = .1f;  // Replace this with importance_cutoff?
            if (fmaxf(next_contrib_to_pixel) < rr_thresh && prd_camera->depth > 3) {
                float q = fmaxf((float).05, 1 - fmaxf(next_contrib_to_pixel));
                float p = curand_uniform(&prd_camera->rng);
                if (p < q)
                    return;
                next_contrib_to_pixel = next_contrib_to_pixel / (1 - q);
            }

            // Trace next ray
            PerRayData_camera prd_reflection = DefaultCameraPRD();
            prd_reflection.integrator = prd_camera->integrator;
            prd_reflection.contrib_to_pixel = next_contrib_to_pixel;
            prd_reflection.rng = prd_camera->rng;
            prd_reflection.depth = prd_camera->depth + 1;
            prd_reflection.use_gi = prd_camera->use_gi;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_reflection, opt1, opt2);
            unsigned int raytype = static_cast<unsigned int>(RayType::CAMERA_RAY_TYPE);
            optixTrace(params.root, hit_point, sample.wi, params.scene_epsilon, 1e16f, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
            L += prd_reflection.color;
        }
    }

    prd_camera->color += L;
    prd_camera->albedo = Kd;  // Might change
    prd_camera->normal = world_normal;
}

#endif  // CAMERA_PATHV1_SHADER_CU