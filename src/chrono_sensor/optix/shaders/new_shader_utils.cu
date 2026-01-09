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
// Authors: Asher Elmquist, Han Wang, Yan Xiao
// =============================================================================
//
// RT util functions for material shading
//
// =============================================================================

#ifndef SHADER_UTILS_H
#define SHADER_UTILS_H

#include "chrono_sensor/optix/shaders/device_utils.h"

static __device__ __inline__ float3 CrossProduct(const float3& a, const float3& b) {
    return {(a.y * b.z) - (b.y * a.z), (a.z * b.x) - (b.z * a.x), (a.x * b.y) - (b.x * a.y)};
}

static __device__ __inline__ float NormalDist(const float& NdH, const float& roughness) {
    float rough_sqr = roughness * roughness;
    float den_2 = NdH * NdH * (rough_sqr - 1.f) + 1.f;
    float denominator = den_2 * den_2;
    return rough_sqr / denominator;
}

// algorithm reference: https://www.gdcvault.com/play/1024478/PBR-Diffuse-Lighting-for-GGX
static __device__ __inline__ float HammonSmith(float NdV, float NdL, const float& roughness) {
    NdV = abs(NdV);
    NdL = abs(NdL);
    float denominator = lerp(2.f * NdV * NdL, NdL + NdV, roughness);
    return 0.5f / denominator;
}

// triangle mesh querie information
__device__ __inline__ void GetTriangleData(float3& normal, unsigned int& mat_id, float2& uv, float3& tangent, const unsigned int& mesh_id) {
    
    const int tri_id = optixGetPrimitiveIndex();
    const float2 bary_coord = optixGetTriangleBarycentrics();

    const MeshParameters& mesh_params = params.mesh_pool[mesh_id];
    const uint4& vertex_idx = mesh_params.vertex_index_buffer[tri_id];

    const float3& v1 = make_float3(mesh_params.vertex_buffer[vertex_idx.x]);
    const float3& v2 = make_float3(mesh_params.vertex_buffer[vertex_idx.y]);
    const float3& v3 = make_float3(mesh_params.vertex_buffer[vertex_idx.z]);

    //// Calculate normales either from normal buffer or vertex positions ////
    
    if (mesh_params.normal_index_buffer && mesh_params.normal_buffer) {  // Use vertex normals if normal index buffer exists
        const uint4& normal_idx = mesh_params.normal_index_buffer[tri_id];

        normal = normalize(make_float3(mesh_params.normal_buffer[normal_idx.y]) * bary_coord.x +
                           make_float3(mesh_params.normal_buffer[normal_idx.z]) * bary_coord.y +
                           make_float3(mesh_params.normal_buffer[normal_idx.x]) * (1.0f - bary_coord.x - bary_coord.y));

    }
    else {  // else use face normals calculated from vertices
        normal = normalize(Cross(v2 - v1, v3 - v1));
    }

    // calculate texcoords if they exist
    if (mesh_params.uv_index_buffer && mesh_params.uv_buffer) {  // use vertex normals if normal index buffer exists
        const uint4& uv_idx = mesh_params.uv_index_buffer[tri_id];
        const float2& uv1 = mesh_params.uv_buffer[uv_idx.x];
        const float2& uv2 = mesh_params.uv_buffer[uv_idx.y];
        const float2& uv3 = mesh_params.uv_buffer[uv_idx.z];

        uv = uv2 * bary_coord.x + uv3 * bary_coord.y + uv1 * (1.0f - bary_coord.x - bary_coord.y);
        float3 e1 = v2 - v1;
        float3 e2 = v3 - v1;
        float2 delta_uv1 = uv2 - uv1;
        float2 delta_uv2 = uv3 - uv1;
        float f = 1.f / (delta_uv1.x * delta_uv2.y - delta_uv2.x * delta_uv1.y);
        tangent.x = f * (delta_uv2.y * e1.x - delta_uv1.y * e2.x);
        tangent.y = f * (delta_uv2.y * e1.y - delta_uv1.y * e2.y);
        tangent.z = f * (delta_uv2.y * e1.z - delta_uv1.y * e2.z);
        tangent = normalize(tangent);
    } else {
        uv = make_float2(0.f);
        tangent = make_float3(0.f);
    }

    // get material index
    if (mesh_params.mat_index_buffer) {                  // use vertex normals if normal index buffer exists
        mat_id += mesh_params.mat_index_buffer[tri_id];  // the material index gives an offset id
    }
}

// Return default per ray data (PRD) of shadow
__device__ __inline__ PerRayData_shadow DefaultShadowPRD() {
    PerRayData_shadow prd = {
        make_float3(1.f, 1.f, 1.f),  // default opacity amount
        3,                           // default depth
        0.f                          // default distance remaining to light
    };                        
    return prd;
};

static __device__ inline void SamplePointLight(Light pl, LightSample* ls) {
    ls->dir = normalize(pl.pos - ls->hitpoint);  // How much slow down due to derefing hitpoint twice?
    float dist = Length(pl.pos - ls->hitpoint);
    ls->dist = dist;
    ls->pdf = 1.f;
    float point_light_falloff = (pl.max_range * pl.max_range / (dist * dist + pl.max_range * pl.max_range));
    ls->L = pl.color * point_light_falloff;
}

static __device__ inline void SampleSpotLight(Light spot, LightSample* ls) {
    ls->dir = normalize(spot.pos - ls->hitpoint);  // How much slow down due to derefing hitpoint twice?
    float dist = Length(spot.pos - ls->hitpoint);
    ls->dist = dist;
    ls->pdf = 1.f;

    float cos_theta = Dot(spot.spot_dir, -1 * ls->dir);

    // Replace max range with a high intensity
    // float point_light_falloff = (spot.max_range * spot.max_range / (dist * dist + spot.max_range * spot.max_range));
    ls->L = spot.color / (dist * dist);

    float falloff;  // spot light falloff
    if (cos_theta >= spot.cos_falloff_start) {
        falloff = 1.f;
        return;
    }
    if (cos_theta < spot.cos_total_width) {
        falloff = 0.f;
        ls->L = make_float3(0.f);
        return;
    }

    float delta = (cos_theta - spot.cos_total_width) / (spot.cos_falloff_start - spot.cos_total_width);
    falloff = (delta * delta) * (delta * delta);

    ls->L = ls->L * falloff;
    // printf("falloff: %f | dist: %f | cosTheta: %f\n", falloff, dist, cos_theta*180/CUDART_PI);
}

static __device__ inline void SampleLight(Light light, LightSample* ls) {
    switch (light.type) {
        case LightType::POINT_LIGHT:
            SamplePointLight(light, ls);
            break;
        case LightType::SPOT_LIGHT:
            // printf("Sample Spot!\n");
            SampleSpotLight(light, ls);
            break;
        default:
            break;
    }
}

// Get texture value in float
static __device__ __inline__ float GetTexValFloat(const cudaTextureObject_t& tex, const float2& text_scale, const float2& uv) {
    return tex2D<float>(tex, uv.x * text_scale.x, uv.y * text_scale.y);
}

static __device__ __inline__ float3 GetTexFloat4ValFloat3(const cudaTextureObject_t& tex, const float2& text_scale, const float2& uv) {
    float4 temp = tex2D<float4>(tex, uv.x * text_scale.x, uv.y * text_scale.y);
    return {temp.x, temp.y, temp.z};
}

// Calculate refracted color
static __device__ __inline__ float3 GetRefractedColor(const ContextParameters& cntxt_params,
                                                            PerRayData_camera* prd_camera,
                                                            const int& num_blended_materials,
                                                            unsigned int& material_id,
                                                            const float2& uv,
                                                            const float3& hit_point,
                                                            const float3& ray_dir) {
    float accum_opacity = 0.f;
    // Accumulate opacity by multiplication
    for (int b = 0; b < num_blended_materials; b++) {
        const MaterialParameters& mat = params.material_pool[material_id + b];
        float opacity = (mat.opacity_tex) ? GetTexValFloat(mat.opacity_tex, mat.tex_scale, uv) : mat.transparency;
        float mat_blend_weight = (mat.weight_tex) ? GetTexValFloat(mat.weight_tex, mat.tex_scale, uv) : (1.f / num_blended_materials);
    
        accum_opacity += mat_blend_weight * opacity;
    }

    float3 refracted_color = make_float3(0);
    if (accum_opacity < 0.99f) {
        float3 refract_importance = prd_camera->contrib_to_pixel * (1 - accum_opacity);
        // Launch the next refracted ray
        if (fmaxf(refract_importance) > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
            PerRayData_camera prd_refraction = DefaultCameraPRD();
            prd_refraction.integrator = prd_camera->integrator;
            prd_refraction.contrib_to_pixel = refract_importance;
            prd_refraction.rng = prd_camera->rng;
            prd_refraction.depth = prd_camera->depth + 1;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_refraction, opt1, opt2);

            // make_camera_data(make_float3(0), refract_importance, prd_camera.rnd, prd_camera.depth + 1);
            // float3 refract_dir = refract(optixGetWorldRayDirection(), world_normal, 1.f, 1.f);
            float3 refract_dir = ray_dir;  // pure transparency without refraction
            unsigned int raytype = static_cast<unsigned int>(RayType::CAMERA_RAY_TYPE);
            optixTrace(params.root, hit_point, refract_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                        OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
            refracted_color = prd_refraction.color;  // TODO: not sure added here or not
        }
    }

    return refracted_color;
}


static __device__ __inline__ float3 PrincipledBRDF(
    const MaterialParameters& mat, const float2& uv, const int& num_materials, const float& NdV, const float& NdL,
    const float& NdH, const float& VdH
) {
    // Get BRDF model parameter values, with using textures' values if textures exist
    float3 albedo = (mat.kd_tex) ? Pow(GetTexFloat4ValFloat3(mat.kd_tex, mat.tex_scale, uv), 2.2) : mat.Kd; // transfer texture from sRGB space into linear color space
    float roughness = (mat.roughness_tex) ? GetTexValFloat(mat.roughness_tex, mat.tex_scale, uv): mat.roughness;
    float metallic = (mat.metallic_tex) ? GetTexValFloat(mat.metallic_tex, mat.tex_scale, uv) : mat.metallic;
    float opacity = (mat.opacity_tex) ? GetTexValFloat(mat.opacity_tex, mat.tex_scale, uv) : mat.transparency;
    float mat_blend_weight = (mat.weight_tex) ? GetTexValFloat(mat.weight_tex, mat.tex_scale, uv) : (1.f / num_materials);
    float contrib_weight = opacity * mat_blend_weight;  // correct for transparency, light bounces, and blend weight

    // float3 subsurface_albedo_updated = subsurface_albedo;
    
    // ==== Specular portion of reflection ==== //
    float3 F = make_float3(0.0f);
    // Use dielectric workflow
    if (mat.use_specular_workflow) {
        float3 specular = (mat.ks_tex) ? GetTexFloat4ValFloat3(mat.ks_tex, mat.tex_scale, uv) : mat.Ks;
        float3 F0 = specular * 0.08f;
        F = fresnel_schlick(VdH, 5.f, F0, make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
    }
    else {
        F = metallic * albedo + (1 - metallic) * make_float3(0.04f); // default dielectrics F0 = [0.04, 0.04, 0.04]
        albedo = albedo * (1 - metallic);  // since metals do not do surface reflection
    }
    float3 light_reflected_ratio = ((make_float3(1.f) - F) * albedo) * contrib_weight * NdL;
    
    // ==== Diffuse portion of reflection ==== //
    // D = NormalDist(NdH, roughness), 1/pi omitted; G = HammonSmith(NdV, NdL, roughness), 4 * NdV * NdL omitted
    float3 f_ct = F * NormalDist(NdH, roughness) * HammonSmith(NdV, NdL, roughness);
    light_reflected_ratio += f_ct * contrib_weight * NdL;

    return clamp(light_reflected_ratio, make_float3(0.f), make_float3(1.f));
}

// Calculate responsed color of surface reflection toward light sources
static __device__ __inline__ float3 GetLightReflectedColor(PerRayData_camera* prd_camera,
                                                           const int& num_blended_materials,
                                                           unsigned int& material_id,
                                                           const float2& uv,
                                                           const float3& hit_point,
                                                           const float3& world_normal,
                                                           const float3& ray_dir,
                                                           const float& NdV) {
    
    float3 light_reflected_color = make_float3(0.0f);
    // Iterate over all lights
    for (int i = 0; i < params.num_lights; i++) {
        Light l = params.lights[i];
        LightSample ls;
        ls.hitpoint = hit_point;
        ls.wo = -ray_dir;
        ls.n = world_normal;
        SampleLight(l, &ls);
        if (ls.pdf > 0 && fmaxf(ls.L) > 0) {
            float NdL = Dot(world_normal, ls.dir);
            // if we think we can see the light, let's see if we are correct
            if (NdL > 0.0f) {
                // Check shadows
                PerRayData_shadow prd_shadow = DefaultShadowPRD();
                prd_shadow.depth = prd_camera->depth + 1;
                prd_shadow.ramaining_dist = ls.dist;
                unsigned int opt1;
                unsigned int opt2;
                unsigned int raytype = static_cast<unsigned int>(RayType::SHADOW_RAY_TYPE);
                pointer_as_ints(&prd_shadow, opt1, opt2);
                optixTrace(
                    params.root, hit_point, ls.dir, params.scene_epsilon, ls.dist, optixGetRayTime(),
                    OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype
                );

                float3 in_light_color = ls.L * prd_shadow.attenuation * NdL;
                if (fmaxf(in_light_color) > 0.0f) {
                    float3 halfway = normalize(ls.dir - ray_dir);
                    float NdH = Dot(world_normal, halfway);
                    float VdH = Dot(-ray_dir, halfway); // dot(V, H) = dot(H, L), since H is halfway between L and V

                    for (int b = 0; b < num_blended_materials; b++) {
                        const MaterialParameters& mat = params.material_pool[material_id + b];
                        light_reflected_color += in_light_color * prd_camera->contrib_to_pixel
                                                 * PrincipledBRDF(mat, uv, num_blended_materials, NdV, NdL, NdH, VdH);
                    }
                }
            }
        }
    }

    // for(int i = 0; i < params.num_lights; i++){
    //     Light light = params.lights[i];
    //     if (light.type != LightType::AREA_LIGHT)
    //         continue;
    //     AreaLight& a = static_cast<AreaLight&>(light);
    //
    //    float dist_to_light = Length(a.pos - hit_point);
    //    float3 normal = CrossProduct(a.du, a.dv);
    //
    //    if (dist_to_light < 2 * a.max_range) {
    //
    //        float3 sampleColor = make_float3(0.0f);
    //
    //        for(int lightSampleID = 0; lightSampleID < 5; lightSampleID++){
    //
    //            float3 tempPos = a.pos + (curand_uniform(&prd_camera->rng)*a.du)
    //                            + (curand_uniform(&prd_camera->rng)*a.dv);
    //
    //            float3 dir_to_light = normalize(tempPos - hit_point);
    //            float NdL = Dot(world_normal, dir_to_light);
    //
    //            // Dot product of normal of area light and direction to light
    //            // float AdL = Dot(a.normal, dir_to_light);
    //
    //            // Checking to see if we can hit light rays towards the source and the orientation of the area
    //            light
    //            // Allows the light ray to hit light-emitting surface part of area light
    //
    //            if (NdL > 0.0f) {
    //                // check shadows
    //                PerRayData_shadow prd_shadow = DefaultShadowPRD();
    //                prd_shadow.depth = prd_camera->depth + 1;
    //                prd_shadow.ramaining_dist = dist_to_light;
    //                unsigned int opt1;
    //                unsigned int opt2;
    //                pointer_as_ints(&prd_shadow, opt1, opt2);
    //                unsigned int raytype = (unsigned int)RayType::SHADOW_RAY_TYPE;
    //
    //                // TODO: Re-implement this multiple times with slightly different dir_to_light values to
    //                improve data sampling
    //
    //                optixTrace(params.root, hit_point, dir_to_light, params.scene_epsilon, dist_to_light,
    //                        optixGetRayTime(), OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2,
    //                        raytype);
    //
    //                float3 light_attenuation = prd_shadow.attenuation;
    //
    //                float point_light_falloff = (a.max_range * a.max_range / (dist_to_light * dist_to_light +
    //                a.max_range * a.max_range)); float3 incoming_light_ray = a.color * light_attenuation *
    //                point_light_falloff * NdL;
    //
    //                if (fmaxf(incoming_light_ray) > 0.0f) {
    //
    //                    float3 halfway = normalize(dir_to_light - ray_dir);
    //                    float NdV = Dot(world_normal, -ray_dir);
    //                    float NdH = Dot(world_normal, halfway);
    //                    float VdH = Dot(-ray_dir, halfway);
    //
    //                    for (int b = 0; b < num_blended_materials; b++) {
    //                        const MaterialParameters& mat = params.material_pool[material_id + b];
    //                        float3 subsurface_albedo = mat.Kd;
    //
    //                        if (mat.kd_tex) {
    //                            const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x,
    //                            uv.y*mat.tex_scale.y);
    //                            // transfer sRGB texture into linear color space.
    //                            subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
    //                        }
    //
    //                        float roughness = mat.roughness;
    //                        if (mat.roughness_tex) {
    //                            roughness = tex2D<float>(mat.roughness_tex, uv.x*mat.tex_scale.x,
    //                            uv.y*mat.tex_scale.y);
    //                        }
    //                        float metallic = mat.metallic;
    //                        if (mat.metallic_tex) {
    //                            metallic = tex2D<float>(mat.metallic_tex, uv.x*mat.tex_scale.x,
    //                            uv.y*mat.tex_scale.y);
    //                        }
    //                        float transparency = mat.transparency;
    //                        if (mat.opacity_tex) {  // override value with a texture if available
    //                            transparency = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x,
    //                            uv.y*mat.tex_scale.y);
    //                        }
    //                        float mat_blend_weight = 1.f / num_blended_materials;
    //                        if (mat.weight_tex) {  // override blending with weight texture if available
    //                            mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
    //                        }
    //
    //                        float3 F = make_float3(0.0f);
    //                        // float3 subsurface_albedo_updated = subsurface_albedo;
    //                        // === dielectric workflow
    //                        if (mat.use_specular_workflow) {
    //                            float3 specular = mat.Ks;
    //                            if (mat.ks_tex) {
    //                                const float4 tex = tex2D<float4>(mat.ks_tex,uv.x*mat.tex_scale.x,
    //                                uv.y*mat.tex_scale.y); specular = make_float3(tex.x, tex.y, tex.z);
    //                            }
    //                            float3 F0 = specular * 0.08f;
    //                            F = fresnel_schlick(VdH, 5.f, F0,
    //                                                make_float3(1.f) /*make_float3(fresnel_max) it is usually
    //                                                1*/);
    //                        } else {
    //                            float3 default_dielectrics_F0 = make_float3(0.04f);
    //                            F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
    //                            subsurface_albedo = subsurface_albedo *
    //                                                (1 - metallic);  // since imetals do not do subsurface
    //                                                reflection
    //                        }
    //
    //                        // Diffuse portion of reflection
    //                        float3 contrib_weight =
    //                            prd_camera->contrib_to_pixel * transparency *
    //                            mat_blend_weight;  // correct for transparency, light bounces, and blend weight
    //                        sampleColor +=
    //                            ((make_float3(1.f) - F) * subsurface_albedo * incoming_light_ray) *
    //                            contrib_weight;
    //                        float D = NormalDist(NdH, roughness);        // 1/pi omitted
    //                        float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted
    //
    //                        float3 f_ct = F * D * G;
    //                        sampleColor += f_ct * incoming_light_ray * contrib_weight;
    //                    }
    //                }
    //            }
    //        }
    //
    //        sampleColor = sampleColor / 5;
    //        light_reflected_color += sampleColor;
    //    }
    //}

    return light_reflected_color;
}

// Calculate Ambient Light response
static __device__ __inline__ float3 GetAmbientColor(PerRayData_camera* prd_camera,
                                                          const int& num_blended_materials,
                                                          unsigned int& material_id,
                                                          const float2& uv,
                                                          const float3& world_normal,
                                                          const float3& ray_dir) {
    float3 ambient_color = make_float3(0.0f);
    if (!prd_camera->use_gi) {
        float NdV = Dot(world_normal, -ray_dir);
        for (int b = 0; b < num_blended_materials; b++) {
            const MaterialParameters& mat = params.material_pool[material_id + b];
            
            // Get model parameter values, with using textures' values if textures exist
            float3 albedo = (mat.kd_tex) ? Pow(GetTexFloat4ValFloat3(mat.kd_tex, mat.tex_scale, uv), 2.2) : mat.Kd; // transfer texture from sRGB space into linear color space
            float roughness = (mat.roughness_tex) ? GetTexValFloat(mat.roughness_tex, mat.tex_scale, uv): mat.roughness;
            float metallic = (mat.metallic_tex) ? GetTexValFloat(mat.metallic_tex, mat.tex_scale, uv) : mat.metallic;
            float opacity = (mat.opacity_tex) ? GetTexValFloat(mat.opacity_tex, mat.tex_scale, uv) : mat.transparency;
            float mat_blend_weight = (mat.weight_tex) ? GetTexValFloat(mat.weight_tex, mat.tex_scale, uv) : (1.f / num_blended_materials);
            
            // Correct for opacity, light bounces, and blend weight
            float3 contrib_weight = prd_camera->contrib_to_pixel * opacity * mat_blend_weight;  

            // ambient light model is partial "flashlight" ambient light, partially from normal direction
            ambient_color += params.ambient_light_color *
                             (make_float3(NdV) + make_float3(Dot(world_normal, make_float3(0, 0, 1)) * .5f + .5f)) *
                             albedo * contrib_weight;
        }
    }

    return ambient_color;
}


// Calculate surface reflection toward the specular reflection direction. Do this reflection regardless of GI on or off.
static __device__ __inline__ float3 GetSpecularReflectedColor(const ContextParameters& cntxt_params,
                                                              PerRayData_camera* prd_camera,
                                                              const int& num_blended_materials,
                                                              unsigned int& material_id,
                                                              const float2& uv,
                                                              const float3& world_normal,
                                                              const float3& ray_dir,
                                                              const float3& hit_point,
                                                              const float3& NdV) {
    float3 next_contrib_to_pixel = make_float3(0.f);
    float3 next_dir = normalize(reflect(ray_dir, world_normal));
    {
        float NdL = Dot(world_normal, next_dir);
        float3 halfway = normalize(next_dir - ray_dir);
        float NdH = Dot(world_normal, halfway);
        float VdH = Dot(-ray_dir, halfway);  // Same as LdH

        for (int b = 0; b < num_blended_materials; b++) {
            const MaterialParameters& mat = params.material_pool[material_id + b];
            
            // Get BRDF model parameter values, with using textures' values if textures exist
            float3 albedo = (mat.kd_tex) ? Pow(GetTexFloat4ValFloat3(mat.kd_tex, mat.tex_scale, uv), 2.2) : mat.Kd; // transfer texture from sRGB space into linear color space
            float roughness = (mat.roughness_tex) ? GetTexValFloat(mat.roughness_tex, mat.tex_scale, uv): mat.roughness;
            float metallic = (mat.metallic_tex) ? GetTexValFloat(mat.metallic_tex, mat.tex_scale, uv) : mat.metallic;
            float opacity = (mat.opacity_tex) ? GetTexValFloat(mat.opacity_tex, mat.tex_scale, uv) : mat.transparency;
            float mat_blend_weight = (mat.weight_tex) ? GetTexValFloat(mat.weight_tex, mat.tex_scale, uv) : (1.f / num_materials);

            float3 F = make_float3(0.0f);
            // === dielectric workflow
            if (mat.use_specular_workflow) {
                float3 specular = (mat.ks_tex) ? GetTexFloat4ValFloat3(mat.ks_tex, mat.tex_scale, uv) : mat.Ks;
                float3 F0 = specular * 0.08f;
                F = fresnel_schlick(VdH, 5.f, F0, make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
            } else {
                float3 default_dielectrics_F0 = make_float3(0.04f);
                F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
            }

            float D = NormalDist(NdH, roughness);        // 1/pi omitted
            float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted

            float3 f_ct = F * D * G;

            // Note only specular part appears here. Energy preserve
            // Since it is not random, PDF is 1 (normally 1/pi),
            // If the camera uses GI, then it will trace two rays. So each ray's contribution should be halfed

            // corrected for opacity, bounce contribution, and blend
            float weight = opacity * mat_blend_weight;

            // mirror correction accounts for us oversampling this direction
            // following line comes from a heuristic. Perect reflection for metalic smooth objects,
            // no reflection for rough non-metalic objects
            float mirror_correction = (1.f - roughness) * (1.f - roughness) * metallic * metallic;

            // if global illumination, ray contrib will be halved since two rays are propogated
            if (prd_camera->use_gi) {
                weight = weight * .5f;
            }

            float3 partial_contrib = mirror_correction * weight * f_ct * NdL / (4 * CUDART_PI_F);
            partial_contrib = clamp(partial_contrib, make_float3(0), make_float3(1));

            partial_contrib = partial_contrib * prd_camera->contrib_to_pixel;

            next_contrib_to_pixel += partial_contrib;
            next_contrib_to_pixel = clamp(next_contrib_to_pixel, make_float3(0), make_float3(1));
        }
    }

    float3 specular_reflection_color = make_float3(0.0);
    {
        if (luminance(next_contrib_to_pixel) > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
            // Launch the next specular reflected ray
            PerRayData_camera prd_reflection = DefaultCameraPRD();
            prd_reflection.integrator = prd_camera->integrator;
            prd_reflection.contrib_to_pixel = next_contrib_to_pixel;
            prd_reflection.rng = prd_camera->rng;
            prd_reflection.depth = prd_camera->depth + 1;
            prd_reflection.use_gi = prd_camera->use_gi;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_reflection, opt1, opt2);
            unsigned int raytype = static_cast<unsigned int>(RayType::CAMERA_RAY_TYPE);
            optixTrace(
                cntxt_params.root,          // The scene traversable handle (OptixTraversableHandle); basically the top-level acceleration structure (TLAS).
                hit_point,                  // origin of the traced ray
                next_dir,                    // direction of the traced ray
                cntxt_params.scene_epsilon, // minimum intersection distance to avoid self-intersection (“shadow acne”)
                1e16f,                      // A very large max distance (effectively “infinite” for the scene scale)
                optixGetRayTime(),          // time value for launching this ray
                OptixVisibilityMask(1),     // Only intersects geometry whose instance mask matches 1
                OPTIX_RAY_FLAG_NONE,        // No special flags (e.g., no disable-anyhit, no terminate-on-first-hit, etc.)
                0,                          // SBT offset (used when you have multiple SBT records for the same ray type). It selects the first “ray type slot”
                1,                          // SBT stride (used when you have multiple SBT records for the same ray type). It usually means “one ray type stride”
                0,                          // missSBTIndex. It selects the first miss program
                opt1,                       // Final payloads; the per-ray data pointer (first 32 bits); optixGetPayload_0() = opt1
                opt2,                       // Final payloads; the per-ray data pointer (second 32 bits); optixGetPayload_1() = opt2
                raytype                     // The ray type index (used when you have multiple ray types, e.g., radiance rays, shadow rays, etc.)
            );

            specular_reflection_color = prd_reflection.color;
        }
    }

    return specular_reflection_color;
}

//==========================
// Indirect illumination ray
//==========================
static __device__ __inline__ float3 GetDiffuseReflectedColor(PerRayData_camera* prd_camera,
                                                               const int& num_blended_materials,
                                                               unsigned int& material_id,
                                                               const float2& uv,
                                                               const float3& world_normal,
                                                               const float3& ray_dir,
                                                               const float3& hit_point,
                                                               const float3& mirror_reflection_color) {
    float NdV = Dot(world_normal, -ray_dir);
    float3 diffuse_reflected_color = make_float3(0);

    if (prd_camera->use_gi) {
        // Sample hemisphere for next ray when using global illumination
        float z1 = curand_uniform(&prd_camera->rng);
        float z2 = curand_uniform(&prd_camera->rng);
        float3 next_dir = sample_cosine_hemisphere_dir(z1, z2, world_normal);

        float NdL = Dot(world_normal, next_dir);
        float3 halfway = normalize(next_dir - ray_dir);
        float NdH = Dot(world_normal, halfway);
        float VdH = Dot(-ray_dir, halfway);  // Same as LdH

        float3 next_contrib_to_pixel = make_float3(0.f);

        for (int b = 0; b < num_blended_materials; b++) {
            const MaterialParameters& mat = params.material_pool[material_id + b];
            next_contrib_to_pixel += prd_camera->contrib_to_pixel * PrincipledBRDF(mat, uv, num_blended_materials, NdV, NdL, NdH, VdH);
        }

        // If mirror_reflection, then it will trace two rays. So each ray's contribution should be halfed
        if (prd_camera->use_gi) {
        // if ((mirror_reflection_color.x < 1e-6) && (mirror_reflection_color.y < 1e-6) && (mirror_reflection_color.z < 1e-6)) {
            next_contrib_to_pixel = next_contrib_to_pixel * .5f;
        }

        if (luminance(next_contrib_to_pixel) > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
            PerRayData_camera prd_reflection = DefaultCameraPRD();
            prd_reflection.integrator = prd_camera->integrator;
            prd_reflection.contrib_to_pixel = next_contrib_to_pixel;
            prd_reflection.rng = prd_camera->rng;
            prd_reflection.depth = prd_camera->depth + 1;
            prd_reflection.use_gi = prd_camera->use_gi;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_reflection, opt1, opt2);
            unsigned int raytype = static_cast<unsigned int>(RayType::CAMERA_RAY_TYPE);
            optixTrace(params.root, hit_point, next_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
            diffuse_reflected_color = prd_reflection.color;  // accumulate indirect lighting color
        }
    }

    return diffuse_reflected_color;
}

// Account for Fog effect
static __device__ __inline__ void AddFogEffect(
    PerRayData_camera* prd_camera, const ContextParameters& cntxt_params, const float& ray_dist
) {
    if (prd_camera->use_fog && cntxt_params.fog_scattering > 0.f) {
        float blend_alpha = expf(-cntxt_params.fog_scattering * ray_dist);
        prd_camera->color = blend_alpha * prd_camera->color + (1 - blend_alpha) * cntxt_params.fog_color;
    }
}

// Legacy camera shader
static __device__ __inline__ void CameraLegacyShader(const ContextParameters& cntxt_params,
                                                     PerRayData_camera* prd_camera,
                                                     const MaterialRecordParameters* mat_params,
                                                     unsigned int& material_id,
                                                     const unsigned int& num_blended_materials,
                                                     const float3& world_normal,
                                                     const float2& uv,
                                                     const float3& tangent,
                                                     const float& ray_dist,   // ray distance from previous to current hit points
                                                     const float3& hit_point, // current hit point
                                                     const float3& ray_dir) {
    // printf("MS| d: %d | contr: (%f,%f,%f)\n", prd_camera->depth, prd_camera->contrib_to_pixel.x, prd_camera->contrib_to_pixel.y, prd_camera->contrib_to_pixel.z);

    float NdV = Dot(world_normal, -ray_dir);

    // If not blended materials, check for transparent cards and short circuit on the transparent texture
    const MaterialParameters& mat = cntxt_params.material_pool[material_id];
    if (num_blended_materials == 1) {
        
        //// Figure out tranparency
        float opacity = mat.transparency;
        // Opacity texture exists
        if (mat.opacity_tex) {
            opacity = tex2D<float>(mat.opacity_tex, uv.x * mat.tex_scale.x, uv.y * mat.tex_scale.y);
        }
        // Diffuse-color texture with alpha channel exists
        else if (mat.kd_tex) {
            const float4 tex = tex2D<float4>(mat.kd_tex, uv.x * mat.tex_scale.x, uv.y * mat.tex_scale.y);
            opacity = tex.w;  // to handle transparent card textures such as tree leaves
        }

        // If this is perfectly transparent, ignore it and trace the next ray (handles things like tree leaf cards)
        if (opacity < 1e-6) {
            // Launch the next refraction ray
            if (prd_camera->depth + 1 < cntxt_params.max_depth) {
                PerRayData_camera prd_refraction = DefaultCameraPRD();
                prd_refraction.integrator = prd_camera->integrator;
                prd_refraction.contrib_to_pixel = prd_camera->contrib_to_pixel;
                prd_refraction.rng = prd_camera->rng;
                prd_refraction.depth = prd_camera->depth + 1;
                
                unsigned int opt1, opt2;
                pointer_as_ints(&prd_refraction, opt1, opt2);
                
                unsigned int raytype = static_cast<unsigned int>(RayType::CAMERA_RAY_TYPE);
                optixTrace(
                    cntxt_params.root,          // The scene traversable handle (OptixTraversableHandle); basically the top-level acceleration structure (TLAS).
                    hit_point,                  // origin of the traced ray
                    ray_dir,                    // direction of the traced ray
                    cntxt_params.scene_epsilon, // minimum intersection distance to avoid self-intersection (“shadow acne”)
                    1e16f,                      // A very large max distance (effectively “infinite” for the scene scale)
                    optixGetRayTime(),          // time value for launching this ray
                    OptixVisibilityMask(1),     // Only intersects geometry whose instance mask matches 1
                    OPTIX_RAY_FLAG_NONE,        // No special flags (e.g., no disable-anyhit, no terminate-on-first-hit, etc.)
                    0,                          // SBT offset (used when you have multiple SBT records for the same ray type). It selects the first “ray type slot”
                    1,                          // SBT stride (used when you have multiple SBT records for the same ray type). It usually means “one ray type stride”
                    0,                          // missSBTIndex. It selects the first miss program
                    opt1,                       // Final payloads; the per-ray data pointer (first 32 bits); optixGetPayload_0() = opt1
                    opt2,                       // Final payloads; the per-ray data pointer (second 32 bits); optixGetPayload_1() = opt2
                    raytype                     // The ray type index (used when you have multiple ray types, e.g., radiance rays, shadow rays, etc.)
                );
                prd_camera->color = prd_refraction.color;
                
                AddFogEffect(prd_camera, cntxt_params, ray_dist);

                // For GI, harmless without GI
                prd_camera->albedo = prd_refraction.albedo;
                prd_camera->normal = prd_refraction.normal;
            }
            return;
        }
    }


    // Get color contributed by direct illumination reflected for each light: traverse to light, and calculate each material's shading
    prd_camera->color = GetLightReflectedColor(prd_camera, num_blended_materials, material_id, uv, hit_point, world_normal, ray_dir, NdV);

    // for each blended material accumulate specular reflection, and perform traversal
    float3 specular_reflected_color = GetSpecularReflectedColor(
        params, prd_camera, num_blended_materials, material_id, uv, world_normal, ray_dir, hit_point, NdV
    );
    prd_camera->color += specular_reflected_color;

    // Note: consider most general case: only one non-transparent materiral, this calculation is generally skipped
    // Get refracted color contributed for each blended material with accumulated opacity, and perform traversal
    if (num_blended_materials > 1 && mat.transparency < 0.99f) {
        prd_camera->color += GetRefractedColor(cntxt_params, prd_camera, num_blended_materials, material_id, uv, hit_point, ray_dir);
    }

    // Send ray in random direction if global illumination enabled, calculate each materia's shading for a combined shading
    if (prd_camera->use_gi) {
        prd_camera->color += GetDiffuseReflectedColor(
            prd_camera, num_blended_materials, material_id, uv, world_normal, ray_dir, hit_point, specular_reflected_color
        );
    }
    // Else add ambient color from all blended materials
    else {
        prd_camera->color += GetAmbientColor(prd_camera, num_blended_materials, material_id, uv, world_normal, ray_dir);;
    }

    // Add emissive color
    prd_camera->color += (mat.emissive_power * mat.Ke * abs(Dot(world_normal, -ray_dir)));

    // Account for fog
    AddFogEffect(prd_camera, cntxt_params, ray_dist);
    
    // printf("Color: (%.2f,%.2f,%.2f)\n", prd_camera->color.x, prd_camera->color.y, prd_camera->color.z);
    
    // Collect albedo and world_normal for OptiX denoiser
    if (prd_camera->depth == 2) {
        float3 accum_albedo = make_float3(0.f);
        for (int b = 0; b < num_blended_materials; b++) {
            const MaterialParameters& mat = cntxt_params.material_pool[material_id + b];
            
            // Acquire albedo and blending-weight
            float3 albedo = (mat.kd_tex) ? Pow(GetTexFloat4ValFloat3(mat.kd_tex, mat.tex_scale, uv), 2.2) : mat.Kd;
            float mat_blend_weight = (mat.weight_tex) ? GetTexValFloat(mat.weight_tex, mat.tex_scale, uv) : (1.f / num_blended_materials); 
            
            accum_albedo += albedo * mat_blend_weight;
        }
        
        prd_camera->albedo = accum_albedo;
        prd_camera->normal = world_normal;
    }
}

static __device__ __inline__ float LambertianBSDFPdf(float3& wo, float3& wi, float3& n) {
    // float WodWi = Dot(wo,wi);
    float NdWi = Dot(n, wi);
    return NdWi > 0 ? NdWi * INV_PI : 0;
}

static __device__ __inline__ void LambertianBSDFSample(BSDFSample& sample,
                                                       const MaterialParameters& mat,
                                                       const float2& uv,
                                                       bool eval,
                                                       float z1,
                                                       float z2) {
    float3 Kd = mat.Kd;
    if (mat.kd_tex) {
        const float4 tex = tex2D<float4>(mat.kd_tex, uv.x * mat.tex_scale.x, uv.y * mat.tex_scale.y);
        // transfer sRGB texture into linear color space.
        Kd = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
        // printf("Querying Texture map| Kd:(%f,%f,%f)\n", Kd.x, Kd.y, Kd.z);
    }
    sample.f = Kd * INV_PI;
    if (eval)
        return;

    sample.wi = sample_cosine_hemisphere_dir(z1, z2, sample.n);
    sample.pdf = LambertianBSDFPdf(sample.wo, sample.wi, sample.n);
}

// Retroreflective BSDF PDF function
static __device__ __inline__ float RetroreflectiveBSDFPdf(float3& wo, float3& wi, float3& n) {
    float WodWi = Dot(wo, wi);
    float NdWi = Dot(n, wi);
    return WodWi > .999f ? NdWi : 0.f;
}

// Retroreflective BSDF Sampling function
static __device__ __inline__ void RetroreflectiveBSDFSample(BSDFSample& sample,
                                                            const MaterialParameters& mat,
                                                            bool eval,
                                                            float z1,
                                                            float z2) {
    // Use Ks as the efficiency factor
    sample.f = mat.Ks;
    if (eval)
        return;

    sample.wi = sample.wo;
    sample.pdf = RetroreflectiveBSDFPdf(sample.wo, sample.wi, sample.n);
}

static __device__ __inline__ BSDFSample SampleBSDF(BSDFType type,
                                                   BSDFSample& sample,
                                                   const MaterialParameters& mat,
                                                   const float2& uv,
                                                   bool eval = false,
                                                   float z1 = 0,
                                                   float z2 = 0) {
    switch (type) {
        case BSDFType::DIFFUSE:
            LambertianBSDFSample(sample, mat, uv, eval, z1, z2);
            break;
        case BSDFType::SPECULAR:
            break;
        case BSDFType::DIELECTRIC:
            break;
        case BSDFType::GLOSSY:
            break;
        case BSDFType::PRINCIPLED:
            break;
        case BSDFType::HAPKE:
            break;
        case BSDFType::RETROREFLECTIVE:
            RetroreflectiveBSDFSample(sample, mat, eval, z1, z2);
            break;
        default:
            break;
    }

    return sample;
}

static __device__ __inline__ float EvalBSDFPDF(BSDFType type, float3& wo, float3& wi, float3& n) {
    float pdf;
    switch (type) {
        case BSDFType::DIFFUSE:
            pdf = LambertianBSDFPdf(wo, wi, n);
            break;
        case BSDFType::SPECULAR:
            break;
        case BSDFType::DIELECTRIC:
            break;
        case BSDFType::GLOSSY:
            break;
        case BSDFType::PRINCIPLED:
            break;
        case BSDFType::HAPKE:
            break;
        case BSDFType::RETROREFLECTIVE:
            pdf = RetroreflectiveBSDFPdf(wo, wi, n);
            break;
        default:
            break;
    }

    return pdf;
}


// Importance Sampling power heurustic method
static __device__ __inline__ float ISPowerHeuristic(int nf, float fPdf, int ng, float gPdf) {
    float f = nf * fPdf, g = ng * gPdf;
    return (f * f) / (f * f + g * g);
}


static __device__ __inline__ float3 ComputeDirectLight(Light& l,
                                                       LightSample& ls,
                                                       const MaterialParameters& mat,
                                                       const float2& uv,
                                                       int depth) {
    float3 Ld = make_float3(0.f);
    SampleLight(l, &ls);
    BSDFType bsdf_type = mat.bsdf_type;
    if (ls.pdf > 0 && fmaxf(ls.L) > 0) {
        float NdL = Dot(ls.n, ls.dir);
        if (NdL > 0) {
            // Evaluate BSDF at light direction
            BSDFSample bsdf;
            bsdf.wi = ls.dir;
            bsdf.wo = ls.wo;
            bsdf.n = ls.n;
            SampleBSDF(bsdf_type, bsdf, mat, uv, true);
            float scatterPDF = EvalBSDFPDF(bsdf_type, bsdf.wo, bsdf.wi, bsdf.n);
            if (!(fmaxf(bsdf.f) > 0))
                return Ld;  // If the BSDF is black, direct light contribution  is 0?
            // Shoot shadow rays
            PerRayData_shadow prd_shadow = DefaultShadowPRD();
            prd_shadow.depth = depth + 1;
            prd_shadow.ramaining_dist = ls.dist;
            unsigned int opt1;
            unsigned int opt2;
            pointer_as_ints(&prd_shadow, opt1, opt2);
            unsigned int raytype = static_cast<unsigned int>(RayType::SHADOW_RAY_TYPE);
            optixTrace(params.root, ls.hitpoint, ls.dir, params.scene_epsilon, ls.dist, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

            // light contribution
            float3 light_contrib = bsdf.f * NdL * (prd_shadow.attenuation);
            // printf("L Contr: (%f,%f,%f)\n", light_contrib.x,light_contrib.y, light_contrib.z);
            if (l.delta) {
                Ld += light_contrib * ls.L / ls.pdf;
            } else {
                float is_weight = ISPowerHeuristic(1, ls.pdf, 1, scatterPDF);
                Ld += light_contrib * ls.L * is_weight / ls.pdf;
            }
        }
    }

    // TODO:: Add MIS computation for non delta lights

    return Ld;
}


static __device__ __inline__ void CameraPathIntegrator(PerRayData_camera* prd_camera,
                                                       const MaterialRecordParameters* mat_params,
                                                       unsigned int& material_id,
                                                       const unsigned int& num_blended_materials,
                                                       const float3& world_normal,
                                                       const float2& uv,
                                                       const float3& tangent,
                                                       const float& ray_dist,
                                                       const float3& ray_orig,
                                                       const float3& ray_dir) {
    // if (prd_camera->depth >= 3)
    //     printf("PI| d: %d | contr: (%f,%f,%f)\n", prd_camera->depth, prd_camera->contrib_to_pixel.x,
    //            prd_camera->contrib_to_pixel.y, prd_camera->contrib_to_pixel.z);
    const MaterialParameters& mat = params.material_pool[material_id];
    float3 Kd = mat.Kd;
    if (mat.kd_tex) {
        const float4 tex = tex2D<float4>(mat.kd_tex, uv.x * mat.tex_scale.x, uv.y * mat.tex_scale.y);
        // transfer sRGB texture into linear color space.
        Kd = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
    }
    BSDFType bsdf_type = mat.bsdf_type;
    float3 hit_point = ray_orig + ray_dir * ray_dist;
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
        //  Uniform sample light
        unsigned int sample_light_index =
            (unsigned int)(curand_uniform(&prd_camera->rng) *
                           params.num_lights);  // TODO: Won't work for whitted as no GI, have a global sampler instead?
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

#endif // SHADER_UTILS_H