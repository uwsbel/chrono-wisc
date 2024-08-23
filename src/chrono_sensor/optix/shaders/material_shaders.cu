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
// RT kernels for material shading
//
// =============================================================================

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
#endif

#include "chrono_sensor/optix/shaders/device_utils.h"

#ifdef USE_SENSOR_NVDB
#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/Ray.h>
#include <nanovdb/util/HDDA.h>
#endif

static __device__ __inline__ float3 CrossProduct(const float3& a, const float3& b){

    return {(a.y * b.z) - (b.y * a.z),
            (a.z * b.x) - (b.z * a.x),
            (a.x * b.y) - (b.x * a.y)};
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
__device__ __inline__ void GetTriangleData(float3& normal,
                                           unsigned int& mat_id,
                                           float2& uv,
                                           float3& tangent,
                                           const unsigned int& mesh_id) {
    const int tri_id = optixGetPrimitiveIndex();
    const float2 bary_coord = optixGetTriangleBarycentrics();

    const MeshParameters& mesh_params = params.mesh_pool[mesh_id];
    const uint4& vertex_idx = mesh_params.vertex_index_buffer[tri_id];

    const float3& v1 = make_float3(mesh_params.vertex_buffer[vertex_idx.x]);
    const float3& v2 = make_float3(mesh_params.vertex_buffer[vertex_idx.y]);
    const float3& v3 = make_float3(mesh_params.vertex_buffer[vertex_idx.z]);

    // calculate normales either from normal buffer or vertex positions
    if (mesh_params.normal_index_buffer &&
        mesh_params.normal_buffer) {  // use vertex normals if normal index buffer exists
        const uint4& normal_idx = mesh_params.normal_index_buffer[tri_id];

        normal = normalize(make_float3(mesh_params.normal_buffer[normal_idx.y]) * bary_coord.x +
                           make_float3(mesh_params.normal_buffer[normal_idx.z]) * bary_coord.y +
                           make_float3(mesh_params.normal_buffer[normal_idx.x]) * (1.0f - bary_coord.x - bary_coord.y));

    } else {  // else use face normals calculated from vertices
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

//=============================
// Calculating Refracted color
//=============================

static __device__ __inline__ float3 CalculateRefractedColor(
    PerRayData_camera* prd_camera,
    const int& num_blended_materials,
    unsigned int& material_id,
    const float2& uv,
    const float3& hit_point,
    const float3& ray_dir){

    float accumulated_transparency = 0.f;
    {
        for (int b = 0; b < num_blended_materials; b++) {
            // accumulate transparency by multiplication
            const MaterialParameters& mat = params.material_pool[material_id + b];
            float mat_opacity = mat.transparency;
            if (mat.opacity_tex) {  // override value with a texture if available
                mat_opacity = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float mat_blend_weight = 1.f / num_blended_materials;
            if (mat.weight_tex) {  // override blending with weight texture if available
                mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
            }
            accumulated_transparency += mat_blend_weight * mat_opacity;
        }
    }

    float3 refracted_color = make_float3(0);
    {
        if (accumulated_transparency < 1.f - 1 / 255.f) {
            float3 refract_importance = prd_camera->contrib_to_pixel * (1 - accumulated_transparency);
            if (fmaxf(refract_importance) > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
                PerRayData_camera prd_refraction = default_camera_prd();
                prd_refraction.integrator = prd_camera->integrator;
                prd_refraction.contrib_to_pixel = refract_importance;
                prd_refraction.rng = prd_camera->rng;
                prd_refraction.depth = prd_camera->depth + 1;
                unsigned int opt1, opt2;
                pointer_as_ints(&prd_refraction, opt1, opt2);

                // make_camera_data(make_float3(0), refract_importance, prd_camera.rnd, prd_camera.depth + 1);
                // float3 refract_dir = refract(optixGetWorldRayDirection(), world_normal, 1.f, 1.f);
                float3 refract_dir = ray_dir;  // pure transparency without refraction
                unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
                optixTrace(params.root, hit_point, refract_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                           OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
                refracted_color = prd_refraction.color;  // TODO: not sure added here or not
            }
        }
    }

    return refracted_color;
}

//=====================================================
// Calculating Surface reflection toward light sources
//=====================================================

static __device__ __inline__ float3 CalculateReflectedColor(
    PerRayData_camera* prd_camera,
    const int& num_blended_materials,
    unsigned int& material_id,
    const float2& uv,
    const float3& hit_point,
    const float3& world_normal,
    const float3& ray_dir){

    float NdV = Dot(world_normal, -ray_dir);
    float3 light_reflected_color = make_float3(0.0f);
    {
        // iterate through the lights
        for (int i = 0; i < params.num_lights; i++) {
            Light light = params.lights[i];
            if (light.type != LightType::POINT_LIGHT)
                continue;
            PointLight& l = static_cast<PointLight&>(light);//params.lights[i];
            float dist_to_light = Length(l.pos - hit_point);
            if (dist_to_light < 2 * l.max_range) {
                
                float3 dir_to_light = normalize(l.pos - hit_point);
                float NdL = Dot(world_normal, dir_to_light);
                

                // if we think we can see the light, let's see if we are correct
                if (NdL > 0.0f) {
                    // check shadows
                    PerRayData_shadow prd_shadow = default_shadow_prd();
                    prd_shadow.depth = prd_camera->depth + 1;
                    prd_shadow.ramaining_dist = dist_to_light;
                    unsigned int opt1;
                    unsigned int opt2;
                    pointer_as_ints(&prd_shadow, opt1, opt2);
                    unsigned int raytype = (unsigned int)SHADOW_RAY_TYPE;
                    optixTrace(params.root, hit_point, dir_to_light, params.scene_epsilon, dist_to_light,
                               optixGetRayTime(), OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2,
                               raytype);

                    float3 light_attenuation = prd_shadow.attenuation;

                    float point_light_falloff =
                        (l.max_range * l.max_range / (dist_to_light * dist_to_light + l.max_range * l.max_range));

                    float3 incoming_light_ray = l.color * light_attenuation * point_light_falloff * NdL;

                    if (fmaxf(incoming_light_ray) > 0.0f) {
                        float3 halfway = normalize(dir_to_light - ray_dir);
                        float NdV = Dot(world_normal, -ray_dir);
                        float NdH = Dot(world_normal, halfway);
                        float VdH = Dot(-ray_dir, halfway);

                        for (int b = 0; b < num_blended_materials; b++) {
                            const MaterialParameters& mat = params.material_pool[material_id + b];
                            float3 subsurface_albedo = mat.Kd;
                            if (mat.kd_tex) {
                                const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                // transfer sRGB texture into linear color space.
                                subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
                            }
                            float roughness = mat.roughness;
                            if (mat.roughness_tex) {
                                roughness = tex2D<float>(mat.roughness_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                            }
                            float metallic = mat.metallic;
                            if (mat.metallic_tex) {
                                metallic = tex2D<float>(mat.metallic_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                            }
                            float transparency = mat.transparency;
                            if (mat.opacity_tex) {  // override value with a texture if available
                                transparency = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                            }
                            float mat_blend_weight = 1.f / num_blended_materials;
                            if (mat.weight_tex) {  // override blending with weight texture if available
                                mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
                            }

                            float3 F = make_float3(0.0f);
                            // float3 subsurface_albedo_updated = subsurface_albedo;
                            // === dielectric workflow
                            if (mat.use_specular_workflow) {
                                float3 specular = mat.Ks;
                                if (mat.ks_tex) {
                                    const float4 tex = tex2D<float4>(mat.ks_tex,uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                    specular = make_float3(tex.x, tex.y, tex.z);
                                }
                                float3 F0 = specular * 0.08f;
                                F = fresnel_schlick(VdH, 5.f, F0,
                                                    make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
                            } else {
                                float3 default_dielectrics_F0 = make_float3(0.04f);
                                F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
                                subsurface_albedo = subsurface_albedo *
                                                    (1 - metallic);  // since imetals do not do subsurface reflection
                            }

                            // Diffuse portion of reflection
                            float3 contrib_weight =
                                prd_camera->contrib_to_pixel * transparency *
                                mat_blend_weight;  // correct for transparency, light bounces, and blend weight
                            light_reflected_color +=
                                ((make_float3(1.f) - F) * subsurface_albedo * incoming_light_ray) * contrib_weight;
                            float D = NormalDist(NdH, roughness);        // 1/pi omitted
                            float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted
                            float3 f_ct = F * D * G;
                            light_reflected_color += f_ct * incoming_light_ray * contrib_weight;
                        }
                    }
                }
            }
        }

        for(int i = 0; i < params.num_lights; i++){
            Light light = params.lights[i];
            if (light.type != LightType::AREA_LIGHT)
                continue;
            AreaLight& a = static_cast<AreaLight&>(light);  

            float dist_to_light = Length(a.pos - hit_point); 
            float3 normal = CrossProduct(a.du, a.dv);
            
            if (dist_to_light < 2 * a.max_range) {
                
                float3 sampleColor = make_float3(0.0f);

                for(int lightSampleID = 0; lightSampleID < 5; lightSampleID++){
                    
                    float3 tempPos = a.pos + (curand_uniform(&prd_camera->rng)*a.du)
                                    + (curand_uniform(&prd_camera->rng)*a.dv);
                    
                    float3 dir_to_light = normalize(tempPos - hit_point);
                    float NdL = Dot(world_normal, dir_to_light);

                    // Dot product of normal of area light and direction to light
                    // float AdL = Dot(a.normal, dir_to_light);

                    // Checking to see if we can hit light rays towards the source and the orientation of the area light
                    // Allows the light ray to hit light-emitting surface part of area light
                    
                    if (NdL > 0.0f) {
                        // check shadows
                        PerRayData_shadow prd_shadow = default_shadow_prd();
                        prd_shadow.depth = prd_camera->depth + 1;
                        prd_shadow.ramaining_dist = dist_to_light;
                        unsigned int opt1;
                        unsigned int opt2;
                        pointer_as_ints(&prd_shadow, opt1, opt2);
                        unsigned int raytype = (unsigned int)SHADOW_RAY_TYPE;

                        // TODO: Re-implement this multiple times with slightly different dir_to_light values to improve data sampling

                        optixTrace(params.root, hit_point, dir_to_light, params.scene_epsilon, dist_to_light,
                                optixGetRayTime(), OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2,
                                raytype);

                        float3 light_attenuation = prd_shadow.attenuation;

                        float point_light_falloff = (a.max_range * a.max_range / (dist_to_light * dist_to_light + a.max_range * a.max_range));
                        float3 incoming_light_ray = a.color * light_attenuation * point_light_falloff * NdL;

                        if (fmaxf(incoming_light_ray) > 0.0f) {

                            float3 halfway = normalize(dir_to_light - ray_dir);
                            float NdV = Dot(world_normal, -ray_dir);
                            float NdH = Dot(world_normal, halfway);
                            float VdH = Dot(-ray_dir, halfway);

                            for (int b = 0; b < num_blended_materials; b++) {
                                const MaterialParameters& mat = params.material_pool[material_id + b];
                                float3 subsurface_albedo = mat.Kd;
                                
                                if (mat.kd_tex) {
                                    const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                    // transfer sRGB texture into linear color space.
                                    subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
                                }

                                float roughness = mat.roughness;
                                if (mat.roughness_tex) {
                                    roughness = tex2D<float>(mat.roughness_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                }
                                float metallic = mat.metallic;
                                if (mat.metallic_tex) {
                                    metallic = tex2D<float>(mat.metallic_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                }
                                float transparency = mat.transparency;
                                if (mat.opacity_tex) {  // override value with a texture if available
                                    transparency = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                }
                                float mat_blend_weight = 1.f / num_blended_materials;
                                if (mat.weight_tex) {  // override blending with weight texture if available
                                    mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
                                }

                                float3 F = make_float3(0.0f);
                                // float3 subsurface_albedo_updated = subsurface_albedo;
                                // === dielectric workflow
                                if (mat.use_specular_workflow) {
                                    float3 specular = mat.Ks;
                                    if (mat.ks_tex) {
                                        const float4 tex = tex2D<float4>(mat.ks_tex,uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                        specular = make_float3(tex.x, tex.y, tex.z);
                                    }
                                    float3 F0 = specular * 0.08f;
                                    F = fresnel_schlick(VdH, 5.f, F0,
                                                        make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
                                } else {
                                    float3 default_dielectrics_F0 = make_float3(0.04f);
                                    F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
                                    subsurface_albedo = subsurface_albedo *
                                                        (1 - metallic);  // since imetals do not do subsurface reflection
                                }

                                // Diffuse portion of reflection
                                float3 contrib_weight =
                                    prd_camera->contrib_to_pixel * transparency *
                                    mat_blend_weight;  // correct for transparency, light bounces, and blend weight
                                sampleColor +=
                                    ((make_float3(1.f) - F) * subsurface_albedo * incoming_light_ray) * contrib_weight;
                                float D = NormalDist(NdH, roughness);        // 1/pi omitted
                                float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted

                                float3 f_ct = F * D * G;
                                sampleColor += f_ct * incoming_light_ray * contrib_weight;
                            }
                        }
                    }
                }

                sampleColor = sampleColor / 5;
                light_reflected_color += sampleColor;
            }
        }
    }

    return light_reflected_color;
}

//===========================
// Calculating Ambient Light
//===========================

static __device__ __inline__ float3 CalculateAmbientLight(
    PerRayData_camera* prd_camera,
    const int& num_blended_materials,
    unsigned int& material_id,
    const float2& uv,
    const float3& world_normal,
    const float3& ray_dir){

    float3 ambient_light = make_float3(0.0f);
    {
        if (!prd_camera->use_gi) {
            float NdV = Dot(world_normal, -ray_dir);
            for (int b = 0; b < num_blended_materials; b++) {
                const MaterialParameters& mat = params.material_pool[material_id + b];
                float3 subsurface_albedo = mat.Kd;
                if (mat.kd_tex) {
                    const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                    // transfer sRGB texture into linear color space.
                    subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
                }
                float transparency = mat.transparency;
                if (mat.opacity_tex) {  // override value with a texture if available
                    transparency = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                }
                float mat_blend_weight = 1.f / num_blended_materials;
                if (mat.weight_tex) {  // override blending with weight texture if available
                    mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
                }

                float3 contrib_weight = prd_camera->contrib_to_pixel * transparency *
                                       mat_blend_weight;  // correct for transparency, light bounces, and blend weight

                // ambient light model is partial "flashlight" ambient light, partially from normal direction
                ambient_light += params.ambient_light_color *
                                 (make_float3(NdV) + make_float3(Dot(world_normal, make_float3(0, 0, 1)) * .5f + .5f)) *
                                 subsurface_albedo * contrib_weight;
            }
        }
    }

    return ambient_light;
}

//===============================================================
// If the surface is very smoooth, trace the reflected direction
// Do this reflection regardless of GI on or off.
//===============================================================

static __device__ __inline__ float3 CalculateContributionToPixel(
    PerRayData_camera* prd_camera,
    const int& num_blended_materials,
    unsigned int& material_id,
    const float2& uv,
    const float3& world_normal,
    const float3& ray_dir,
    const float3& hit_point){
    
    float NdV = Dot(world_normal, -ray_dir);
    float3 next_contrib_to_pixel = make_float3(0.f);
    float3 next_dir = normalize(reflect(ray_dir, world_normal));
    {
        float NdL = Dot(world_normal, next_dir);
        float3 halfway = normalize(next_dir - ray_dir);
        float NdH = Dot(world_normal, halfway);
        float VdH = Dot(-ray_dir, halfway);  // Same as LdH

        for (int b = 0; b < num_blended_materials; b++) {
            const MaterialParameters& mat = params.material_pool[material_id + b];
            float3 subsurface_albedo = mat.Kd;
            if (mat.kd_tex) {
                const float4 tex = tex2D<float4>(mat.kd_tex, uv.x, uv.y);
                // transfer sRGB texture into linear color space.
                subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
            }
            float roughness = mat.roughness;
            if (mat.roughness_tex) {
                roughness = tex2D<float>(mat.roughness_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float metallic = mat.metallic;
            if (mat.metallic_tex) {
                metallic = tex2D<float>(mat.metallic_tex,uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float transparency = mat.transparency;
            if (mat.opacity_tex) {  // override value with a texture if available
                transparency = tex2D<float>(mat.opacity_tex,uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float mat_blend_weight = 1.f / num_blended_materials;
            if (mat.weight_tex) {  // override blending with weight texture if available
                mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
            }

            float3 F = make_float3(0.0f);
            // === dielectric workflow
            if (mat.use_specular_workflow) {
                float3 specular = mat.Ks;
                if (mat.ks_tex) {
                    const float4 tex = tex2D<float4>(mat.ks_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                    specular = make_float3(tex.x, tex.y, tex.z);
                }
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

            // corrected for transparency, bounce contribution, and blend
            float weight = transparency * mat_blend_weight;

            // mirror correction accounts for us oversampling this direction
            // following line comes from a heuristic. Perect reflection for metalic smooth objects,
            // no reflection for rough non-metalic objects
            float mirror_correction = (1.f - roughness) * (1.f - roughness) * metallic * metallic;

            // if global illumination, ray contrib will be halved since two rays are propogated
            if (prd_camera->use_gi) {
                weight = weight*.5f;
            }

            float3 partial_contrib = mirror_correction * weight * f_ct * NdL / (4 * CUDART_PI_F);
            partial_contrib = clamp(partial_contrib, make_float3(0), make_float3(1));

            partial_contrib = partial_contrib * prd_camera->contrib_to_pixel;

            next_contrib_to_pixel += partial_contrib;
            next_contrib_to_pixel = clamp(next_contrib_to_pixel, make_float3(0), make_float3(1));
        }
    }

    float3 mirror_reflection_color = make_float3(0.0);
    {
        if (luminance(next_contrib_to_pixel) > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
            PerRayData_camera prd_reflection = default_camera_prd();
            prd_reflection.integrator = prd_camera->integrator;
            prd_reflection.contrib_to_pixel = next_contrib_to_pixel;
            prd_reflection.rng = prd_camera->rng;
            prd_reflection.depth = prd_camera->depth + 1;
            prd_reflection.use_gi = prd_camera->use_gi;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_reflection, opt1, opt2);
            unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
            optixTrace(params.root, hit_point, next_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

            mirror_reflection_color = prd_reflection.color;
        }
    }

    return mirror_reflection_color;
}

//=================
// Global illumination ray
//=================

static __device__ __inline__ float3 CalculateGIReflectionColor(
    PerRayData_camera* prd_camera,
    const int& num_blended_materials,
    unsigned int& material_id,
    const float2& uv,
    const float3& world_normal,
    const float3& ray_dir,
    const float3& hit_point,
    const float3& mirror_reflection_color){

    float NdV = Dot(world_normal, -ray_dir);
    float3 gi_reflection_color = make_float3(0);

    if (prd_camera->use_gi) {
        // sample hemisphere for next ray when using global illumination
        float z1 = curand_uniform(&prd_camera->rng);
        float z2 = curand_uniform(&prd_camera->rng);
        float3 next_dir = sample_hemisphere_dir(z1, z2, world_normal);

        float NdL = Dot(world_normal, next_dir);
        float3 halfway = normalize(next_dir - ray_dir);

        float NdH = Dot(world_normal, halfway);
        float VdH = Dot(-ray_dir, halfway);  // Same as LdH

        float3 next_contrib_to_pixel = make_float3(0.f);

        for (int b = 0; b < num_blended_materials; b++) {
            const MaterialParameters& mat = params.material_pool[material_id + b];
            float3 subsurface_albedo = mat.Kd;
            if (mat.kd_tex) {
                const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                // transfer sRGB texture into linear color space.
                subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
            }
            float roughness = mat.roughness;
            if (mat.roughness_tex) {
                roughness = tex2D<float>(mat.roughness_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float metallic = mat.metallic;
            if (mat.metallic_tex) {
                metallic = tex2D<float>(mat.metallic_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float transparency = mat.transparency;
            if (mat.opacity_tex) {  // override value with a texture if available
                transparency = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float mat_blend_weight = 1.f / num_blended_materials;
            if (mat.weight_tex) {  // override blending with weight texture if available
                mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
            }

            float3 F = make_float3(0.0f);
            // === dielectric workflow
            if (mat.use_specular_workflow) {
                float3 specular = mat.Ks;
                if (mat.ks_tex) {
                    const float4 tex = tex2D<float4>(mat.ks_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                    specular = make_float3(tex.x, tex.y, tex.z);
                }
                float3 F0 = specular * 0.08f;
                F = fresnel_schlick(VdH, 5.f, F0, make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
            } else {
                float3 default_dielectrics_F0 = make_float3(0.04f);
                F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
                subsurface_albedo = subsurface_albedo*(1 - metallic);  // since metals do not do subsurface reflection
            }

            float D = NormalDist(NdH, roughness);        // 1/pi omitted
            float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted
            float3 f_ct = F * D * G;

            // corrected for transparency, bounce contribution, and blend
            float3 weight = transparency * prd_camera->contrib_to_pixel * mat_blend_weight;

            // If mirror_reflection, then it will trace two rays. So each ray's contribution should be halfed
            if ((mirror_reflection_color.x < 1e-6) && (mirror_reflection_color.y < 1e-6) && (mirror_reflection_color.z < 1e-6)) {
                weight = weight*.5f;
            }

            // Specular part
            next_contrib_to_pixel += weight * f_ct * NdL;

            // Diffuse part
            F = clamp(F, make_float3(0), make_float3(1));
            next_contrib_to_pixel += weight * (make_float3(1.f) - F) * subsurface_albedo * NdL;
        }

        if (luminance(next_contrib_to_pixel) > params.importance_cutoff &&
            prd_camera->depth + 1 < params.max_depth) {
            PerRayData_camera prd_reflection = default_camera_prd();
            prd_reflection.integrator = prd_camera->integrator;
            prd_reflection.contrib_to_pixel = next_contrib_to_pixel;
            prd_reflection.rng = prd_camera->rng;
            prd_reflection.depth = prd_camera->depth + 1;
            prd_reflection.use_gi = prd_camera->use_gi;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_reflection, opt1, opt2);
            unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
            optixTrace(params.root, hit_point, next_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                        OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
            gi_reflection_color = prd_reflection.color;  // accumulate indirect lighting color
        }
    }

    return gi_reflection_color;
}

static __device__ __inline__ void CameraShader(PerRayData_camera* prd_camera,
                                               const MaterialRecordParameters* mat_params,
                                               unsigned int& material_id,
                                               const unsigned int& num_blended_materials,
                                               const float3& world_normal,
                                               const float2& uv,
                                               const float3& tangent,
                                               const float& ray_dist,
                                               const float3& ray_orig,
                                               const float3& ray_dir) {
    //printf("MS| d: %d | contr: (%f,%f,%f)\n", prd_camera->depth, prd_camera->contrib_to_pixel.x,  prd_camera->contrib_to_pixel.y, prd_camera->contrib_to_pixel.z);
    float3 hit_point = ray_orig + ray_dir * ray_dist;

    // if not blended materials, check for transparent cards and short circuit on the transparent texture
    const MaterialParameters& mat = params.material_pool[material_id];
    if (num_blended_materials == 1) {
       

        float transparency = mat.transparency;
        // figure out tranparency
        if (mat.kd_tex) {
            const float4 tex = tex2D<float4>(mat.kd_tex,uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            if (tex.w < 1e-6)
                transparency = 0.f;  // to handle transparent card textures such as tree leaves
        }

        if (mat.opacity_tex) {
            transparency = tex2D<float>(mat.opacity_tex,uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
        }

        // if this is perfectly transparent, we ignore it and trace the next ray (handles things like tree leaf cards)
        if (transparency < 1e-6) {
            if (prd_camera->depth + 1 < params.max_depth) {
                PerRayData_camera prd_refraction = default_camera_prd();
                prd_refraction.integrator = prd_camera->integrator;
                prd_refraction.contrib_to_pixel = prd_camera->contrib_to_pixel;
                prd_refraction.rng = prd_camera->rng;
                prd_refraction.depth = prd_camera->depth + 1;
                unsigned int opt1, opt2;
                pointer_as_ints(&prd_refraction, opt1, opt2);
                unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
                optixTrace(params.root, hit_point, ray_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                           OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
                prd_camera->color = prd_refraction.color;
                //account for fog
                if (prd_camera->use_fog && params.fog_scattering > 0.f) {
                    float blend_alpha = expf(-params.fog_scattering * ray_dist);
                    prd_camera->color = blend_alpha * prd_camera->color + (1 - blend_alpha) * params.fog_color;
                }

                // For GI, harmless without GI
                prd_camera->albedo = prd_refraction.albedo;
                prd_camera->normal = prd_refraction.normal;
            }
            return;
        }
    }

    // for each blended material accumulate transparency, and perform traversal
    float3 refracted_color = CalculateRefractedColor(prd_camera, num_blended_materials, material_id, uv, hit_point, ray_dir);

    // for each light, traverse to light, and calculate each material's shading
    float3 light_reflected_color = CalculateReflectedColor(prd_camera, num_blended_materials, material_id, uv, hit_point, world_normal, ray_dir);

    // for each blended material, calculating total ambient light
    float3 ambient_light = CalculateAmbientLight(prd_camera, num_blended_materials, material_id, uv, world_normal, ray_dir);

    // for each blended material accumulate reflection, and perform traversal    
    float3 next_dir = normalize(reflect(ray_dir, world_normal));
    float3 mirror_reflection_color = CalculateContributionToPixel(prd_camera, num_blended_materials, material_id, uv, world_normal, ray_dir, hit_point);

    // send ray in random direction if global illumination enabled, calculate each materia's shading for a combined shading
    float3 gi_reflection_color = CalculateGIReflectionColor(prd_camera, num_blended_materials, material_id, uv, world_normal, ray_dir, hit_point, mirror_reflection_color);
    
    //=================
    // Combine all traced colors together
    //=================
    prd_camera->color = mirror_reflection_color + light_reflected_color + refracted_color;

    prd_camera->color += prd_camera->use_gi ? gi_reflection_color : ambient_light;

    // Add emissive color
    prd_camera->color += (mat.emissive_power * mat.Ke * abs(Dot(world_normal, -ray_dir)));
    

    // apply fog model
    if (prd_camera->use_fog && params.fog_scattering > 0.f) {
        float blend_alpha = expf(-params.fog_scattering * ray_dist);
        prd_camera->color = blend_alpha * prd_camera->color + (1 - blend_alpha) * params.fog_color;
    }

    //printf("Color: (%.2f,%.2f,%.2f)\n", prd_camera->color.x, prd_camera->color.y, prd_camera->color.z);
    if (prd_camera->depth == 2 && prd_camera->use_gi) {
        float3 accumulated_subsurface_albedo = make_float3(0.f);
        for (int b = 0; b < num_blended_materials; b++) {
            const MaterialParameters& mat = params.material_pool[material_id + b];
            float3 subsurface_albedo = mat.Kd;
            if (mat.kd_tex) {
                const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                // transfer sRGB texture into linear color space.
                subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
            }
            float mat_blend_weight = 1.f / num_blended_materials;
            if (mat.weight_tex) {  // override blending with weight texture if available
                mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
            }
            accumulated_subsurface_albedo += subsurface_albedo * mat_blend_weight;
        }
        prd_camera->albedo = accumulated_subsurface_albedo;
        prd_camera->normal = world_normal;
    }
    
}

static __device__ __inline__ void LidarShader(PerRayData_lidar* prd_lidar,
                                              const MaterialParameters& mat,
                                              const float3& world_normal,
                                              const float2& uv,
                                              const float3& tangent,
                                              const float& ray_dist,
                                              const float3& ray_orig,
                                              const float3& ray_dir) {
    prd_lidar->range = ray_dist;
    prd_lidar->intensity = mat.lidar_intensity * abs(Dot(world_normal, -ray_dir));
}

static __device__ __inline__ void RadarShader(PerRayData_radar* prd_radar,
                                              const MaterialParameters& mat,
                                              const float3& world_normal,
                                              const float2& uv,
                                              const float3& tangent,
                                              const float& ray_dist,
                                              const float3& ray_orig,
                                              const float3& ray_dir,
                                              const float3& translational_velocity,
                                              const float3& angular_velocity,
                                              const float& objectId) {
    prd_radar->range = ray_dist;
    prd_radar->rcs = mat.radar_backscatter * abs(Dot(world_normal, -ray_dir));
    float3 hit_point = ray_orig + ray_dir * ray_dist;
    float3 origin = optixTransformPointFromObjectToWorldSpace(make_float3(0, 0, 0));
    float3 r = hit_point - origin;

    prd_radar->velocity = translational_velocity + Cross(angular_velocity, r);
    prd_radar->objectId = objectId;
}

static __device__ __inline__ void ShadowShader(PerRayData_shadow* prd,
                                               const MaterialParameters& mat,
                                               const float3& world_normal,
                                               const float2& uv,
                                               const float3& tangent,
                                               const float& ray_dist,
                                               const float3& ray_orig,
                                               const float3& ray_dir) {

    float transparency = mat.transparency;
    if (mat.kd_tex) {
        const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
        if (tex.w < 1e-6)
            transparency = 0.f;  // to handle transparent card textures such as tree leaves
    }
    if (mat.opacity_tex) {
        transparency = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
    }
    float3 hit_point = ray_orig + ray_dir * ray_dist;
    //printf("Hit Point SH: (%f,%f,%f)\n", hit_point.x, hit_point.y, hit_point.z);
    float atten = 1.f - transparency;  // TODO: figure out the attenuation from the material transparency

    // if the occlusion amount is below the
    prd->attenuation = prd->attenuation * atten;

    if (fmaxf(prd->attenuation) > params.importance_cutoff && prd->depth + 1 < params.max_depth) {
        PerRayData_shadow prd_shadow = default_shadow_prd();
        prd_shadow.attenuation = prd->attenuation;
        prd_shadow.depth = prd->depth + 1;
        prd_shadow.ramaining_dist = prd->ramaining_dist - ray_dist;
        unsigned int opt1, opt2;
        pointer_as_ints(&prd_shadow, opt1, opt2);

        float3 hit_point = ray_orig + ray_dist * ray_dir;
        unsigned int raytype = (unsigned int)SHADOW_RAY_TYPE;
        optixTrace(params.root, hit_point, ray_dir, params.scene_epsilon, prd_shadow.ramaining_dist, optixGetRayTime(),
                   OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

        prd->attenuation = prd_shadow.attenuation;
    }
}

static __device__ __inline__ void SemanticShader(PerRayData_semantic* prd,
                                                 const MaterialParameters& mat,
                                                 const float3& world_normal,
                                                 const float2& uv,
                                                 const float3& tangent,
                                                 const float& ray_dist,
                                                 const float3& ray_orig,
                                                 const float3& ray_dir) {
    prd->class_id = mat.class_id;
    prd->instance_id = mat.instance_id;
}

static __device__ inline void CameraHapkeShader(PerRayData_camera* prd_camera,
                                               const MaterialRecordParameters* mat_params,
                                               unsigned int& material_id,
                                               const unsigned int& num_blended_materials,
                                               const float3& world_normal,
                                               const float2& uv,
                                               const float3& tangent,
                                               const float& ray_dist,
                                               const float3& ray_orig,
                                               const float3& ray_dir){  

        
        //printf("Distance: %.2f\n", ray_dist);
        //prd_camera->color += make_float3(ray_dist, ray_dist, ray_dist);
        //prd_camera->color += make_float3(1.f,1.f,1.f);
        //return;
        //printf("Hapke Shader!\n");
        // float w = 0.32357f; // average single scattering albedo
        // float b = 0.23955f; // shape controlling parameter for the amplitude of backward and forward scatter of particles
        // float c = 0.30452f; // weighting factor that controls the contribution of backward and forward scatter.
        // float B_s0 = 1.80238f;
        // float h_s = 0.07145f;
        // float B_c0 = 0.0f;
        // float h_c = 1.0f;
        // float phi = 0.3f;
        // //float K = 1.0f;
        // float theta_p = 23.4f*(CUDART_PI_F/180);


       const MaterialParameters& mat = params.material_pool[material_id]; // Assume no blended materials for now
       float3 subsarface_albedo = mat.Kd;
       float3 specular = mat.Ks;

       // Get Hapke material parameters
       float w = mat.w;
       float b = mat.b;
       float c = mat.c;
       float B_s0 = mat.B_s0;
       float h_s = mat.h_s;
       float B_c0 = 0.0f;
       float h_c = 1.0f;
       float phi = mat.phi;
       float theta_p = mat.theta_p;

       

       float3 hit_point = ray_orig + ray_dir * ray_dist;

       float cos_e = Dot(world_normal, -ray_dir);

       float3 reflected_color = make_float3(0.0f);
       {
           for (int i = 0; i < params.num_lights; i++) {
               Light light = params.lights[i];
               if (light.type != LightType::POINT_LIGHT)
                   continue;
               PointLight& l = static_cast<PointLight&>(light);
               float dist_to_light = Length(l.pos - hit_point);
               //printf("dist_to_light:%.4f\n", dist_to_light);
               if (1) {//dist_to_light < 2 * l.max_range{ // Sun should have infinity range, so this condition will always be true for ths sun
                   float3 dir_to_light = normalize(l.pos - hit_point);
                   float cos_i = Dot(dir_to_light, world_normal);
                   //printf("cos_i:%.2f",cos_i);
                   if (cos_i > 0) {

                       // Cast a shadow ray to see any attenuation of light
                       PerRayData_shadow prd_shadow = default_shadow_prd();
                       prd_shadow.depth = prd_camera->depth + 1;
                       prd_shadow.ramaining_dist = dist_to_light;
                       unsigned int opt1;
                       unsigned int opt2;
                       pointer_as_ints(&prd_shadow, opt1, opt2);
                       unsigned int raytype = (unsigned int)SHADOW_RAY_TYPE;
                       optixTrace(params.root, hit_point, dir_to_light, params.scene_epsilon, dist_to_light, optixGetRayTime(),
                               OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

                       float3 light_attenuation = prd_shadow.attenuation;

                       float point_light_falloff  = 1.0f; // ??
                       float3 incoming_light_ray = l.color * cos_i * light_attenuation; // Add attenuation later
                       //printf("incoming_light_ray: (%.2f,%.2f,%.2f)\n", incoming_light_ray.x, incoming_light_ray.y, incoming_light_ray.z);
                       if (fmaxf(incoming_light_ray) > 0.0f) {
                           
                           float cos_g = Dot(dir_to_light, -ray_dir);
                           float sin_i = sqrt(1 - (cos_i*cos_i)); // + sqrt
                           float sin_e = sqrt(1 - (cos_e*cos_e));
                           float sin_g = sqrt(1 - (cos_g*cos_g));

                           float tan_i = sin_i/cos_i;
                           float tan_e = sin_e/cos_e;

                           float cot_i = 1/tan_i;
                           float cot_e = 1/tan_e;
                           float cot_i_sq = cot_i * cot_i;
                           float cot_e_sq = cot_e * cot_e;


                           // Calculate Psi
                           float cos_psi = Dot(normalize(dir_to_light - (cos_i*world_normal)), normalize(-ray_dir - (cos_e*world_normal)));
                           float psi = acos(cos_psi);
                           float psi_half = psi/2;
                           float f_psi = expf(-2 * tan(psi_half));
                           float sin_psi_half = sin(psi_half);
                           float sin_psi_half_sq = sin_psi_half * sin_psi_half;
                           float psi_per_pi = psi/CUDART_PI_F; // TODO: Define 1/PI as a constant

                           float tan_theta_p = tan(theta_p);
                           float tan_theta_p_sq = tan_theta_p * tan_theta_p;

                           float cot_theta_p = 1/tan_theta_p;
                           float cot_theta_p_sq = cot_theta_p * cot_theta_p;

                           float E_1_i = expf((-2/CUDART_PI_F) * cot_theta_p * cot_i);
                           float E_2_i = expf((-1/CUDART_PI_F) * cot_theta_p * cot_theta_p * cot_i * cot_i);

                           float E_1_e = expf((-2/CUDART_PI_F) * cot_theta_p * cot_e);
                           float E_2_e = expf((-1/CUDART_PI_F) * cot_theta_p * cot_theta_p * cot_e * cot_e);

                           float chi_theta_p = 1 / sqrtf(1 + CUDART_PI_F * tan_theta_p_sq);
                           
                           float eta_i = chi_theta_p * (cos_i + sin_i * tan_theta_p * E_2_i / (2 - E_1_i));
                           float eta_e = chi_theta_p * (cos_e + sin_e * tan_theta_p * E_2_e / (2 - E_1_e));

                           float mu0 = cos_i;
                           float mu = cos_e;
                           float mu0_e = chi_theta_p;
                           float mu_e = chi_theta_p;
                           float S = 0.0f;
                           if (cos_i >= cos_g) { // for x,y \in [0,pi], if x <= y => cos(x) >= cos(y)
                               mu0_e *= cos_i + sin_i * tan_theta_p * (cos_psi * E_2_e + sin_psi_half_sq * E_2_i) / (2 - E_1_e - psi_per_pi * E_1_i);

                               mu_e *= cos_e + sin_e * tan_theta_p * (E_2_e - sin_psi_half_sq * E_2_i) / (2 - E_1_e - psi_per_pi * E_1_i);

                               S = mu_e / eta_e * mu0 / eta_i * chi_theta_p / (1 - f_psi + f_psi * chi_theta_p * (mu0/eta_i));
                           }else {
                               mu0_e *= cos_i + sin_i * tan_theta_p * (E_2_i - sin_psi_half_sq * E_2_e) / (2 - E_1_i - psi_per_pi * E_1_e);

                               mu_e *= cos_e + sin_e * tan_theta_p * (cos_psi * E_2_i + sin_psi_half_sq * E_2_e) / (2 - E_1_i - psi_per_pi * E_1_e);

                               S = mu_e / eta_e * mu0 / eta_i * chi_theta_p / (1 - f_psi + f_psi * chi_theta_p * (mu/eta_e));
                           }

                           float KPhi = 1.209 * pow(phi, 2.0f/3);
                           float K = -log(1 - KPhi)/KPhi;

                           float tan_ghalf = sin_g / (1 + cos_g);
                           float tan_ghalf_per_hC = tan_ghalf/h_c;

                           float B_C = 0.0f;
                           if (cos_g < 1.0f)
                               B_C = (1 + (1 - exp(-tan_ghalf_per_hC)) / tan_ghalf_per_hC) / (2 * pow(1 + tan_ghalf_per_hC, 2));
                           else if (cos_g == 1)
                               B_C = 1;
                           
                           float r0Term = sqrt(1 - w);
                           float r0 = (1 - r0Term)/(1 + r0Term);

                           float LS = mu0_e / (mu0_e + mu_e);
                           float b_sq = b*b;

                           float twobcos_g = 2 * b * cos_g;

                           float p_g = (1 + c) / 2 * (1-b_sq) / pow(1 - (2*b*cos_g) + b_sq, 1.5f) + (1 - c)/2 * (1-b_sq)/pow(1 + (2*b*cos_g) + b_sq, 1.5f);

                           float B_S = 1 / (1 + tan_ghalf / h_s);

                           float x_i = mu0_e/K;
                           float x_e = mu_e/K;
                           float H_i = 1/(1 - w * x_i * (r0 + (1 - 2 * r0 * x_i) / 2 * log((1+x_i)/x_i)));
                           float H_e = 1/(1 - w * x_e * (r0 + (1 - 2 * r0 * x_e) / 2 * log((1+x_e)/x_e)));

                           float M = H_i * H_e - 1;
                           float f_ct = LS * K * w/(4*CUDART_PI_F) * (p_g * (1 + B_s0 * B_S) + M) * (1 + B_c0 * B_C) * S /cos_i;
                           //printf("fct:%.2f\n", f_ct);
                           reflected_color += f_ct * incoming_light_ray * subsarface_albedo;
                           //printf("reflected_color:(%.2f,%.2f,%.2f)\n", reflected_color.x, reflected_color.y, reflected_color.z);
                       }

                   }       
               }
           }
       }
     //printf("reflected_color:(%.2f,%.2f,%.2f)\n", reflected_color.x, reflected_color.y, reflected_color.z);
     prd_camera->color += reflected_color;
}

static __device__ __inline__ float SchlickPhase(float VdL, float k) {
    float numerator = 1 - (k * k);
    float denominator = 4 * CUDART_PI * (1 - k * VdL) * (1 - k * VdL);
    return numerator / denominator;
}


static __device__ inline void CameraVolumetricShader(PerRayData_camera* prd_camera,
                                                     const MaterialRecordParameters* mat_params,
                                                     unsigned int& material_id,
                                                     const unsigned int& num_blended_materials,
                                                     const float3& world_normal,
                                                     const float2& uv,
                                                     const float3& tangent,
                                                     const float& ray_dist,
                                                     const float3& ray_orig,
                                                     const float3& ray_dir) {
#ifdef USE_SENSOR_NVDB
    nanovdb::NanoGrid<float>* grid = params.handle_ptr;
    const nanovdb::Vec3f ray_orig_v(ray_orig.x, ray_orig.y, ray_orig.z);
    const nanovdb::Vec3f ray_dir_v(ray_dir.x, ray_dir.y, ray_dir.z);
    float3 hitPoint = ray_orig + ray_dir * ray_dist;

    nanovdb::Vec3d hitPointIdx = grid->worldToIndex(nanovdb::Vec3d(hitPoint.x, hitPoint.y, hitPoint.z));
    nanovdb::Vec3d rayDirIdx = grid->worldToIndex(ray_dir_v);
    nanovdb::Vec3d rayOrigIdx = grid->worldToIndex(ray_orig_v);

    nanovdb::Ray<float> ray(rayOrigIdx, rayDirIdx, ray_dist, 1e20);
    /*   printf("VolShader: ray_dist: %f| hitP: %f,%f,%f | hitP Idx: %f,%f,%f | rayStartIdx: %f %f %f | rayDirIdx:
       %f,%f,%f\n", ray_dist, hitPoint.x, hitPoint.y, hitPoint.z, hitPointIdx[0], hitPointIdx[1], hitPointIdx[2],
       ray.start()[0], ray.start()[1], ray.start()[2], rayDirIdx[0], rayDirIdx[1], rayDirIdx[2]);*/

    nanovdb::Coord ijk = nanovdb::RoundDown<nanovdb::Coord>(ray.start());  // first hit of bbox
    // printf("ZCrossing::ray.start(): (%f,%f,%f) | ray.dir(): (%f,%f,%f)\n", ray.start()[0], ray.start()[1],
    // ray.start()[2], ray.dir()[0],ray.dir()[1],ray.dir()[2]);

    float v;
    nanovdb::DefaultReadAccessor<float> acc = grid->tree().getAccessor();
    nanovdb::HDDA<nanovdb::Ray<float>, nanovdb::Coord> hdda(ray, acc.getDim(ijk, ray));
    const auto v0 = acc.getValue(ijk);

    // printf("Start Value: %f | Start Idx: %f,%f,%f\n", v0, ijk.asVec3d()[0], ijk.asVec3d()[1], ijk.asVec3d()[2]);
    static const float Delta = 1.0001f;
    int nsteps = 0;
    float transmittance = 1.0f;
    float absorptionCoeff = 0.001;
    float scatteringCoeff = 0.01;
    float extinctionCoeff = absorptionCoeff + scatteringCoeff;
    float3 inScattering = make_float3(0);
    float outScattering = 0;
    float k = 0;  // isotropic reflections
    float3 volAlbedo = make_float3(0.659, 0.459, 0.051);

    int inactiveSteps = 0;
    float3 volumeLight = make_float3(0);
    while (hdda.step() && nsteps < 100) {
        ijk = nanovdb::RoundDown<nanovdb::Coord>(ray(hdda.time() + Delta));
        hdda.update(ray, acc.getDim(ijk, ray));
        if (hdda.dim() > 1 || !acc.isActive(ijk)) {
            inactiveSteps++;
            if (inactiveSteps > 1000)
                break;
            continue;  // either a tile value or an inactive voxel
        }

        // sample lights
        while (hdda.step() && acc.isActive(hdda.voxel())) {  // in the narrow band
            v = acc.getValue(hdda.voxel());                  // density
            ijk = hdda.voxel();
            nanovdb::Vec3f volPntIdx =
                grid->indexToWorld(ijk.asVec3s());  // TODO: Make VDB to chrono data type conversion function
            float3 volPnt = make_float3(volPntIdx[0], volPntIdx[1], volPntIdx[2]);
            // printf("density: %f\n", v);
            transmittance *= exp((-v * extinctionCoeff * Delta));
            outScattering = scatteringCoeff * v;

            for (int i = 0; i < params.num_lights; i++) {
                Light l = params.lights[i];
                if (l.type != LightType::POINT_LIGHT)
                    continue;
                float dist_to_light = Length(l.pos - volPnt);
                if (dist_to_light < 2 * l.max_range) {
                    float3 dir_to_light = normalize(l.pos - volPnt);
                    float VdL = Dot(-1 * normalize(ray_dir), -1 * dir_to_light);
                    if (VdL > 0) {
                        float3 light_attenuation = make_float3(0);  // TODO: shoot shadow ray
                        {
                            // Ray march to determine light attenuation
                            nanovdb::Ray<float> sRay(
                                ijk.asVec3d(),
                                grid->worldToIndex(nanovdb::Vec3d(dir_to_light.x, dir_to_light.y, dir_to_light.z)), 0.f,
                                1e20);
                            nanovdb::Coord sijk = nanovdb::RoundDown<nanovdb::Coord>(sRay.start());
                            nanovdb::HDDA<nanovdb::Ray<float>, nanovdb::Coord> shdda(sRay, acc.getDim(sijk, sRay));
                            int sinactiveSteps = 0;
                            int ssteps = 0;
                            float sV = 0;
                            float sTransmittance = 1.0f;
                            while (shdda.step() && ssteps < 50) {
                                sijk = nanovdb::RoundDown<nanovdb::Coord>(sRay(shdda.time() + Delta));
                                shdda.update(sRay, acc.getDim(sijk, sRay));
                                if (shdda.dim() > 1 || !acc.isActive(sijk)) {
                                    sinactiveSteps++;
                                    if (sinactiveSteps > 20)
                                        break;
                                    continue;  // either a tile value or an inactive voxel
                                }
                                while (shdda.step() && acc.isActive(shdda.voxel())) {  // in the narrow band
                                    sV = acc.getValue(shdda.voxel());
                                    sTransmittance *= exp((-sV * extinctionCoeff * Delta));  // density
                                    ssteps++;
                                }
                            }
                            light_attenuation = make_float3(clamp(sTransmittance, 0.f, 1.f));
                        }
                        // printf("Light Atten: %f\n", light_attenuation.x);
                        float point_light_falloff =
                            (l.max_range * l.max_range / (dist_to_light * dist_to_light + l.max_range * l.max_range));

                        float3 incoming_light_ray = l.color * light_attenuation * point_light_falloff * VdL;
                        float phase = SchlickPhase(VdL, k);
                        inScattering = params.ambient_light_color + incoming_light_ray * phase;

                        volumeLight += transmittance * inScattering * outScattering * Delta;
                    }
                }
            }
            nsteps++;
            // break;
        }
    }
    float alpha = 1 - clamp(transmittance, 0, 1);
    if (nsteps > 0) {
        prd_camera->transparency = 1 - alpha;
        prd_camera->color += volumeLight;  // make_float3(1-alpha, 1-alpha, 1-alpha);
        // prd_camera->color += make_float3(0, 0, 1);
    } else {
        // prd_camera->transparency = 1.f;
        prd_camera->color += make_float3(0, 0, 0);  // 0.1f, 0.2f, 0.4f
    }
#endif
}

static __device__ __inline__ void DepthShader(PerRayData_depthCamera* prd,
                                              const MaterialParameters& mat,
                                              const float3& world_normal,
                                              const float2& uv,
                                              const float3& tangent,
                                              const float& ray_dist,
                                              const float3& ray_orig,
                                              const float3& ray_dir) {
    prd->depth = fminf(prd->max_depth, ray_dist);
}

static __device__ inline void SamplePointLight(Light pl, LightSample* ls) {
    ls->dir = normalize(pl.pos - ls->hitpoint); // How much slow down due to derefing hitpoint twice?
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

    float cos_theta = Dot(spot.spot_dir, -1*ls->dir);
    
    // Replace max range with a high intensity
    //float point_light_falloff = (spot.max_range * spot.max_range / (dist * dist + spot.max_range * spot.max_range));
    ls->L = spot.color / (dist*dist);

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
    //printf("falloff: %f | dist: %f | cosTheta: %f\n", falloff, dist, cos_theta*180/CUDART_PI);
}

static __device__ inline void SampleLight(Light light, LightSample* ls) {
    switch (light.type) {
        case LightType::POINT_LIGHT:
            SamplePointLight(light, ls);
            break;
        case LightType::SPOT_LIGHT:
            //printf("Sample Spot!\n");
            SampleSpotLight(light,ls);
            break;
        default:
            break;
    }
}






static __device__ __inline__ float LambertianBSDFPdf(float3& wo, float3& wi, float3& n) {
    // float WodWi = Dot(wo,wi);
    float NdWi = Dot(n, wi);
    return NdWi > 0 ? NdWi * INV_PI : 0;
}

static __device__ __inline__ void LambertianBSDFSample(BSDFSample& sample, const MaterialParameters& mat, bool eval, float z1, float z2) {
    sample.f = mat.Kd * INV_PI;
    if (eval) return;

    sample.wi = sample_hemisphere_dir(z1, z2, sample.n);
    sample.pdf = LambertianBSDFPdf(sample.wo, sample.wi, sample.n);
}


static __device__ __inline__ BSDFSample SampleBSDF(BSDFType type,
                                                   BSDFSample& sample,
                                                   const MaterialParameters& mat,
                                                   bool eval = false,
                                                   float z1 = 0,
                                                   float z2 = 0) {
    switch (type) {
        case BSDFType::DIFFUSE:
            LambertianBSDFSample(sample, mat, eval, z1, z2);
            break;
        case BSDFType::SPECULAR:
            break;
        case BSDFType::DIELECTRIC:
            break;
        case BSDFType::GLOSSY:
            break;
        case BSDFType::DISNEY:
            break;
        case BSDFType::HAPKE:
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
            pdf = LambertianBSDFPdf(wo,wi,n);
            break;
        case BSDFType::SPECULAR:
            break;
        case BSDFType::DIELECTRIC:
            break;
        case BSDFType::GLOSSY:
            break;
        case BSDFType::DISNEY:
            break;
        case BSDFType::HAPKE:
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

static __device__ __inline__ float3 ComputeDirectLight(Light& l, LightSample& ls, const MaterialParameters& mat, int depth) {
    float3 Ld = make_float3(0.f);
    SampleLight(l, &ls);
    BSDFType bsdfType = (BSDFType)mat.BSDFType;
    if (ls.pdf > 0 && fmaxf(ls.L) > 0) {
        float NdL = Dot(ls.n, ls.dir);
        if (NdL > 0) {
            // Evaluate BSDF at light direction
            BSDFSample bsdf;
            bsdf.wi = ls.dir;
            bsdf.wo = ls.wo;
            bsdf.n = ls.n;
            SampleBSDF(bsdfType, bsdf, mat, true);
            float scatterPDF = EvalBSDFPDF(bsdfType, bsdf.wo, bsdf.wi, bsdf.n);
            if (!(fmaxf(bsdf.f) > 0)) return Ld; // If the BSDF is black, direct light contribution  is 0?
            // Shoot shadow rays
            PerRayData_shadow prd_shadow = default_shadow_prd();
            prd_shadow.depth = depth + 1;
            prd_shadow.ramaining_dist = ls.dist;
            unsigned int opt1;
            unsigned int opt2;
            pointer_as_ints(&prd_shadow, opt1, opt2);
            unsigned int raytype = (unsigned int)SHADOW_RAY_TYPE;
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

    //if (prd_camera->depth >= 3)
    //    printf("PI| d: %d | contr: (%f,%f,%f)\n", prd_camera->depth, prd_camera->contrib_to_pixel.x,
    //           prd_camera->contrib_to_pixel.y, prd_camera->contrib_to_pixel.z);
    const MaterialParameters& mat = params.material_pool[material_id];
    BSDFType bsdfType = (BSDFType)mat.BSDFType;
    float3 hit_point = ray_orig + ray_dir * ray_dist;
    float3 L = make_float3(0.0f);

    float3 wo = -ray_dir;

    // Add ambient light
    //prd_camera->color += params.ambient_light_color * prd_camera->contrib_to_pixel;  // ?

    float3 Le = make_float3(0.f);
    // TODO: Add Emisions from Area Lights

    // Direct light contributions
    float3 Ld = make_float3(0.f);
   
    if (params.num_lights > 0 && bsdfType != BSDFType::SPECULAR) {
        //printf("Direct Light| BSDF: %d != %d\n", bsdfType, BSDFType::SPECULAR);
        // Uniform sample light
        unsigned int sample_light_index = (unsigned int)(curand_uniform(&prd_camera->rng) *params.num_lights);  // TODO: Won't work for whitted as no GI, have a global sampler instead?
        Light l = params.lights[sample_light_index];
        LightSample ls;
        ls.hitpoint = hit_point;
        ls.wo = wo;
        ls.n = world_normal;
        
        // Compute direct lighting
        float3 ld = ComputeDirectLight(l, ls, mat, prd_camera->depth);
        //printf("d: %d | DL: (%f,%f,%f) \n", prd_camera->depth, ld.x,ld.y,ld.z);
        Ld = prd_camera->contrib_to_pixel * ComputeDirectLight(l,ls,mat,prd_camera->depth);
    }
    L += Ld;
    
    if (prd_camera->depth + 1 < params.max_depth) {
       // printf("Next ray!\n");
        BSDFSample sample;
        sample.wo = wo;
        sample.n = world_normal;
        float z1 = curand_uniform(&prd_camera->rng);
        float z2 = curand_uniform(&prd_camera->rng);
        SampleBSDF(bsdfType, sample, mat, false, z1, z2);
        float NdL = Dot(sample.n, sample.wi);
        float3 next_contrib_to_pixel = prd_camera->contrib_to_pixel * sample.f * NdL / sample.pdf; 
        if (luminance(sample.f) > params.importance_cutoff && sample.pdf > 0 && fmaxf(next_contrib_to_pixel) > 0) {         
            // Check possible rr termination
            float rr_thresh = .1f; // Replace this with importance_cutoff?
            if (fmaxf(next_contrib_to_pixel) < rr_thresh && prd_camera->depth > 3) {
                float q = fmaxf((float).05, 1 - fmaxf(next_contrib_to_pixel));
                float p = curand_uniform(&prd_camera->rng);
                if (p < q) return;
                next_contrib_to_pixel = next_contrib_to_pixel / (1-q);
            }

            // Trace next ray
            PerRayData_camera prd_reflection = default_camera_prd();
            prd_reflection.integrator = prd_camera->integrator;
            prd_reflection.contrib_to_pixel = next_contrib_to_pixel;
            prd_reflection.rng = prd_camera->rng;
            prd_reflection.depth = prd_camera->depth + 1;
            prd_reflection.use_gi = prd_camera->use_gi;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_reflection, opt1, opt2);
            unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
            optixTrace(params.root, hit_point, sample.wi, params.scene_epsilon, 1e16f, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
            L += prd_reflection.color;
        }
    }

    prd_camera->color += L;
    prd_camera->albedo = mat.Kd; // Might change
    prd_camera->normal = world_normal;
}

static __device__ __inline__ void TransientPathIntegrator(PerRayData_transientCamera* prd_camera,
                                                          const MaterialRecordParameters* mat_params,
                                                          unsigned int& material_id,
                                                          const unsigned int& num_blended_materials,
                                                          const float3& world_normal,
                                                          const float2& uv,
                                                          const float3& tangent,
                                                          const float& ray_dist,
                                                          const float3& ray_orig,
                                                          const float3& ray_dir) {

    //printf("TRANSIENT Integrator!\n");
    const MaterialParameters& mat = params.material_pool[material_id];
    BSDFType bsdfType = (BSDFType)mat.BSDFType;
    float3 hit_point = ray_orig + ray_dir * ray_dist;
    float3 L = make_float3(0.0f);

    float3 wo = -ray_dir;

    prd_camera->path_length += ray_dist;

    // Add ambient light
    // prd_camera->color += params.ambient_light_color * prd_camera->contrib_to_pixel;  // ?

    float3 Le = make_float3(0.f);
    // TODO: Add Emisions from Area Lights

    // Direct light contributions
    float3 Ld = make_float3(0.f);

    if (params.num_lights > 0 && bsdfType != BSDFType::SPECULAR) {
        // Uniform sample light
        unsigned int sample_light_index =
            (unsigned int)(curand_uniform(&prd_camera->rng) *
                           params.num_lights);  // TODO: Won't work for whitted as no GI, have a global sampler instead?
        Light l = params.lights[sample_light_index];
        LightSample ls;
        ls.hitpoint = hit_point;
        ls.wo = wo;
        ls.n = world_normal;

        // Compute direct lighting
        Ld = prd_camera->contrib_to_pixel * ComputeDirectLight(l, ls, mat, prd_camera->depth);
  
        if (fmaxf(Ld) > 0) {
        
            TransientSample sample = {};
            sample.pathlength = prd_camera->path_length + ls.dist;
            sample.color = Ld;
            int idx = params.max_depth * prd_camera->current_pixel + (prd_camera->depth - 1);
           
            params.transient_buffer[idx] = sample;
        }
    }
    L += Ld;

    if (prd_camera->depth + 1 < params.max_depth) {
        BSDFSample sample;
        sample.wo = wo;
        sample.n = world_normal;
        float z1 = curand_uniform(&prd_camera->rng);
        float z2 = curand_uniform(&prd_camera->rng);
        SampleBSDF(bsdfType, sample, mat, false, z1, z2);
        if (luminance(sample.f) > params.importance_cutoff > 0 && sample.pdf > 0) {
            float NdL = Dot(sample.n, sample.wi);
            float3 next_contrib_to_pixel = prd_camera->contrib_to_pixel * sample.f * NdL / sample.pdf;
            if (fmaxf(next_contrib_to_pixel) < 0) {
                prd_camera->depth_reached = prd_camera->depth;
                return;
            }
            // Check possible rr termination
            float rr_thresh = .1f;
            if (fmaxf(next_contrib_to_pixel) < rr_thresh && prd_camera->depth > 3) {
                float q = fmaxf((float).05, 1 - fmaxf(next_contrib_to_pixel));
                float p = curand_uniform(&prd_camera->rng);
                if (p < q)
                    return;
                next_contrib_to_pixel = next_contrib_to_pixel / (1 - q);
            }

            // Trace next ray
            PerRayData_transientCamera prd_reflection = default_transientCamera_prd(prd_camera->current_pixel);
            prd_reflection.integrator = prd_camera->integrator;
            prd_reflection.contrib_to_pixel = next_contrib_to_pixel;
            prd_reflection.rng = prd_camera->rng;
            prd_reflection.depth = prd_camera->depth + 1;
            prd_reflection.use_gi = prd_camera->use_gi;
            prd_reflection.path_length = prd_camera->path_length;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_reflection, opt1, opt2);
            unsigned int raytype = (unsigned int)TRANSIENT_RAY_TYPE;
            optixTrace(params.root, hit_point, sample.wi, params.scene_epsilon, 1e16f, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
            L += prd_reflection.color;
            prd_camera->depth_reached = prd_reflection.depth_reached;
        }
    } else {
        prd_camera->depth_reached = prd_camera->depth;
    }

    prd_camera->color += L;
    prd_camera->albedo = mat.Kd;  // Might change
    prd_camera->normal = world_normal;
}

static __device__ __inline__ float ComputePathLengthImportance(float path_length) {
    float importance = 1.f;
    float v = (path_length - params.target_dist) / params.window_size;
    switch (params.timegated_mode) {
        case TIMEGATED_MODE::BOX: {
            importance = fabsf(v) < .5f ? 1.f : 0.f;
            break;
        } case TIMEGATED_MODE::TENT :{
            importance = fmaxf(1 - fabsf(v),0.f);
            break;
        }
        case TIMEGATED_MODE::COS :{
            importance = cosf(2*CUDART_PI*v);
            break;
        }
        case TIMEGATED_MODE::SIN: {
            importance = sinf(2 * CUDART_PI * v);
            break;
        }
        case TIMEGATED_MODE::EXPONENTIAL: {
            importance = path_length > params.target_dist ? 0.f : expf(v);
            break;
        }
    }
    return importance;
}

static __device__ __inline__ void TimeGatedIntegrator(PerRayData_transientCamera* prd_camera,
                                                          const MaterialRecordParameters* mat_params,
                                                          unsigned int& material_id,
                                                          const unsigned int& num_blended_materials,
                                                          const float3& world_normal,
                                                          const float2& uv,
                                                          const float3& tangent,
                                                          const float& ray_dist,
                                                          const float3& ray_orig,
                                                          const float3& ray_dir) {
    //printf("TIMEGATED Integrator!\n");
    const MaterialParameters& mat = params.material_pool[material_id];
    BSDFType bsdfType = (BSDFType)mat.BSDFType;
    float3 hit_point = ray_orig + ray_dir * ray_dist;
    float3 L = make_float3(0.0f);

    float3 wo = -ray_dir;

    prd_camera->path_length += ray_dist;

    // Add ambient light
    // prd_camera->color += params.ambient_light_color * prd_camera->contrib_to_pixel;  // ?

    float3 Le = make_float3(0.f);
    // TODO: Add Emisions from Area Lights

    // Direct light contributions
    float3 Ld = make_float3(0.f);

    if (params.num_lights > 0 && bsdfType != BSDFType::SPECULAR) {
        // Uniform sample light
        unsigned int sample_light_index =
            (unsigned int)(curand_uniform(&prd_camera->rng) *
                           params.num_lights);  // TODO: Won't work for whitted as no GI, have a global sampler instead?
        Light l = params.lights[sample_light_index];
        LightSample ls;
        ls.hitpoint = hit_point;
        ls.wo = wo;
        ls.n = world_normal;

        // Compute direct lighting
        Ld = prd_camera->contrib_to_pixel * ComputeDirectLight(l, ls, mat, prd_camera->depth);

        if (fmaxf(Ld) > 0) {
            float path_importance = ComputePathLengthImportance(prd_camera->path_length + ls.dist);
            L += (Ld * path_importance);
        }
    }


    if (prd_camera->depth + 1 < params.max_depth) {
        BSDFSample sample;
        sample.wo = wo;
        sample.n = world_normal;
        float z1 = curand_uniform(&prd_camera->rng);
        float z2 = curand_uniform(&prd_camera->rng);
        SampleBSDF(bsdfType, sample, mat, false, z1, z2);
        if (luminance(sample.f) > params.importance_cutoff > 0 && sample.pdf > 0) {

            float NdL = Dot(sample.n, sample.wi);
            float3 next_contrib_to_pixel = prd_camera->contrib_to_pixel * sample.f * NdL / sample.pdf;
            if (fmaxf(next_contrib_to_pixel) < 0) {
                prd_camera->depth_reached = prd_camera->depth;
                return;
            }
            // Check possible rr termination
            float rr_thresh = .1f;
            if (fmaxf(next_contrib_to_pixel) < rr_thresh && prd_camera->depth > 3) {
                float q = fmaxf((float).05, 1 - fmaxf(next_contrib_to_pixel));
                float p = curand_uniform(&prd_camera->rng);
                if (p < q)
                    return;
                next_contrib_to_pixel = next_contrib_to_pixel / (1 - q);
            }

            // Trace next ray
            PerRayData_transientCamera prd_reflection = default_transientCamera_prd(prd_camera->current_pixel);
            prd_reflection.integrator = prd_camera->integrator;
            prd_reflection.contrib_to_pixel = next_contrib_to_pixel;
            prd_reflection.rng = prd_camera->rng;
            prd_reflection.depth = prd_camera->depth + 1;
            prd_reflection.use_gi = prd_camera->use_gi;
            prd_reflection.path_length = prd_camera->path_length;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_reflection, opt1, opt2);
            unsigned int raytype = (unsigned int)TRANSIENT_RAY_TYPE;
            optixTrace(params.root, hit_point, sample.wi, params.scene_epsilon, 1e16f, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
            L += prd_reflection.color;
            prd_camera->depth_reached = prd_reflection.depth_reached;
        }
    } else {
        prd_camera->depth_reached = prd_camera->depth;
    }

    prd_camera->color += L;
    prd_camera->albedo = mat.Kd;  // Might change
    prd_camera->normal = world_normal;
}

static __device__  __inline__ int binary_search_cdf(const float* cdf, int num_elements, float value) {
    int low = 0;
    int high = num_elements - 1;

    while (low < high) {
        int mid = (low + high) / 2;
        if (cdf[mid] < value) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    return low;
}

static __device__ __inline__ void sample_reuse_kernel(const float* cdf,
                                    const float* pmf,
                                    int num_elements,
                                    float* random_samples,
                                    int* sampled_indices,
                                    float* rescaled_samples) {

    //float value = random_samples[idx];

    //// Find the index corresponding to the CDF
    //int index = binary_search_cdf(cdf, num_elements, value);

    //// Compute the PMF and CDF values at the index
    //float pmf_value = pmf[index];
    //float cdf_value = (index > 0) ? cdf[index - 1] : 0.0f;

    //// Rescale the sample for reuse
    //float rescaled_value = (value - cdf_value) / pmf_value;

    //// Store the results
    //sampled_indices[idx] = index;
    //rescaled_samples[idx] = rescaled_value;
}

static __device__ __inline__ float3 EmitterLaserSample() {

}

static __device__ __inline__ void SampleHiddenGeometryPos() {

}

static __device__ __inline__ void HiddenGeometrySample() {

}

static __device__ __inline__ void LaserNEEE(PerRayData_laserSampleRay* prd,
                                            const MaterialRecordParameters* mat_params,
                                            unsigned int& material_id,
                                            const unsigned int& num_blended_materials,
                                            const float3& world_normal,
                                            const float2& uv,
                                            const float3& tangent,
                                            const float& ray_dist,
                                            const float3& ray_orig,
                                            const float3& ray_dir) {

    const MaterialParameters& mat = params.material_pool[material_id];
    BSDFType bsdfType = (BSDFType)mat.BSDFType;
    // Set the hit point manually as the laser focus point to account for floating point errors in optix
    // Since the original hitpoint with error is somehere around the focus point this pobs won't be a bad approximation?
    float3 hit_point = prd->laser_hitpoint ; //
    float3 Lr = make_float3(0.0f);

    float3 wo = -ray_dir;

    float3 Ld = make_float3(0.f);

    if (params.num_lights > 0 && bsdfType != BSDFType::SPECULAR) {
        // Uniform sample light
        unsigned int sample_light_index = 0; // NLOS scenes assume there is only one light source in the scene
        Light l = params.lights[sample_light_index];
        LightSample ls;
        ls.hitpoint = hit_point;
        ls.wo = wo;
        ls.n = world_normal;

        // Compute direct lighting
        float3 dl = ComputeDirectLight(l, ls, mat, prd->depth);
        Ld = prd->contribution * dl;
      
   /*      if (fmaxf(Ld) < 0) {
            
            printf("Hit Point LS: (%f,%f,%f) | t: %f | contr: (%f,%f,%f) | dl: (%f,%f,%f), Ld: (%f,%f,%f)\n",  
                hit_point.x, hit_point.y, hit_point.z, optixGetRayTime(), 
                prd->contribution.x,prd->contribution.y,prd->contribution.z,
                dl.x,dl.y,dl.z,
                Ld.x,Ld.y,Ld.z);
         }*/
        prd->path_length += ls.dist;
        prd->Lr = Ld;
    }

}

static __device__ __inline__ void MITransientIntegrator(PerRayData_transientCamera* prd_camera,
                                                      const MaterialRecordParameters* mat_params,
                                                      unsigned int& material_id,
                                                      const unsigned int& num_blended_materials,
                                                      const float3& world_normal,
                                                      const float2& uv,
                                                      const float3& tangent,
                                                      const float& ray_dist,
                                                      const float3& ray_orig,
                                                      const float3& ray_dir) {
    // printf("TIMEGATED Integrator!\n");
    const MaterialParameters& mat = params.material_pool[material_id];
    BSDFType bsdfType = (BSDFType)mat.BSDFType;
    float3 hit_point = ray_orig + ray_dir * ray_dist;
   
    float3 L = make_float3(0.0f);

    float3 wo = -ray_dir;

    prd_camera->path_length += ray_dist;

    // Direct emission
    {

    }

    // Laser sampling
    float3 Lr = make_float3(0.f);
    if (params.nlos_laser_sampling) {
        /*printf("NLOS Laser Sampling: hp: (%f,%f,%f), lp: (%f,%f,%f)\n", 
            hit_point.x,hit_point.y,hit_point.z, 
            prd_camera->laser_focus_point.x,prd_camera->laser_focus_point.y,prd_camera->laser_focus_point.z);*/
        float laser_dist = Length(prd_camera->laser_focus_point - hit_point);
        float3 laser_dir = (prd_camera->laser_focus_point - hit_point) / laser_dist;
        // Shoot shadow ray to test visibility to laser target
        PerRayData_shadow prd_shadow = default_shadow_prd();
        prd_shadow.depth = prd_camera->depth + 1;
        prd_shadow.ramaining_dist = laser_dist;
        unsigned int opt1;
        unsigned int opt2;
        pointer_as_ints(&prd_shadow, opt1, opt2);
        unsigned int raytype = (unsigned int)SHADOW_RAY_TYPE;
        optixTrace(params.root, hit_point, laser_dir, params.scene_epsilon, laser_dist, optixGetRayTime(),
                   OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

        float NdLaser = Dot(world_normal, -1*laser_dir);
      
        if (fmaxf(prd_shadow.attenuation) > 0 && laser_dist > 1e-8f) {  // 
           /* printf("NdLaser: %f, d: (%f,%f,%f), n: (%f,%f,%f)\n", NdLaser, laser_dir.x, laser_dir.y, laser_dir.z,
                   world_normal.x, world_normal.y, world_normal.z);*/
            BSDFSample laser_dir_bsdf;
            laser_dir_bsdf.wo = wo;
            laser_dir_bsdf.wi  = laser_dir;
            laser_dir_bsdf.n = world_normal;
            SampleBSDF(bsdfType,laser_dir_bsdf,mat,true);
            //laser_dir_bsdf.pdf = EvalBSDFPDF(bsdfType, wo, laser_dir, laser_dir_bsdf.n);
          
            if (NdLaser > 0) {
                float pdf_ls = (laser_dist * laser_dist) / NdLaser;
                laser_dir_bsdf.pdf /= pdf_ls;
            }
            // Shoot laser ray towards laser focus point
            PerRayData_laserSampleRay prd = default_laserSampleRay_prd();
            prd.sample_laser = true;
            prd.path_length = prd_camera->path_length + laser_dist;
            prd.bsdf_pdf = laser_dir_bsdf.pdf;
            prd.contribution = prd_camera->contrib_to_pixel * laser_dir_bsdf.pdf;
            prd.depth = prd_camera->depth + 1;
            prd.laser_hitpoint = prd_camera->laser_focus_point;
            unsigned int opt1;
            unsigned int opt2;
            pointer_as_ints(&prd, opt1, opt2);
            unsigned int raytype = (unsigned int)LASER_SAMPLE_RAY_TYPE;
            //printf("Hit Point: (%f,%f,%f)\n", hit_point.x, hit_point.y, hit_point.z);
        /*    printf("laser dist: %f t: %f | PL: %f |Target: (%f,%f,%f), o: (%f,%f,%f) d: (%f,%f,%f) | Proj HP: (%f,%f,%f)\n", 
                   laser_dist,
                   optixGetRayTime(),
                   prd.path_length,
                   prd.laser_hitpoint.x, prd.laser_hitpoint.y, prd.laser_hitpoint.z, 
                   hit_point.x, hit_point.y, hit_point.z, laser_dir.x,
                   laser_dir.y, laser_dir.z,
                   proj_hp.x, proj_hp.y, proj_hp.z);*/
              optixTrace(params.root, hit_point, laser_dir, params.scene_epsilon, laser_dist, laser_dist, OptixVisibilityMask(1),
                       OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT, 0, 1, 0, opt1, opt2, raytype);
         
            // Add transient sample
            if (fmaxf(prd.Lr) > 0) {
               /* printf("Laser Lr: (%f,%f,%f) | PL Before: %f, PL: %f\n", prd.Lr.x, prd.Lr.y, prd.Lr.z,
                         prd_camera->path_length + laser_dist, prd.path_length);*/
                TransientSample sample = {};
                sample.pathlength = prd.path_length;
                sample.color = prd.Lr;
                int idx = params.max_depth * prd_camera->current_pixel + (prd_camera->depth - 1);

                params.transient_buffer[idx] = sample;
                
            }
        }
    } 
    else { // Do standard NEE Direct lighting
        // Uniform sample light
        unsigned int sample_light_index =
            (unsigned int)(curand_uniform(&prd_camera->rng) *
                           params.num_lights);  // TODO: Won't work for whitted as no GI, have a global sampler instead?
        Light l = params.lights[sample_light_index];
        LightSample ls;
        ls.hitpoint = hit_point;
        ls.wo = wo;
        ls.n = world_normal;

        // Compute direct lighting
        Lr = prd_camera->contrib_to_pixel * ComputeDirectLight(l, ls, mat, prd_camera->depth);

        if (fmaxf(Lr) > 0) {
            TransientSample sample = {};
            sample.pathlength = prd_camera->path_length + ls.dist;
            sample.color = Lr;
            int idx = params.max_depth * prd_camera->current_pixel + (prd_camera->depth - 1);

            params.transient_buffer[idx] = sample;
        }
    }

    L += Lr;

    // Find next direction by either hidden geometry sampling or standard bsdf sampling
    {
        if (prd_camera->depth + 1 < params.max_depth) {
            BSDFSample sample;
            sample.wo = wo;
            sample.n = world_normal;
            float z1 = curand_uniform(&prd_camera->rng);
            float z2 = curand_uniform(&prd_camera->rng);
            SampleBSDF(bsdfType, sample, mat, false, z1, z2);

            if (luminance(sample.f) > params.importance_cutoff > 0 && sample.pdf > 0) {
                float NdL = Dot(sample.n, sample.wi);
                float3 next_contrib_to_pixel = prd_camera->contrib_to_pixel * sample.f * NdL / sample.pdf;
                if (fmaxf(next_contrib_to_pixel) < 0) {
                    prd_camera->depth_reached = prd_camera->depth;
                    return;
                }
                // Check possible rr termination
                float rr_thresh = .1f;
                if (fmaxf(next_contrib_to_pixel) < rr_thresh && prd_camera->depth > 3) {
                    float q = fmaxf((float).05, 1 - fmaxf(next_contrib_to_pixel));
                    float p = curand_uniform(&prd_camera->rng);
                    if (p < q) {
                        prd_camera->depth_reached = prd_camera->depth;
                        return;
                    }
                    next_contrib_to_pixel = next_contrib_to_pixel / (1 - q);
                }

                // Trace next ray
                PerRayData_transientCamera prd_reflection = default_transientCamera_prd(prd_camera->current_pixel);
                prd_reflection.integrator = prd_camera->integrator;
                prd_reflection.contrib_to_pixel = next_contrib_to_pixel;
                prd_reflection.rng = prd_camera->rng;
                prd_reflection.depth = prd_camera->depth + 1;
                prd_reflection.use_gi = prd_camera->use_gi;
                prd_reflection.path_length = prd_camera->path_length;
                prd_reflection.laser_focus_point = prd_camera->laser_focus_point;
                unsigned int opt1, opt2;
                pointer_as_ints(&prd_reflection, opt1, opt2);
                unsigned int raytype = (unsigned int)TRANSIENT_RAY_TYPE;
                optixTrace(params.root, hit_point, sample.wi, params.scene_epsilon, 1e16f, optixGetRayTime(),
                           OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
                L += prd_reflection.color;
                prd_camera->depth_reached = prd_reflection.depth_reached;
            }
        } else {
            prd_camera->depth_reached = prd_camera->depth;
        }

        prd_camera->color += L;
        prd_camera->albedo = mat.Kd;  // Might change
        prd_camera->normal = world_normal;
    }
}

extern "C" __global__ void __closesthit__material_shader() {
    //printf("Material Shader!\n");
    // determine parameters that are shared across all ray types
    const MaterialRecordParameters* mat_params = (MaterialRecordParameters*)optixGetSbtDataPointer();

    const float3 ray_orig = optixGetWorldRayOrigin();
    const float3 ray_dir = normalize(optixGetWorldRayDirection());  // this may be modified by the scaling transform
    const float ray_dist = optixGetRayTmax();

    float3 hit_point = ray_orig + ray_dir*ray_dist;

    //printf("NVDBVolShader: orig: (%f,%f,%f), dir:(%f,%f,%f)\n", ray_orig.x, ray_orig.y, ray_orig.z, ray_dir.x, ray_dir.y,ray_dir.z);
    float3 object_normal;
    float2 uv;
    float3 tangent;


    // check if we hit a triangle
    unsigned int material_id = mat_params->material_pool_id;
    const MaterialParameters& mat = params.material_pool[material_id];
    if (optixIsTriangleHit()) {
        GetTriangleData(object_normal, material_id, uv, tangent, mat_params->mesh_pool_id);
    } else {
        object_normal = make_float3(__int_as_float(optixGetAttribute_0()), __int_as_float(optixGetAttribute_1()),
                                    __int_as_float(optixGetAttribute_2()));
        uv = make_float2(__int_as_float(optixGetAttribute_3()), __int_as_float(optixGetAttribute_4()));
        tangent = make_float3(__int_as_float(optixGetAttribute_5()), __int_as_float(optixGetAttribute_6()),
                              __int_as_float(optixGetAttribute_7()));
    }

    

    if (mat.kn_tex) {
        float3 bitangent = normalize(Cross(object_normal, tangent));
        const float4 tex = tex2D<float4>(mat.kn_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
        float3 normal_delta = make_float3(tex.x, tex.y, tex.z) * 2.f - make_float3(1.f);
        object_normal =normalize(normal_delta.x * tangent + normal_delta.y * bitangent + normal_delta.z * object_normal);
    }

    float3 world_normal = normalize(optixTransformNormalFromObjectToWorldSpace(object_normal));

    // from here on out, things are specific to the ray type
    RayType raytype = (RayType)optixGetPayload_2();


    switch (raytype) {
        case CAMERA_RAY_TYPE:
            PerRayData_camera* prd_cam = getCameraPRD();
            switch (prd_cam->integrator)
            {
                case Integrator::PATH:
                    CameraPathIntegrator(prd_cam, mat_params, material_id, mat_params->num_blended_materials,
                                 world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
                    break;
                case Integrator::VOLUMETRIC:
                    break; 
                case Integrator::LEGACY:
                    //printf("LEGACY SHADER!\n");
                    CameraShader(prd_cam, mat_params, material_id, mat_params->num_blended_materials,world_normal, uv,tangent, ray_dist, ray_orig, ray_dir);
                    break;
                default:
                    break;
            }
            // switch(mat.BSDFType) {
            //     case 0:
            //         CameraShader(getCameraPRD(), mat_params, material_id, mat_params->num_blended_materials, world_normal, uv,
            //                 tangent, ray_dist, ray_orig, ray_dir);
            //         break;
            //     case 1:
            //         CameraHapkeShader(getCameraPRD(), mat_params, material_id, mat_params->num_blended_materials, world_normal, uv,
            //                 tangent, ray_dist, ray_orig, ray_dir);
            //         break;
            //     case 2:
            //         CameraDiffuseShader(getCameraPRD(), mat_params, material_id, mat_params->num_blended_materials, world_normal, uv,
            //                 tangent, ray_dist, ray_orig, ray_dir);
            //         break;
            //     case 3:
            //         CameraVolumetricShader(getCameraPRD(), mat_params, material_id, mat_params->num_blended_materials,
            //                             world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            //         break;
            //     default:
            //         break;
            // }
                
            break;
        case TRANSIENT_RAY_TYPE:
            PerRayData_transientCamera* transCam_prd = getTransientCameraPRD();
            switch (transCam_prd->integrator)
            {
                case Integrator::TRANSIENT:
                    TransientPathIntegrator(transCam_prd, mat_params, material_id, mat_params->num_blended_materials, world_normal, uv,
                        tangent, ray_dist, ray_orig, ray_dir);
                    break;
                case Integrator::TIMEGATED:
                   TimeGatedIntegrator(transCam_prd, mat_params, material_id, mat_params->num_blended_materials,
                                            world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
                   break;
                case Integrator::MITRANSIENT:
                   MITransientIntegrator(transCam_prd, mat_params, material_id, mat_params->num_blended_materials,
                                        world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
                    break;
                default:
                    break;
            }
            break;
        case LIDAR_RAY_TYPE:
            LidarShader(getLidarPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
        case RADAR_RAY_TYPE:
            RadarShader(getRadarPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir,
                        mat_params->translational_velocity, mat_params->angular_velocity, mat_params->objectId);
            break;
        case SHADOW_RAY_TYPE:
            ShadowShader(getShadowPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
        case SEGMENTATION_RAY_TYPE:
            SemanticShader(getSemanticPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
        case DEPTH_RAY_TYPE:
            DepthShader(getDepthCameraPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
        case LASER_SAMPLE_RAY_TYPE:
            PerRayData_laserSampleRay* prd = getLaserPRD();
            if (prd->sample_laser) {
                LaserNEEE(prd, mat_params, material_id, mat_params->num_blended_materials,
                                      world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            } else {
                prd->laser_hitpoint = hit_point;
            }
            break;
    }
}
