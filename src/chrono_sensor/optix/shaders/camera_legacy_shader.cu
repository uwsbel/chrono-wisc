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
// Legacy camera shader
//
// =============================================================================

#ifndef CAMERA_LEGACY_SHADER_CU
#define CAMERA_LEGACY_SHADER_CU

#include "chrono_sensor/optix/shaders/device_utils.h"
#include "chrono_sensor/optix/shaders/shader_utils.cu"
#include "chrono_sensor/optix/shaders/ChOptixLightHubs.cu"


// Indirect illumination ray
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

    {
        // Sample hemisphere for next ray when using global illumination
        float z1 = curand_uniform(&prd_camera->rng);
        float z2 = curand_uniform(&prd_camera->rng);
        float3 next_dir = sample_cosine_hemisphere_dir(z1, z2, world_normal);

        float NdL = Dot(world_normal, next_dir);
        float3 halfway = normalize(next_dir - ray_dir);
        float NdH = Dot(world_normal, halfway);
        float VdH = Dot(-ray_dir, halfway);  // Same as LdH

        float3 this_contrib_to_pixel = make_float3(0.f);
        float3 next_contrib_to_pixel = make_float3(0.f);

        for (int b = 0; b < num_blended_materials; b++) {
            const MaterialParameters& mat = params.material_pool[material_id + b];

			// Get BRDF model parameter values, with using textures' values if textures exist
    		float3 albedo = (mat.kd_tex) ? Pow(GetTexFloat4ValFloat3(mat.kd_tex, mat.tex_scale, uv), 2.2) : mat.Kd; // transfer texture from sRGB space into linear color space
    		float roughness = (mat.roughness_tex) ? GetTexValFloat(mat.roughness_tex, mat.tex_scale, uv): mat.roughness;
    		float metallic = (mat.metallic_tex) ? GetTexValFloat(mat.metallic_tex, mat.tex_scale, uv) : mat.metallic;
			float3 specular = (mat.ks_tex) ? GetTexFloat4ValFloat3(mat.ks_tex, mat.tex_scale, uv) : mat.Ks; // transfer texture from sRGB space into linear color space
    		float opacity = (mat.opacity_tex) ? GetTexValFloat(mat.opacity_tex, mat.tex_scale, uv) : mat.transparency;
    		float mat_blend_weight = (mat.weight_tex) ? GetTexValFloat(mat.weight_tex, mat.tex_scale, uv) : (1.f / num_blended_materials);
    		float contrib_weight = opacity * mat_blend_weight;  // correct for transparency, light bounces, and blend weight

            this_contrib_to_pixel += PrincipledBRDF(
				albedo, roughness, metallic, specular, contrib_weight, mat.use_specular_workflow, NdV, NdL, NdH, VdH
			);
            
        }
        next_contrib_to_pixel = prd_camera->contrib_to_pixel * this_contrib_to_pixel;

        // If mirror_reflection, then it will trace two rays. So each ray's contribution should be halfed
        if (prd_camera->use_gi) {
        // if ((mirror_reflection_color.x < 1e-6) && (mirror_reflection_color.y < 1e-6) && (mirror_reflection_color.z < 1e-6)) {
            next_contrib_to_pixel = next_contrib_to_pixel * .5f;
            this_contrib_to_pixel = this_contrib_to_pixel * .5f;
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
            diffuse_reflected_color = prd_reflection.color ;  // accumulate indirect lighting color
        }
    }

    return diffuse_reflected_color;
}


// Calculate ambient Light response
static __device__ __inline__ float3 GetAmbientReflectedColor(PerRayData_camera* prd_camera,
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


// Calculate surface reflection toward the specular reflection direction. Do this reflection regardless of GI on or off.
static __device__ __inline__ float3 GetSpecularReflectedColor(const ContextParameters& cntxt_params,
                                                              PerRayData_camera* prd_camera,
                                                              const int& num_blended_materials,
                                                              unsigned int& material_id,
                                                              const float2& uv,
                                                              const float3& world_normal,
                                                              const float3& ray_dir,
                                                              const float3& hit_point,
                                                              const float& NdV) {
    float3 next_contrib_to_pixel = make_float3(0.f);
    float3 this_contrib_to_pixel = make_float3(0.f);
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
            float3 specular = (mat.ks_tex) ? GetTexFloat4ValFloat3(mat.ks_tex, mat.tex_scale, uv) : mat.Ks; // transfer texture from sRGB space into linear color space
            float opacity = (mat.opacity_tex) ? GetTexValFloat(mat.opacity_tex, mat.tex_scale, uv) : mat.transparency;
            float mat_blend_weight = (mat.weight_tex) ? GetTexValFloat(mat.weight_tex, mat.tex_scale, uv) : (1.f / num_blended_materials);
            float weight = opacity * mat_blend_weight; // corrected for opacity, bounce contribution, and blend
            
            /*
            float3 F = make_float3(0.0f);
            // === dielectric workflow
            if (mat.use_specular_workflow) {
                float3 specular = (mat.ks_tex) ? GetTexFloat4ValFloat3(mat.ks_tex, mat.tex_scale, uv) : mat.Ks;
                float3 F0 = specular * 0.08f;
                F = fresnel_schlick(VdH, 5.f, F0, make_float3(1.f));
            } else {
                float3 default_dielectrics_F0 = make_float3(0.04f);
                F = metallic * albedo + (1 - metallic) * default_dielectrics_F0;
            }

            float D = NormalDist(NdH, roughness);        // 1/pi omitted
            float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted

            float3 f_ct = F * D * G;

            // Note only specular part appears here. Energy preserve
            // Since it is not random, PDF is 1 (normally 1/pi),
            // If the camera uses GI, then it will trace two rays. So each ray's contribution should be halfed
            */

            
            float mirror_correction = (1.f - roughness) * (1.f - roughness) * metallic * metallic;

            this_contrib_to_pixel += PrincipledBRDF(
				albedo, roughness, metallic, specular, weight, mat.use_specular_workflow, NdV, NdL, NdH, VdH
			) * mirror_correction * NdL / (4 * CUDART_PI_F);
            
            this_contrib_to_pixel = clamp(this_contrib_to_pixel, make_float3(0), make_float3(1));

            // mirror correction accounts for us oversampling this direction
            // following line comes from a heuristic. Perect reflection for metalic smooth objects,
            // no reflection for rough non-metalic objects
            

            // if global illumination, ray contrib will be halved since two rays are propogated
            // if (prd_camera->use_gi) {
            //     weight = weight * .5f;
            // }

            // float3 partial_contrib = mirror_correction * weight * f_ct * NdL / (4 * CUDART_PI_F);
            // partial_contrib = clamp(partial_contrib, make_float3(0), make_float3(1));
            // partial_contrib = partial_contrib * prd_camera->contrib_to_pixel;
            // next_contrib_to_pixel += partial_contrib;
            // next_contrib_to_pixel = clamp(next_contrib_to_pixel, make_float3(0), make_float3(1));
        }
        // partial_contrib = partial_contrib * prd_camera->contrib_to_pixel;
        if (prd_camera->use_gi) {
            this_contrib_to_pixel = this_contrib_to_pixel * .5f;
        }
        next_contrib_to_pixel = prd_camera->contrib_to_pixel * this_contrib_to_pixel;
        next_contrib_to_pixel = clamp(next_contrib_to_pixel, make_float3(0), make_float3(1));

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

            specular_reflection_color = prd_reflection.color;  // accumulate specular reflection color
        }
    }

    return specular_reflection_color;
}


// Calculate responded color of surface reflection toward light sources
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
        ChOptixLight light = params.lights[i];
        LightSample ls;
        ls.hitpoint = hit_point;
        ls.wo = -ray_dir;
        ls.n = world_normal;
        // SampleLight(light, &ls);
        CheckVisibleAndSampleLight(params, prd_camera, light, ls);
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

						// Get BRDF model parameter values, with using textures' values if textures exist
						float3 albedo = (mat.kd_tex) ? Pow(GetTexFloat4ValFloat3(mat.kd_tex, mat.tex_scale, uv), 2.2) : mat.Kd; // transfer texture from sRGB space into linear color space
						float roughness = (mat.roughness_tex) ? GetTexValFloat(mat.roughness_tex, mat.tex_scale, uv): mat.roughness;
						float metallic = (mat.metallic_tex) ? GetTexValFloat(mat.metallic_tex, mat.tex_scale, uv) : mat.metallic;
						float3 specular = (mat.ks_tex) ? GetTexFloat4ValFloat3(mat.ks_tex, mat.tex_scale, uv) : mat.Ks; // transfer texture from sRGB space into linear color space
						float opacity = (mat.opacity_tex) ? GetTexValFloat(mat.opacity_tex, mat.tex_scale, uv) : mat.transparency;
						float mat_blend_weight = (mat.weight_tex) ? GetTexValFloat(mat.weight_tex, mat.tex_scale, uv) : (1.f / num_blended_materials);
						float contrib_weight = opacity * mat_blend_weight;  // correct for transparency, light bounces, and blend weight

                        light_reflected_color += in_light_color * prd_camera->contrib_to_pixel * PrincipledBRDF(
							albedo, roughness, metallic, specular, contrib_weight, mat.use_specular_workflow, NdV, NdL, NdH, VdH
						);
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


// -------------- //
// ---- MAIN ---- //
// -------------- //

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
        prd_camera->color += GetAmbientReflectedColor(
			prd_camera, num_blended_materials, material_id, uv, world_normal, ray_dir
		);
    }

    // Add emissive color
    prd_camera->color += (mat.emissive_power * mat.Ke * abs(Dot(world_normal, -ray_dir)));

    // Account for fog
    AddFogEffect(prd_camera, cntxt_params, ray_dist);
    
    // printf("Color: (%.2f,%.2f,%.2f)\n", prd_camera->color.x, prd_camera->color.y, prd_camera->color.z);
    
    // Collect albedo and world_normal for OptiX denoiser for the first hits
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

#endif // CAMERA_LEGACY_SHADER_CU