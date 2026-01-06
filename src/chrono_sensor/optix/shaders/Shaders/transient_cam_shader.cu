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
// Transient camera shader
//
// =============================================================================

#include "chrono_sensor/optix/shaders/device_utils.h"


__device__ __inline__ void SampleBoxPosition(const BoxParameters& box, const float3& sample, PositionSample& ps) {
    // Use sample.z to determine which face to sample
    float face_selector = sample.z * 6.0f;  // [0, 6)
    int face = static_cast<int>(face_selector);

    float u = 2.0f * sample.x - 1.0f;
    float v = 2.0f * sample.y - 1.0f;

    float3 local_pos, normal;

    switch (face) {
        case 0:  // +X face
            local_pos = make_float3(box.lengths.x * 0.5f, box.lengths.y * u * 0.5f, box.lengths.z * v * 0.5f);
            normal = make_float3(1.0f, 0.0f, 0.0f);
            break;
        case 1:  // -X face
            local_pos = make_float3(-box.lengths.x * 0.5f, box.lengths.y * u * 0.5f, box.lengths.z * v * 0.5f);
            normal = make_float3(-1.0f, 0.0f, 0.0f);
            break;
        case 2:  // +Y face
            local_pos = make_float3(box.lengths.x * u * 0.5f, box.lengths.y * 0.5f, box.lengths.z * v * 0.5f);
            normal = make_float3(0.0f, 1.0f, 0.0f);
            break;
        case 3:  // -Y face
            local_pos = make_float3(box.lengths.x * u * 0.5f, -box.lengths.y * 0.5f, box.lengths.z * v * 0.5f);
            normal = make_float3(0.0f, -1.0f, 0.0f);
            break;
        case 4:  // +Z face
            local_pos = make_float3(box.lengths.x * u * 0.5f, box.lengths.y * v * 0.5f, box.lengths.z * 0.5f);
            normal = make_float3(0.0f, 0.0f, 1.0f);
            break;
        case 5:  // -Z face
            local_pos = make_float3(box.lengths.x * u * 0.5f, box.lengths.y * v * 0.5f, -box.lengths.z * 0.5f);
            normal = make_float3(0.0f, 0.0f, -1.0f);
            break;
    }

    // Apply the quaternion rotation to the local position and normal
    float3 rotated_pos = quaternion_rotate(box.rot_quat, local_pos);
    float3 rotated_normal = quaternion_rotate(box.rot_quat, normal);

    // Translate to world position
    ps.pos = box.pos + rotated_pos;
    ps.n = rotated_normal;

    ps.delta = false;
    ps.pdf = 1.f / box.area;
    ps.uv = make_float2(sample.x, sample.y);
}

static __device__ __inline__ void SampleSpherePosition(SphereParameters& sphere, float3& sample, PositionSample& ps) {
    float3 local_pos = square_to_uniform_sphere(sample);
    ps.pos = local_pos * sphere.radius + sphere.pos;
    // printf("SpherePos| lcl: (%f,%f,%f) | c: (%f,%f,%f) | r: %f\n", local_pos.x, local_pos.y, local_pos.z,
    // sphere.pos.x,sphere.pos.y, sphere.pos.z, sphere.radius);
    ps.delta = sphere.radius == 0.f;
    ps.n = local_pos;
    ps.pdf = 1 / sphere.area;
    ps.uv = make_float2(sample.x, sample.y);
}

static __device__ __inline__ void SampleMeshPosition(MeshParameters& mesh, float3& sample, PositionSample& ps) {
    float tri_pdf;
    // Sample a traingle from the mesh
    int tri_id = sample_reuse(mesh.triangleAreaCDFBuffer, mesh.triangleAreaBuffer, &sample.y, mesh.area,
                              mesh.num_triangles, &tri_pdf);

    const uint4& vertex_idx = mesh.vertex_index_buffer[tri_id];

    const float3& v1 = make_float3(mesh.vertex_buffer[vertex_idx.x]);
    const float3& v2 = make_float3(mesh.vertex_buffer[vertex_idx.y]);
    const float3& v3 = make_float3(mesh.vertex_buffer[vertex_idx.z]);

    float3 v2v1 = v2 - v1;
    float3 v3v1 = v3 - v1;
    float2 bc = square_to_uniform_triangle(sample);  // barycentric coord
    ps.pos = v2v1 * bc.x + v3v1 * bc.y + v1;
    ps.delta = false;
    ps.pdf = 1 / mesh.area;

    // calculate normales either from normal buffer or vertex positions
    if (mesh.normal_index_buffer && mesh.normal_buffer) {  // use vertex normals if normal index buffer exists
        const uint4& normal_idx = mesh.normal_index_buffer[tri_id];

        ps.n = normalize(make_float3(mesh.normal_buffer[normal_idx.y]) * bc.x +
                         make_float3(mesh.normal_buffer[normal_idx.z]) * bc.y +
                         make_float3(mesh.normal_buffer[normal_idx.x]) * (1.0f - bc.x - bc.y));

    } else {  // else use face normals calculated from vertices
        ps.n = normalize(Cross(v2 - v1, v3 - v1));
    }

    // calculate texcoords if they exist
    if (mesh.uv_index_buffer && mesh.uv_buffer) {  // use vertex normals if normal index buffer exists
        const uint4& uv_idx = mesh.uv_index_buffer[tri_id];
        const float2& uv1 = mesh.uv_buffer[uv_idx.x];
        const float2& uv2 = mesh.uv_buffer[uv_idx.y];
        const float2& uv3 = mesh.uv_buffer[uv_idx.z];

        ps.uv = uv2 * bc.x + uv3 * bc.y + uv1 * (1.0f - bc.x - bc.y);
    } else {
        ps.uv = bc;
    }
}

static __device__ __inline__ void SampleShapePosition(Shape& shape, float3& sample, PositionSample& ps) {
    switch (shape.type) {
        case ShapeType::SPHERE: {
            SphereParameters sphere = params.sphere_data[shape.index];
            SampleSpherePosition(sphere, sample, ps);
            // printf("Sampling Sphere: pos: (%f,%f,%f)\n", ps.pos.x, ps.pos.y, ps.pos.z);
            break;
        }
        case ShapeType::BOX: {
            BoxParameters box = params.box_data[shape.index];
            SampleBoxPosition(box, sample, ps);
            // printf("Sampling Box: pos: (%f,%f,%f)\n", ps.pos.x, ps.pos.y, ps.pos.z);
            break;
        }
        case ShapeType::MESH: {
            MeshParameters mesh = params.mesh_pool[shape.index];
            SampleMeshPosition(mesh, sample, ps);
            // printf("Sampling Mesh: pos: (%f,%f,%f)\n", ps.pos.x, ps.pos.y, ps.pos.z);
            break;
        }
        default:
            // printf("Error: Invalid Shape Type!\n");
            break;
    }
}

static __device__ __inline__ void SampleHiddenGeometry(PositionSample& ps, float3& sample) {
    float shape_pdf;
    // sample surface area pmf
    int index = sample_reuse(params.surface_area_cdf_buffer, params.surface_area_buffer, &sample.x,
                             params.total_object_area, params.num_hidden_geometry, &shape_pdf);
    // Get shape
    Shape shape = params.shape_info[index];
    SampleShapePosition(shape, sample, ps);
    ps.pdf *= shape_pdf;
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
    if (prd_camera->depth == 2 && params.nlos_laser_sampling) {
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

        float NdLaser = Dot(world_normal, laser_dir);

        if (NdLaser > 0 && fmaxf(prd_shadow.attenuation) > 0 && laser_dist > 1e-8f) {  //
            /* printf("NdLaser: %f, d: (%f,%f,%f), n: (%f,%f,%f)\n", NdLaser, laser_dir.x, laser_dir.y, laser_dir.z,
                    world_normal.x, world_normal.y, world_normal.z);*/
            BSDFSample laser_dir_bsdf;
            laser_dir_bsdf.wo = wo;
            laser_dir_bsdf.wi = laser_dir;
            laser_dir_bsdf.n = world_normal;
            SampleBSDF(bsdfType, laser_dir_bsdf, mat, uv, true);
            // laser_dir_bsdf.pdf = EvalBSDFPDF(bsdfType, wo, laser_dir, laser_dir_bsdf.n);

            float pdf_ls = (laser_dist * laser_dist) / NdLaser;
            laser_dir_bsdf.pdf /= pdf_ls;

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
            // printf("Hit Point: (%f,%f,%f)\n", hit_point.x, hit_point.y, hit_point.z);
            /*    printf("laser dist: %f t: %f | PL: %f |Target: (%f,%f,%f), o: (%f,%f,%f) d: (%f,%f,%f) | Proj HP:
               (%f,%f,%f)\n", laser_dist, optixGetRayTime(), prd.path_length, prd.laser_hitpoint.x,
               prd.laser_hitpoint.y, prd.laser_hitpoint.z, hit_point.x, hit_point.y, hit_point.z, laser_dir.x,
                       laser_dir.y, laser_dir.z,
                       proj_hp.x, proj_hp.y, proj_hp.z);*/
            optixTrace(params.root, hit_point, laser_dir, params.scene_epsilon, laser_dist, laser_dist,
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT, 0, 1, 0, opt1, opt2, raytype);

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
    } else {  // Do standard NEE Direct lighting
        // Uniform sample light
        unsigned int sample_light_index = (unsigned int)(curand_uniform(&prd_camera->rng) * params.num_lights);
        Light l = params.lights[sample_light_index];
        LightSample ls;
        ls.hitpoint = hit_point;
        ls.wo = wo;
        ls.n = world_normal;

        // Compute direct lighting
        Lr = prd_camera->contrib_to_pixel * ComputeDirectLight(l, ls, mat, uv, prd_camera->depth);

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

    if (prd_camera->depth + 1 < params.max_depth) {
        float hg_p = .99;
        bool hg_sample = curand_uniform(&prd_camera->rng) < hg_p;
        float pdf_method = params.nlos_hidden_geometry_sampling ? (hg_sample ? hg_p : 1 - hg_p) : 1.f;
        // pdf_method = 1.f;
        if (prd_camera->depth == 1 && params.nlos_hidden_geometry_sampling &&
            hg_sample) {  // TODO: Rework the logic to be more efficient
            PositionSample ps;
            ps.t = ray_dist;
            float3 next_dir;
            float NdNextDir = 0.f;
            float HgNdNextDir = 0.f;
            float pdf_hg = 0.f;
            bool retry = true;
            int attempts = 0;
            while (retry && attempts < 4) {
                float z1 = curand_uniform(&prd_camera->rng);
                float z2 = curand_uniform(&prd_camera->rng);
                float z3 = curand_uniform(&prd_camera->rng);
                float3 sample = make_float3(z1, z2, z3);
                SampleHiddenGeometry(ps, sample);

                // Trace ray to hidden object
                next_dir = ps.pos - hit_point;
                float dist_next_dir = Length(next_dir);
                next_dir = next_dir / dist_next_dir;

                NdNextDir = Dot(world_normal, next_dir);
                HgNdNextDir = Dot(ps.n, -next_dir);
                pdf_hg = ps.pdf * sqrt(dist_next_dir) / abs(HgNdNextDir);
                retry = (NdNextDir > 0 && HgNdNextDir > 0 && pdf_hg > 0) ? false : true;
                attempts++;
            }

            // Eval BSDF towards HG direction
            BSDFSample bsdf_sample;
            bsdf_sample.wo = wo;
            bsdf_sample.wi = next_dir;
            bsdf_sample.n = world_normal;
            SampleBSDF(bsdfType, bsdf_sample, mat, uv, true);
            bsdf_sample.pdf = pdf_hg;
            bsdf_sample.f = (NdNextDir > 0 && HgNdNextDir > 0 && pdf_hg > 0) ? bsdf_sample.f * NdNextDir / pdf_hg
                                                                             : make_float3(0.f);

            float3 next_contrib_to_pixel = prd_camera->contrib_to_pixel * bsdf_sample.f / pdf_method;
            /*    if (fmaxf(next_contrib_to_pixel) > 0) {
                 printf("hg| d:%d | prev contr: (%f,%f,%f) | next contr:(%f,%f,%f) | cosi: %f | cosg: %f | pspdf: %f
               |pdfhg: %f| f: (%f,%f,%f) %f\n", prd_camera->depth, prd_camera->contrib_to_pixel.x,
                     prd_camera->contrib_to_pixel.y,
                        prd_camera->contrib_to_pixel.z, next_contrib_to_pixel.x, next_contrib_to_pixel.y,
                        next_contrib_to_pixel.z, NdNextDir, HgNdNextDir, ps.pdf, pdf_hg, bsdf_sample.f.x,
               bsdf_sample.f.y, bsdf_sample.f.z);
                 }*/
            if (fmaxf(next_contrib_to_pixel) < 0) {
                prd_camera->depth_reached = prd_camera->depth;
                return;
            }

            //// Check possible rr termination
            // float rr_thresh = .1f;
            // if (fmaxf(next_contrib_to_pixel) < rr_thresh && prd_camera->depth > 3) {
            //    float q = fmaxf((float).05, 1 - fmaxf(next_contrib_to_pixel));
            //    float p = curand_uniform(&prd_camera->rng);
            //    if (p < q)
            //        return;
            //    next_contrib_to_pixel = next_contrib_to_pixel / (1 - q);
            //}

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
            optixTrace(params.root, hit_point, next_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
            L += prd_reflection.color;
            prd_camera->depth_reached = prd_reflection.depth_reached;
        } else {
            BSDFSample sample;
            sample.wo = wo;
            sample.n = world_normal;
            float z1 = curand_uniform(&prd_camera->rng);
            float z2 = curand_uniform(&prd_camera->rng);
            SampleBSDF(bsdfType, sample, mat, uv, false, z1, z2);

            if (luminance(sample.f) > params.importance_cutoff > 0 && sample.pdf > 0) {
                float NdL = Dot(sample.n, sample.wi);
                float3 next_contrib_to_pixel =
                    prd_camera->contrib_to_pixel * sample.f * NdL / (sample.pdf * pdf_method);
                if (fmaxf(next_contrib_to_pixel) < 0) {
                    prd_camera->depth_reached = prd_camera->depth;
                    return;
                }
                //// Check possible rr termination
                // float rr_thresh = .1f;
                // if (fmaxf(next_contrib_to_pixel) < rr_thresh && prd_camera->depth > 3) {
                //     float q = fmaxf((float).05, 1 - fmaxf(next_contrib_to_pixel));
                //     float p = curand_uniform(&prd_camera->rng);
                //     if (p < q) {
                //         prd_camera->depth_reached = prd_camera->depth;
                //         return;
                //     }
                //     next_contrib_to_pixel = next_contrib_to_pixel / (1 - q);
                // }

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
        }
        case TIMEGATED_MODE::TENT: {
            importance = fmaxf(1 - fabsf(v), 0.f);
            break;
        }
        case TIMEGATED_MODE::COS: {
            importance = cosf(2 * CUDART_PI * v);
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
    // printf("TIMEGATED Integrator!\n");
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
        Ld = prd_camera->contrib_to_pixel * ComputeDirectLight(l, ls, mat, uv, prd_camera->depth);

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
        SampleBSDF(bsdfType, sample, mat, uv, false, z1, z2);
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
    const MaterialParameters& mat = params.material_pool[material_id];
    // printf("d: %d | Hitting Object: %hu | contr: (%f,%f,%f)\n", prd_camera->depth, mat.instance_id,
    // prd_camera->contrib_to_pixel.x, prd_camera->contrib_to_pixel.y, prd_camera->contrib_to_pixel.z);
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
        Ld = prd_camera->contrib_to_pixel * ComputeDirectLight(l, ls, mat, uv, prd_camera->depth);

        if (fmaxf(Ld) > 0) {
            TransientSample sample = {};
            sample.pathlength = prd_camera->path_length + ls.dist;
            sample.color = Ld;
            int idx = params.max_depth * prd_camera->current_pixel + (prd_camera->depth - 1);

            params.transient_buffer[idx] = sample;
        }
    }
    L += Ld;

    // TODO:: Add RR HG sampling selection
    if (prd_camera->depth + 1 < params.max_depth) {
        float hg_p = .99;
        bool hg_sample = curand_uniform(&prd_camera->rng) < hg_p;
        float pdf_method = params.nlos_hidden_geometry_sampling ? (hg_sample ? hg_p : 1 - hg_p) : 1.f;
        pdf_method = 1.f;
        if (prd_camera->depth == 1 && params.nlos_hidden_geometry_sampling &&
            hg_sample) {  // TODO: Rework the logic to be more efficient
            PositionSample ps;
            ps.t = ray_dist;
            float3 next_dir;
            float NdNextDir = 0.f;
            float HgNdNextDir = 0.f;
            float pdf_hg = 0.f;
            bool retry = true;
            int attempts = 0;
            while (retry && attempts < 4) {
                float z1 = curand_uniform(&prd_camera->rng);
                float z2 = curand_uniform(&prd_camera->rng);
                float z3 = curand_uniform(&prd_camera->rng);
                float3 sample = make_float3(z1, z2, z3);
                SampleHiddenGeometry(ps, sample);

                // Trace ray to hidden object
                next_dir = ps.pos - hit_point;
                float dist_next_dir = Length(next_dir);
                next_dir = next_dir / dist_next_dir;

                NdNextDir = Dot(world_normal, next_dir);
                HgNdNextDir = Dot(ps.n, -next_dir);
                pdf_hg = ps.pdf * sqrt(dist_next_dir) / abs(HgNdNextDir);
                retry = (NdNextDir > 0 && HgNdNextDir > 0 && pdf_hg > 0) ? false : true;
                // attempts++;
            }

            // Eval BSDF towards HG direction
            BSDFSample bsdf_sample;
            bsdf_sample.wo = wo;
            bsdf_sample.wi = next_dir;
            bsdf_sample.n = world_normal;
            SampleBSDF(bsdfType, bsdf_sample, mat, uv, true);
            bsdf_sample.pdf = pdf_hg;
            bsdf_sample.f = (NdNextDir > 0 && HgNdNextDir > 0 && pdf_hg > 0) ? bsdf_sample.f * NdNextDir / pdf_hg
                                                                             : make_float3(0.f);

            float3 next_contrib_to_pixel = prd_camera->contrib_to_pixel * bsdf_sample.f / pdf_method;
            /*  if (fmaxf(next_contrib_to_pixel) == 0) {
                   printf("\nhg| d:%d | prev contr: (%f,%f,%f) | next contr:(%f,%f,%f) | n: (%f,%f,%f) | psn:(%f,%f,%f)
               | cosi: %f | cosg: %f | pspdf: %f |pdfhg: %f| f: (%f,%f,%f) %f\n", prd_camera->depth,
               prd_camera->contrib_to_pixel.x, prd_camera->contrib_to_pixel.y, prd_camera->contrib_to_pixel.z,
                       next_contrib_to_pixel.x, next_contrib_to_pixel.y,
                       next_contrib_to_pixel.z,
                       world_normal.x, world_normal.y, world_normal.z,
                       ps.n.x,ps.n.y,ps.n.z,
                       NdNextDir, HgNdNextDir, ps.pdf, pdf_hg, bsdf_sample.f.x, bsdf_sample.f.y,
                       bsdf_sample.f.z);
                  printf("hp: (%f,%f,%f) | sp: (%f,%f,%f) | Next Dir: (%f,%f,%f)\n\n", hit_point.x, hit_point.y,
                         hit_point.z, ps.pos.x, ps.pos.y, ps.pos.z, next_dir.x, next_dir.y, next_dir.z);
               }*/

            if (fmaxf(next_contrib_to_pixel) < 0) {
                prd_camera->depth_reached = prd_camera->depth;
                return;
            }

            //// Check possible rr termination
            // float rr_thresh = .1f;
            // if (fmaxf(next_contrib_to_pixel) < rr_thresh && prd_camera->depth > 3) {
            //     float q = fmaxf((float).05, 1 - fmaxf(next_contrib_to_pixel));
            //     float p = curand_uniform(&prd_camera->rng);
            //     if (p < q)
            //         return;
            //     next_contrib_to_pixel = next_contrib_to_pixel / (1 - q);
            // }

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
            optixTrace(params.root, hit_point, next_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
            L += prd_reflection.color;
            prd_camera->depth_reached = prd_reflection.depth_reached;

        } else {
            BSDFSample sample;
            sample.wo = wo;
            sample.n = world_normal;
            float z1 = curand_uniform(&prd_camera->rng);
            float z2 = curand_uniform(&prd_camera->rng);
            SampleBSDF(bsdfType, sample, mat, uv, false, z1, z2);
            if (luminance(sample.f) > params.importance_cutoff && sample.pdf > 0) {
                float NdL = Dot(sample.n, sample.wi);
                float3 next_contrib_to_pixel =
                    prd_camera->contrib_to_pixel * sample.f * NdL / (sample.pdf * pdf_method);
                /*  if (prd_camera->depth == 2 && fmaxf(next_contrib_to_pixel) >=0) {
                      printf(
                          "obj: %d| d:%d | prev contr: (%f,%f,%f) | next contr:(%f,%f,%f) | NdL: %f | bsdf_pdf: %f
                  pdf_method: %f |f: (%f,%f,%f)\n", mat.instance_id, prd_camera->depth, prd_camera->contrib_to_pixel.x,
                  prd_camera->contrib_to_pixel.y, prd_camera->contrib_to_pixel.z, next_contrib_to_pixel.x,
                  next_contrib_to_pixel.y, next_contrib_to_pixel.z, NdL, sample.pdf, pdf_method, sample.f.x, sample.f.y,
                  sample.f.z);
                  }*/
                if (fmaxf(next_contrib_to_pixel) < 0) {
                    prd_camera->depth_reached = prd_camera->depth;
                    return;
                }
                // Check possible rr termination
                float rr_thresh = .1f;
                // if (fmaxf(next_contrib_to_pixel) < rr_thresh && prd_camera->depth > 3) {
                //     float q = fmaxf((float).05, 1 - fmaxf(next_contrib_to_pixel));
                //     float p = curand_uniform(&prd_camera->rng);
                //     if (p < q)
                //         return;
                //     next_contrib_to_pixel = next_contrib_to_pixel / (1 - q);
                // }

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
        }
    } else {
        prd_camera->depth_reached = prd_camera->depth;
    }

    prd_camera->color += L;
    prd_camera->albedo = mat.Kd;  // Might change
    prd_camera->normal = world_normal;
}

static __device__ __inline__ void TransientCamShader(PerRayData_transientCamera* transCam_prd,
                                                     const MaterialRecordParameters* mat_params,
                                                     unsigned int& material_id,
                                                     const unsigned int& num_blended_materials,
                                                     const float3& world_normal,
                                                     const float2& uv,
                                                     const float3& tangent,
                                                     const float& ray_dist,
                                                     const float3& ray_orig,
                                                     const float3& ray_dir) {

    switch (transCam_prd->integrator) {
        case Integrator::TRANSIENT:
            TransientPathIntegrator(transCam_prd, mat_params, material_id, mat_params->num_blended_materials,
                                    world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
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
}