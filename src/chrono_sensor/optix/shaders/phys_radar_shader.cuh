// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Patrick Chen
// =============================================================================
//
// Hit shader for the wave-domain radar. It reports the surface that was found and nothing
// else: the bounce walk, the visibility test back to the antenna and the amplitude bookkeeping
// all live in phys_radar_raygen.cu, where the accumulated path state is.
//
// =============================================================================

#ifndef PHYS_RADAR_SHADER_CUH
#define PHYS_RADAR_SHADER_CUH

#include "chrono_sensor/optix/shaders/device_utils.cuh"

__device__ __inline__ PerRayData_phys_radar* GetPhysRadarPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_phys_radar*>(ints_as_pointer(opt0, opt1));
}

static __device__ __inline__ void PhysRadarShader(PerRayData_phys_radar* prd,
                                                  const MaterialParameters& mat,
                                                  const float3& world_normal,
                                                  const float& ray_dist,
                                                  const float3& ray_orig,
                                                  const float3& ray_dir,
                                                  const float3& translational_velocity,
                                                  const float3& angular_velocity,
                                                  const float& objectId) {
    prd->distance = ray_dist;
    prd->normal = world_normal;

    // Surface velocity at the hit, rigid body style. Wheel spin and suspension travel therefore
    // reach the Doppler estimate without any extra plumbing, which is where micro-Doppler
    // signatures come from.
    const float3 hit_point = ray_orig + ray_dir * ray_dist;
    const float3 body_origin = optixTransformPointFromObjectToWorldSpace(make_float3(0.f, 0.f, 0.f));
    prd->velocity = translational_velocity + Cross(angular_velocity, hit_point - body_origin);

    prd->metallic = mat.metallic;
    prd->roughness = mat.roughness;
    prd->class_id = mat.class_id;
    prd->object_id = objectId;
}

#endif  // PHYS_RADAR_SHADER_CUH
