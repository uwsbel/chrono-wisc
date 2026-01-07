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
// LiDAR shader
//
// =============================================================================

#include "chrono_sensor/optix/shaders/device_utils.h"

static __device__ __inline__ void LidarShader(PerRayData_lidar* prd_lidar,
                                              const MaterialParameters& mat,
                                              const float3& world_normal,
                                              const float& ray_dist,
                                              const float3& ray_dir) {
    prd_lidar->range = ray_dist;
    prd_lidar->intensity = mat.lidar_intensity * abs(Dot(world_normal, -ray_dir));
}

