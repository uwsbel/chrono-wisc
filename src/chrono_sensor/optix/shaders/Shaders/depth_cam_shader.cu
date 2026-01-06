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
// Depth camera shader
//
// =============================================================================

#include "chrono_sensor/optix/shaders/device_utils.h"

static __device__ __inline__ void DepthCamShader(PerRayData_depthCamera* prd, const float& ray_dist) {
    prd->depth = fminf(prd->max_depth, ray_dist);
}