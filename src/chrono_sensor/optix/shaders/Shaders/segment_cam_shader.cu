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
// Segmentation camera shader
//
// =============================================================================

#include "chrono_sensor/optix/shaders/device_utils.h"

static __device__ __inline__ void SegmentCamShader(PerRayData_segment* prd, const MaterialParameters& mat) {
    prd->class_id = mat.class_id;
    prd->instance_id = mat.instance_id;
}