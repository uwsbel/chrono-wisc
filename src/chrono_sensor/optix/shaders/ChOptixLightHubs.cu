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
// Authors: Bo-Hsun Chen
// =============================================================================
//
// Device-side light class base for different types of lights in OptiX shaders.
//
// =============================================================================

#ifndef CHRONO_SENSOR_OPTIX_LIGHT_HUBS_CU
#define CHRONO_SENSOR_OPTIX_LIGHT_HUBS_CU

#include "chrono_sensor/optix/shaders/ChOptixLightStructs.h"
#include "chrono_sensor/optix/ChOptixDefinitions.h"
#include "chrono_sensor/optix/shaders/ChOptixPointLight.cu"

// Check if the light source is visible to the hit-point
static __device__ __inline__ bool CheckVisibleAndSampleLight(
	const ContextParameters& cntxt_params, PerRayData_camera* prd_camera, const ChOptixLight& light, LightSample& light_sample
) {
	switch (light.light_type) {
        case LightType::POINT_LIGHT:
            bool flag = CheckVisibleAndSamplePointLight(
				cntxt_params, prd_camera, light.specific.point, light.pos, light_sample
			);
			// prd_camera->color = {flag * 1.0, 0.0, 1.0}; // debug
			return flag;
            break;
        case LightType::SPOT_LIGHT:
            // return CheckSpotLightVisible(cntxt_params, light, light_sample); // TODO
            // break;
		case LightType::DIRECTIONAL_LIGHT:
            // return CheckDirectionalLightVisible(cntxt_params, light, light_sample); // TODO
            // break;
		case LightType::RECTANGLE_LIGHT:
			// return CheckRectangleLightVisible(cntxt_params, light, light_sample); // TODO
			// break;
		case LightType::DISK_LIGHT:
			// return CheckDiskLightVisible(cntxt_params, light, light_sample); // TODO
			// break;
		case LightType::ENVIRONMENT_LIGHT:
			// return CheckEnvironmentLightVisible(cntxt_params, light, light_sample); // TODO
			// break;
		//// ---- Register Your Custom Light Type Here (check visibility and sample light) ---- ////
		default:
			printf("Light type not recognized in visibility check ...... \n");
			return false;
            break;
    }
}

# endif // CHRONO_SENSOR_OPTIX_LIGHT_HUBS_CU