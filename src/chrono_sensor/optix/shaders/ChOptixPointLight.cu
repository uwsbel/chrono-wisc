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
// Authors: Nevindu M. Batagoda, Bo-Hsun Chen
// =============================================================================
//
// Device-side functions related to point light in OptiX shaders.
//
// =============================================================================

#ifndef CHRONO_SENSOR_OPTIX_POINT_LIGHT_CU
#define CHRONO_SENSOR_OPTIX_POINT_LIGHT_CU

#include "chrono_sensor/optix/shaders/ChOptixLightStructs.h" // for PointLightData, LightSample
#include "chrono_sensor/optix/ChOptixDefinitions.h" // for PerRayData_camera, ContextParameters
#include "chrono_sensor/optix/shaders/device_utils.h"


	/// @brief Check visibility between the point light and the hit point, and sample the light.
	/// @param cntxt_params context parameters
	/// @param prd_camera per-ray data (PRD) of the camera ray
	/// @param light_sample the light sample to be updated
	/// @return True if the light is visible from the hit point, false otherwise
	static __device__ __inline__ bool CheckVisibleAndSamplePointLight(
		const ContextParameters& cntxt_params,
		const PerRayData_camera* prd_camera,
		const PointLightData& light_data,
		const float3& light_posi,
		LightSample& light_sample
	) {
		// Direction and distance from hit-point to light
		light_sample.dir = light_posi - light_sample.hitpoint;
		light_sample.dist = Length(light_sample.dir);
		light_sample.dir = light_sample.dir / light_sample.dist;
		light_sample.NdL = Dot(light_sample.n, light_sample.dir);
		
		// Light is below the surface
		if (light_sample.NdL < 0) {
			// light_sample.L = {0.f, 0.f, 0.f};
			return false;  
		}

		// Trace shadow ray toward the light to check for occlusion		
		PerRayData_occlusion prd_occ;
		prd_occ.occluded = false;

		unsigned int opt1;
		unsigned int opt2;
		pointer_as_ints(&prd_occ, opt1, opt2);

		// Payload 2: ray type (if your code uses it)
		unsigned int raytype = (unsigned int)RayType::OCCLUSION_RAY_TYPE;

		optixTrace(
			cntxt_params.root,          // The scene traversable handle (OptixTraversableHandle); basically the top-level acceleration structure (TLAS).
			light_sample.hitpoint, 		// origin of the traced ray
			light_sample.dir,          	// direction of the traced ray
			cntxt_params.scene_epsilon, // minimum intersection distance to avoid self-intersection (“shadow acne”)
			light_sample.dist,			// A very large max distance (effectively “infinite” for the scene scale)
			optixGetRayTime(),          // time value for launching this ray
			OptixVisibilityMask(1),     // Only intersects geometry whose instance mask matches 1
			OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT, // terminate on first hit is ideal for occlusion rays
			0,                          // SBT offset (used when you have multiple SBT records for the same ray type). It selects the first “ray type slot”
			1,                          // SBT stride (used when you have multiple SBT records for the same ray type). It usually means “one ray type stride”
			0,                          // missSBTIndex. It selects the first miss program
			opt1,                       // Final payloads; the per-ray data pointer (first 32 bits); optixGetPayload_0() = opt1
			opt2,                       // Final payloads; the per-ray data pointer (second 32 bits); optixGetPayload_1() = opt2
			raytype                     // The ray type index (used when you have multiple ray types, e.g., radiance rays, shadow rays, etc.)
		);
		
		// If the light is occluded or the light intensity is negligible		
		if (fmaxf(light_sample.L) < 1e-6f || prd_occ.occluded) {
			light_sample.L = {0.f, 0.f, 0.f};
			return false;
		}
		// Caculate the remaining attributes of light sample
		else {
			float intense_amount = (light_data.const_color) ? 1.0f : (light_data.atten_scale / (light_sample.dist * light_sample.dist)); // inverse square law
			light_sample.L = light_sample.NdL * intense_amount * light_data.color; 
			// light_sample.L = light_sample.NdL * light_data.color * (light_data.max_range * light_data.max_range / (light_sample.dist * light_sample.dist + light_data.max_range * light_data.max_range));
			light_sample.pdf = 1.0f; // Delta light
			return true;
		}
	}

#endif  // CHRONO_SENSOR_OPTIX_POINT_LIGHT_CU