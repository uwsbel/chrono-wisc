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
// Device-side functions related to environment light in OptiX shaders.
//
// =============================================================================

#ifndef CHRONO_SENSOR_OPTIX_ENVIRONMENT_LIGHT_CU
#define CHRONO_SENSOR_OPTIX_ENVIRONMENT_LIGHT_CU

#include "chrono_sensor/optix/shaders/ChOptixLightStructs.h" // for EnvironmentLightData, LightSample
#include "chrono_sensor/optix/ChOptixDefinitions.h" // for PerRayData_camera, ContextParameters
#include "chrono_sensor/optix/shaders/device_utils.h"


/// Binary search: returns smallest idx in [0, n-1] such that cdf[idx] >= threshold.
/// Assumes cdf is non-decreasing and cdf[n-1] == 65535.
static __device__ __inline__ int SampleCDF_u16(const unsigned short* cdf, int n, unsigned short threshold) {
    int low = 0;
    int high = n - 1;
    while (low < high) {
        int mid = (low + high) >> 1;
        // Note: cdf[] values are in [0, 65535]
        if ((unsigned short)cdf[mid] < threshold)
            low = mid + 1;
        else
            high = mid;
    }
    return low;
}

/// Probability mass of bin idx for a quantized CDF in [0, 65535].
static __device__ __inline__ float CDFDelta_u16(const unsigned short* cdf, int idx) {
    unsigned short prev = (idx == 0) ? 0 : (unsigned short)cdf[idx - 1];
    unsigned short curr = (unsigned short)cdf[idx];
    unsigned short d = (curr >= prev) ? (curr - prev) : 0;
    return (float)d / 65535;
}

/// Evaluate environment radiance for a given direction (z-up) using the same mapping as miss.cu.
/// Returns *linear* radiance (gamma corrected like miss.cu) multiplied by intensity_scale.
static __device__ __inline__ float3 EvalEnvironmentRadiance(const EnvironmentLightData& env, const float3& dir) {
    // Mapping consistent with miss.cu:
    // theta = atan2(x, y) in [-pi, pi] -> tex_x in [-0.5, 0.5]
    // phi   = asin(z)      in [-pi/2, pi/2] -> tex_y in [0, 1]
    float theta = atan2f(dir.x, dir.y);
    float phi = asinf(dir.z);

    float tex_x = theta / (2.0f * CUDART_PI_F);
    float tex_y = phi / CUDART_PI_F + 0.5f;

    float4 tex = tex2D<float4>(env.env_map, tex_x, tex_y);

    // Convert texture value to linear like miss.cu and scale
    float3 rgb = make_float3(tex.x, tex.y, tex.z) * env.intensity_scale;
    return rgb;
}

/// Sample an environment direction using two-stage importance sampling.
/// Outputs:
///   dir_out: normalized direction from hit point toward environment
///   pdf_omega_out: PDF in solid angle measure (sr^-1)
///   tex_x_out/tex_y_out: sampled tex coords (optional debugging)
static __device__ __inline__ void SampleEnvironmentDirection(
    const EnvironmentLightData& env,
    PerRayData_camera* prd_camera,
    float3& dir_out,
    float& pdf_omega_out
) {
    const int W = env.width;
    const int H = env.height;

	    // --- 1) Sample latitude v using marginal CDF ---
    float u0 = curand_uniform(&prd_camera->rng);
    unsigned short t0 = static_cast<unsigned short>(u0 * 65535.f);
    int y = SampleCDF_u16(env.dev_cdf_lat, H, t0);
    float p_y = CDFDelta_u16(env.dev_cdf_lat, y);

    // --- 2) Sample longitude column u using conditional CDF for row v ---
    const unsigned short* row_cdf = env.dev_cdf_lon + y * W;
    float u1 = curand_uniform(&prd_camera->rng);
    unsigned short t1 = static_cast<unsigned short>(u1 * 65535.f);
    int x = SampleCDF_u16(row_cdf, W, t1);
    float p_x_given_y = CDFDelta_u16(row_cdf, x);

    // Jitter within the pixel for continuous sampling in UV domain
    float jx = curand_uniform(&prd_camera->rng);  // (0, 1]
    float jy = curand_uniform(&prd_camera->rng);  // (0, 1]

    // IMPORTANT: to match miss.cu's tex_x range [-0.5, 0.5],
    // we generate tex_x directly in that range.
    float tex_x = ((x + jx) / (float)W) - 0.5f; // [-0.5, 0.5]
    float tex_y = ((y + jy) / (float)H) - 0.5f; // [-0.5, 0.5]


    // Convert tex coords to direction:
    // azimuth = 2 * pi * tex_x, in [-pi, pi]
    // elevation = pi * (tex_y - 0.5), in [-pi/2, pi/2]
    float azimuth = fminf(fmaxf(-CUDART_PI_F, 2.0f * CUDART_PI_F * tex_x), CUDART_PI_F);
    float elevation = fminf(fmaxf(-CUDART_PI_F/2.f, CUDART_PI_F * tex_y), CUDART_PI_F/2.f);

    dir_out.x = cosf(azimuth) * cosf(elevation);
	dir_out.y = sinf(azimuth) * cosf(elevation);
	dir_out.z = sinf(elevation);

	// prd_camera->color = make_float3(0.f, 0.f, 1.f); // debug
	// return; // debug

    // --- PDF computation ---
    //
    // We choose discrete (x,y) with probability mass:
    //   p_xy = p_y * p_x|y
    //
    // Then we sample uniformly within that pixel in (tex_x, tex_y) domain:
    //   pixel area in tex domain: (1/W) * (1/H)
    // Therefore pdf_tex = p_xy / (1/(W*H)) = p_xy * W * H
    //
    // Mapping (tex_x, tex_y) -> (theta, phi):
    //   theta = 2*pi*tex_x  => dtheta = 2*pi d(tex_x)
    //   phi   = pi*(tex_y - 0.5) => dphi = pi d(tex_y)
    //
    // Solid angle differential:
    //   dω = cos(phi) dθ dφ
    //      = cos(phi) * (2*pi) * pi * d(tex_x) d(tex_y)
    //      = 2*pi^2 * cos(phi) * d(tex_x) d(tex_y)
    //
    // Therefore:
    //   pdf_omega = pdf_tex / (2* pi^2 * cos(phi))
    //
    float p_xy = p_y * p_x_given_y;
    float pdf_tex = p_xy * (float)(W * H);

    float denom = 2.0f * CUDART_PI_F * CUDART_PI_F * fmaxf(cosf(elevation), 1e-6f);
    float pdf_omega = pdf_tex / denom;

    pdf_omega_out = pdf_omega;

	
}


/// @brief Check visibility between the hit point along the environment light direction, and sample the light.
/// @param cntxt_params context parameters
/// @param light_data environment light data
/// @param light_sample the light sample to be updated
/// @return True if the light is visible from the hit point, false otherwise
static __device__ __inline__ bool CheckVisibleAndSampleEnvironmentLight(
	const ContextParameters& cntxt_params,
    const EnvironmentLightData& light_data,
    LightSample& light_sample,
    PerRayData_camera* prd_camera
) {

	// if (/*!light_data.env_map ||*/ !light_data.dev_cdf_lon) {
	// 	light_sample.pdf = 1.f;
	// 	light_sample.L = make_float3(0.f, 0.f, 1.f);
	// 	return true;
	// }


    // Sample direction and probabilidy density function (PDF) from the environment map using two-stage importance sampling
    // float3 dir = make_float3(0.f, 0.f, 0.f);
    float pdf_omega = 0.f;
    SampleEnvironmentDirection(light_data, prd_camera, light_sample.dir, pdf_omega);

	// return true;

    // Update geometric terms
    light_sample.dist = 1e16f;  // "infinite" distance
    light_sample.NdL = Dot(light_sample.n, light_sample.dir);

    // Light is below the surface
    if (light_sample.NdL <= 0.f || pdf_omega <= 0.f) {
        light_sample.L = make_float3(0.f, 0.f, 0.f);
        light_sample.pdf = 0.f;
        return false;
    }

    // Trace shadow ray toward the sampled direction to check for occlusion
    PerRayData_occlusion prd_occ;
    prd_occ.occluded = false;

    unsigned int opt1, opt2;
    pointer_as_ints(&prd_occ, opt1, opt2);

    unsigned int raytype = (unsigned int)RayType::OCCLUSION_RAY_TYPE;

    optixTrace(
        cntxt_params.root,
        light_sample.hitpoint,
        light_sample.dir,
        cntxt_params.scene_epsilon,
        1e16f,
        optixGetRayTime(),
        OptixVisibilityMask(1),
        OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT,
        0,
        1,
        0,
        opt1,
        opt2,
        raytype
    );

    // If occluded, no contribution
    if (prd_occ.occluded) {
        light_sample.L = make_float3(0.f, 0.f, 0.f);
        light_sample.pdf = 0.f;
        return false;
    }

    // Evaluate environment radiance along sampled direction
    float3 Le = EvalEnvironmentRadiance(light_data, light_sample.dir);

    // Final contribution (Lambertian shading term uses NdL)
    // NOTE: your shading pipeline uses light_sample.L as "already multiplied by NdL" for other lights.
    light_sample.L = light_sample.NdL * Le;
    light_sample.pdf = pdf_omega;

    return true;
}
	
/*	
	const ContextParameters& cntxt_params,
	const EnvironmentLightData& light_data,
	LightSample& light_sample
	// const PerRayData_camera* prd_camera // debug
) {
	// Direction and distance from hit-point to light
	light_sample.dir = light_data.light_dir;
	light_sample.NdL = Dot(light_sample.n, light_sample.dir);
	
	// Light is below the surface
	if (light_sample.NdL < 0) {
		light_sample.L = make_float3(0.f, 0.f, 0.f);
		light_sample.pdf = 0.f;
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
		1e16f,						// A very large max distance (effectively “infinite” for the scene scale)
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
	
	// If the light is occluded
	if (prd_occ.occluded) {
		light_sample.L = make_float3(0.f, 0.f, 0.f);
		light_sample.pdf = 0.f;
		return false;
	}
	// Caculate the remaining attributes of light sample
	else {
		light_sample.L = light_sample.NdL * light_data.color;
		light_sample.pdf = 1.0f; // Delta light
		return true;
	}
}
*/

#endif  // CHRONO_SENSOR_OPTIX_ENVIRONMENT_LIGHT_CU