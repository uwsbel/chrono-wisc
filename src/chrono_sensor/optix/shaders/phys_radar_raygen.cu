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
// Propagation stage of the wave-domain radar.
//
// Each launched ray walks the scene under geometric optics and, at every surface it meets,
// estimates the next event back to the antenna. A visible surface therefore contributes one
// coherent path per bounce depth, carrying the quantities the signal chain needs and nothing
// else: total path length, its rate of change, a voltage amplitude and a phase.
//
// Amplitudes follow the bistatic radar equation applied to the patch a ray tube subtends. Writing
// the patch radar cross section as sigma = sigma0 * dA with dA = solid_angle * L^2 / cos(theta_i)
// makes the accumulated length cancel, leaving
//
//   a = T * lambda * sqrt(P_t * G_tx * G_rx * sigma0 / cos(theta_i) * solid_angle)
//           / ( (4*pi)^1.5 * R_out )
//
// with T the product of the field reflection coefficients along the way and R_out the distance
// from the last surface back to the antenna. Summing this coherently over the rays that cover a
// target reproduces the target's radar cross section; a flat plate recovers its 4*pi*A^2/lambda^2
// because the per-tube lobes add in phase.
//
// =============================================================================

#include "chrono_sensor/optix/shaders/device_utils.cuh"
#include "chrono_sensor/optix/ChOptixDefinitions.h"

using chrono::sensor::RadarMaterial;
using chrono::sensor::RadarPath;

namespace {

__device__ const float k_four_pi_pow_1p5 = 44.5462f;  // (4*pi)^1.5
__device__ const float k_four_pi = 12.5663706144f;
__device__ const float k_two_pi = 6.28318530718f;

__device__ __inline__ PerRayData_phys_radar DefaultPhysRadarPRD() {
    PerRayData_phys_radar prd;
    prd.distance = -1.f;
    prd.normal = make_float3(0.f, 0.f, 1.f);
    prd.velocity = make_float3(0.f, 0.f, 0.f);
    prd.metallic = 0.f;
    prd.roughness = 1.f;
    prd.class_id = 0;
    prd.object_id = 0.f;
    return prd;
}

/// Separable cos^n antenna pattern, clipped to the forward hemisphere.
__device__ __inline__ float AntennaGain(float peak, float az_exponent, float el_exponent, float az, float el) {
    const float ca = cosf(az);
    const float ce = cosf(el);
    if (ca <= 0.f || ce <= 0.f)
        return 0.f;
    return peak * powf(ca, az_exponent) * powf(ce, el_exponent);
}

/// Radar response of a surface.
///
/// A scene that tags its visual materials with class ids gets the registered response. One that
/// does not still behaves sensibly: metals reflect coherently, rough dielectrics scatter, and the
/// visual roughness channel sets the width of the specular lobe.
__device__ __inline__ RadarMaterial LookupMaterial(const PhysRadarParameters& radar,
                                                   const PerRayData_phys_radar& prd) {
    if (prd.class_id < radar.num_materials && radar.materials[prd.class_id].assigned)
        return radar.materials[prd.class_id];
    if (radar.fallback_material.assigned)
        return radar.fallback_material;

    RadarMaterial m;
    const float metallic = clamp(prd.metallic, 0.f, 1.f);
    const float roughness = clamp(prd.roughness, 0.f, 1.f);
    const float reflectivity = 0.04f + 0.94f * metallic;
    m.specular_reflectivity = reflectivity * (1.f - roughness);
    m.diffuse_reflectivity = reflectivity * roughness * 0.5f;
    m.lobe_width = 0.02f + roughness * roughness;
    m.transmission = 0.f;
    m.assigned = 1;
    return m;
}

/// True when anything blocks the straight line from a surface point back to the antenna.
__device__ __inline__ bool Occluded(const float3& from, const float3& dir, float distance, float ray_time) {
    PerRayData_occlusion prd;
    prd.occluded = false;
    unsigned int opt1;
    unsigned int opt2;
    pointer_as_ints(&prd, opt1, opt2);
    unsigned int raytype = (unsigned int)RayType::OCCLUSION_RAY_TYPE;
    optixTrace(params.root, from, dir, params.scene_epsilon, distance, ray_time, OptixVisibilityMask(1),
               OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT, 0u, 1u, 0u, opt1, opt2, raytype);
    return prd.occluded;
}

/// Append one path, or account for it as dropped when the buffer is full.
__device__ __inline__ void EmitPath(const PhysRadarParameters& radar, const RadarPath& path) {
    const unsigned int slot = atomicAdd(&radar.path_counter[0], 1u);
    if (slot < radar.max_paths)
        radar.paths[slot] = path;
    else
        atomicAdd(&radar.path_counter[1], 1u);
}

}  // namespace

extern "C" __global__ void __raygen__phys_radar() {
    const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
    const PhysRadarParameters& radar = raygen->specific.phys_radar;

    const uint3 idx = optixGetLaunchIndex();
    const uint3 dim = optixGetLaunchDimensions();
    const unsigned int ray_index = dim.x * idx.y + idx.x;

    curandState_t rng = radar.rng_buffer[ray_index];

    // One stratified sample inside this ray's cell of the field of view. Stratification keeps the
    // sampling noise from being confused with speckle, which is a real effect the model must
    // reproduce rather than manufacture.
    const float u = (idx.x + curand_uniform(&rng)) / (float)dim.x;
    const float v = (idx.y + curand_uniform(&rng)) / (float)dim.y;
    const float launch_az = (u - 0.5f) * radar.hFOV;
    const float launch_el = (v - 0.5f) * radar.vFOV;

    // The radar takes a single kinematic snapshot per cycle and carries motion analytically as a
    // rate of change of path length, so the whole coherent interval is traced at one instant.
    const float ray_time = raygen->t0;
    float3 forward, left, up;
    basis_from_quaternion(raygen->rot0, forward, left, up);
    const float3 antenna = raygen->pos0;

    const float cos_el = cosf(launch_el);
    float3 dir = normalize(forward * (cos_el * cosf(launch_az)) + left * (cos_el * sinf(launch_az)) +
                           up * sinf(launch_el));

    const float tx_gain = AntennaGain(radar.tx_gain, radar.tx_az_exponent, radar.tx_el_exponent, launch_az, launch_el);
    if (tx_gain <= 0.f) {
        radar.rng_buffer[ray_index] = rng;
        return;
    }

    const float ray_angular_step = sqrtf(radar.ray_solid_angle);
    const float max_path_length = 2.f * radar.max_range;

    float3 origin = antenna;
    double length_in = 0.0;     // path length from the antenna to the current surface
    float length_rate_in = 0.f; // its rate of change
    float throughput = 1.f;     // product of the field reflection coefficients so far
    float3 upstream_velocity = radar.velocity;

    for (unsigned int bounce = 0; bounce < radar.max_bounces; bounce++) {
        const float budget = max_path_length - (float)length_in;
        if (budget <= radar.min_range)
            break;

        PerRayData_phys_radar prd = DefaultPhysRadarPRD();
        unsigned int opt1;
        unsigned int opt2;
        pointer_as_ints(&prd, opt1, opt2);
        unsigned int raytype = (unsigned int)RayType::PHYS_RADAR_RAY_TYPE;
        optixTrace(params.root, origin, dir, params.scene_epsilon, budget, ray_time, OptixVisibilityMask(1),
                   OPTIX_RAY_FLAG_NONE, 0u, 1u, 0u, opt1, opt2, raytype);

        if (prd.distance < 0.f)
            break;

        const float3 hit = origin + dir * prd.distance;
        length_in += (double)prd.distance;
        length_rate_in += Dot(prd.velocity - upstream_velocity, dir);

        float3 n = normalize(prd.normal);
        if (Dot(n, dir) > 0.f)
            n = -n;
        const float cos_incidence = fmaxf(-Dot(n, dir), 1e-3f);

        const RadarMaterial mat = LookupMaterial(radar, prd);

        // --- Next event estimation: the coherent return this surface sends back to the antenna.
        const float3 to_antenna = antenna - hit;
        const float out_distance = Length(to_antenna);
        const float total_length = (float)length_in + out_distance;
        if (out_distance > radar.min_range && total_length <= max_path_length) {
            const float3 out_dir = to_antenna / out_distance;
            const float cos_scatter = Dot(n, out_dir);
            if (cos_scatter > 0.f) {
                const float3 shading_origin = hit + n * (params.scene_epsilon * 8.f);
                if (!Occluded(shading_origin, out_dir, out_distance - params.scene_epsilon * 16.f, ray_time)) {
                    // Direction of arrival in the sensor frame, which is what sets the receive
                    // pattern here and the per-channel phases downstream.
                    const float3 arrival = make_float3(Dot(forward, -out_dir), Dot(left, -out_dir), Dot(up, -out_dir));
                    const float arrival_az = atan2f(arrival.y, arrival.x);
                    const float arrival_el = asinf(clamp(arrival.z, -1.f, 1.f));
                    const float rx_gain = AntennaGain(radar.rx_gain, radar.rx_az_exponent, radar.rx_el_exponent,
                                                      arrival_az, arrival_el);
                    if (rx_gain > 0.f) {
                        // Bistatic scattering density, divided through by cos(theta_i) as the
                        // amplitude expression wants it. The coherent lobe is Gaussian about the
                        // mirror direction and is never narrower than the diffraction limit of the
                        // patch this ray tube covers, nor than the ray grid can resolve.
                        const float3 mirror = reflect(dir, n);
                        const float alpha = acosf(clamp(Dot(mirror, out_dir), -1.f, 1.f));
                        const float patch_area =
                            radar.ray_solid_angle * (float)(length_in * length_in) / cos_incidence;
                        const float diffraction_width = radar.wavelength * rsqrtf(k_two_pi * fmaxf(patch_area, 1e-12f));
                        const float lobe_width =
                            fmaxf(fmaxf(mat.lobe_width, diffraction_width), ray_angular_step);
                        const float lobe =
                            __expf(-0.5f * alpha * alpha / (lobe_width * lobe_width)) / (k_two_pi * lobe_width * lobe_width);
                        const float sigma_over_cos_i =
                            4.f * mat.diffuse_reflectivity * cos_scatter + k_four_pi * mat.specular_reflectivity * lobe;

                        const float amplitude =
                            throughput * radar.wavelength *
                            sqrtf(radar.transmit_power * tx_gain * rx_gain * sigma_over_cos_i * radar.ray_solid_angle) /
                            (k_four_pi_pow_1p5 * out_distance);

                        if (amplitude > radar.amplitude_cutoff) {
                            // Phase is reduced in double precision before it is rounded: at 300 m a
                            // float path length is already coarser than a tenth of a wavelength.
                            const double cycles = (length_in + (double)out_distance) / (double)radar.wavelength;
                            RadarPath path;
                            path.length = total_length;
                            path.length_rate =
                                length_rate_in + Dot(radar.velocity - prd.velocity, out_dir);
                            path.amplitude = amplitude;
                            path.phase = (float)((cycles - floor(cycles)) * (double)k_two_pi);
                            path.dir_rx = arrival;
                            path.bounces = bounce + 1;
                            path.object_id = (unsigned int)prd.object_id;
                            path.flags = 0;
                            EmitPath(radar, path);
                        }
                    }
                }
            }
        }

        // --- Continuation. A thin dielectric either transmits or reflects, chosen at random with
        // the branch probability set to the power split, so the surviving ray carries no weight
        // beyond the reflection coefficient itself.
        if (mat.transmission > 0.f && curand_uniform(&rng) < mat.transmission) {
            origin = hit - n * (params.scene_epsilon * 8.f);
        } else {
            const float reflected_fraction = fmaxf(1.f - mat.transmission, 1e-6f);
            throughput *= sqrtf(mat.specular_reflectivity / reflected_fraction);
            if (throughput <= 0.f)
                break;
            dir = reflect(dir, n);
            origin = hit + n * (params.scene_epsilon * 8.f);
        }
        upstream_velocity = prd.velocity;
    }

    radar.rng_buffer[ray_index] = rng;
}
