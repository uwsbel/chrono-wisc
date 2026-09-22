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

#include <cuda.h>
#include <cuda_runtime.h>

#include "chrono_sensor/cuda/radar_impair.cuh"

namespace chrono {
namespace sensor {

namespace {

__device__ const float k_pi = 3.14159265359f;
__device__ const float k_c = 299792458.f;

__global__ void ThermalNoiseKernel(RadarComplex* cube,
                                   unsigned int count,
                                   float quadrature_sigma,
                                   curandState_t* states,
                                   unsigned int num_states) {
    const unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_states)
        return;
    curandState_t rng = states[tid];
    for (unsigned int i = tid; i < count; i += num_states) {
        cube[i].re += quadrature_sigma * curand_normal(&rng);
        cube[i].im += quadrature_sigma * curand_normal(&rng);
    }
    states[tid] = rng;
}

__global__ void PhaseNoiseKernel(const float* __restrict__ power_map,
                                 float* __restrict__ out_power_map,
                                 RadarPhaseNoiseParams p) {
    const unsigned int r = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int d = blockIdx.y * blockDim.y + threadIdx.y;
    if (r >= p.num_range_bins || d >= p.num_doppler_bins)
        return;

    const size_t row = (size_t)d * p.num_range_bins;
    float skirt = 0.f;
    const int half = (int)p.skirt_half_width;
    for (int offset = -half; offset <= half; offset++) {
        if (offset == 0)
            continue;
        const int source = (int)r + offset;
        if (source < 0 || source >= (int)p.num_range_bins)
            continue;

        const float frequency = fabsf((float)offset) * p.beat_frequency_per_bin;
        const float delay = 2.f * (float)source * p.range_bin_size / k_c;

        // Single sideband density at this offset, extrapolated from the reference point along the
        // configured roll-off, then suppressed by the range correlation of the dechirping mixer.
        const float decades = __log10f(frequency / p.reference_offset);
        const float density = p.level_at_reference * __powf(10.f, 0.1f * p.decade_slope * decades);
        const float correlation = 4.f * __sinf(k_pi * frequency * delay) * __sinf(k_pi * frequency * delay);

        skirt += power_map[row + source] * density * correlation * p.beat_frequency_per_bin;
    }
    out_power_map[row + r] = power_map[row + r] + skirt;
}

__global__ void UniformFloorKernel(float* power_map, unsigned int count, float floor_power) {
    const unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= count)
        return;
    power_map[i] += floor_power;
}

}  // namespace

void radar_add_thermal_noise(RadarComplex* cube,
                             unsigned int count,
                             float noise_power,
                             curandState_t* states,
                             unsigned int num_states,
                             CUstream stream) {
    if (count == 0 || num_states == 0)
        return;
    const float quadrature_sigma = sqrtf(0.5f * noise_power);
    const unsigned int threads = 256;
    const unsigned int blocks = (num_states + threads - 1) / threads;
    ThermalNoiseKernel<<<blocks, threads, 0, stream>>>(cube, count, quadrature_sigma, states, num_states);
}

void radar_add_phase_noise_skirt(const float* power_map,
                                 float* out_power_map,
                                 const RadarPhaseNoiseParams& params,
                                 CUstream stream) {
    const dim3 threads(32, 8);
    const dim3 blocks((params.num_range_bins + threads.x - 1) / threads.x,
                      (params.num_doppler_bins + threads.y - 1) / threads.y);
    PhaseNoiseKernel<<<blocks, threads, 0, stream>>>(power_map, out_power_map, params);
}

void radar_add_uniform_floor(float* power_map, unsigned int count, float floor_power, CUstream stream) {
    if (count == 0)
        return;
    const unsigned int threads = 256;
    const unsigned int blocks = (count + threads - 1) / threads;
    UniformFloorKernel<<<blocks, threads, 0, stream>>>(power_map, count, floor_power);
}

}  // namespace sensor
}  // namespace chrono
