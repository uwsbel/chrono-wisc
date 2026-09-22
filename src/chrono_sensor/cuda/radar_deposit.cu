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
// A fast-chirp radar forms its range-Doppler map by windowing and transforming a dechirped beat
// signal. For a single propagation path that beat signal is a windowed complex exponential, and
// the transform of one of those is known in closed form, so the cube can be written directly:
// each path contributes its window's response centred on the range and Doppler it implies. That
// skips synthesizing and transforming the time series without giving up anything the transform
// would have carried, because phase, Doppler, per-channel array geometry and coherent summation
// all survive.
//
// What it does give up is intra-interval range migration: the range is evaluated once per
// coherent processing interval rather than once per chirp. At automotive closing speeds the
// migration stays well inside one range bin.
//
// =============================================================================

#include <cuda.h>
#include <cuda_runtime.h>

#include "chrono_sensor/cuda/radar_deposit.cuh"

namespace chrono {
namespace sensor {

namespace {

__device__ const float k_pi = 3.14159265359f;
__device__ const float k_two_pi = 6.28318530718f;

/// Dirichlet kernel: the transform of a rectangular window of length n, evaluated off-bin.
__device__ __inline__ float2 Dirichlet(float delta, float n) {
    const float pd = k_pi * delta;
    const float denominator = sinf(pd / n);
    const float magnitude = (fabsf(denominator) < 1e-7f) ? n : sinf(pd) / denominator;
    const float phase = pd * (n - 1.f) / n;
    return make_float2(magnitude * cosf(phase), magnitude * sinf(phase));
}

/// Transform of a raised-cosine window of length n, evaluated at a fractional bin offset. Every
/// window in use is a0 - a1*cos(2*pi*k/n), whose transform is three shifted Dirichlet kernels.
__device__ __inline__ float2 WindowResponse(int window, float delta, float n) {
    float a0 = 1.f;
    float a1 = 0.f;
    if (window == 1) {  // Hann
        a0 = 0.5f;
        a1 = 0.5f;
    } else if (window == 2) {  // Hamming
        a0 = 0.54f;
        a1 = 0.46f;
    }
    const float2 centre = Dirichlet(delta, n);
    if (a1 == 0.f)
        return centre;
    const float2 upper = Dirichlet(delta + 1.f, n);
    const float2 lower = Dirichlet(delta - 1.f, n);
    return make_float2(a0 * centre.x - 0.5f * a1 * (upper.x + lower.x),
                       a0 * centre.y - 0.5f * a1 * (upper.y + lower.y));
}

__device__ __inline__ float2 ComplexMul(const float2& a, const float2& b) {
    return make_float2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
}

__global__ void ClearCubeKernel(RadarComplex* cube, unsigned int count) {
    const unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= count)
        return;
    cube[i].re = 0.f;
    cube[i].im = 0.f;
}

__global__ void DepositKernel(const RadarPath* __restrict__ paths,
                              unsigned int num_paths,
                              RadarComplex* __restrict__ cube,
                              unsigned long long* __restrict__ provenance,
                              RadarDepositParams p) {
    const unsigned int path_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (path_index >= num_paths)
        return;

    const RadarPath path = paths[path_index];

    // A monostatic path length changes at twice the radial velocity of its last surface.
    const float radial_velocity = 0.5f * path.length_rate;
    const float range = 0.5f * path.length;

    // Range-Doppler coupling: differentiating the dechirped phase leaves the carrier term
    // 2*v/lambda alongside the slope term, so a fast target sits a fraction of a bin away from its
    // true range. The sign is that of an up-chirp, which is what this waveform is.
    const float range_bin = range / p.range_bin_size + p.doppler_range_coupling * radial_velocity;
    if (range_bin < -(float)p.taps || range_bin > (float)(p.num_range_bins + p.taps))
        return;

    // Doppler folds, which is the mechanism behind velocity ghosts, so the bin is wrapped rather
    // than discarded.
    const float doppler_span = (float)p.num_doppler_bins;
    float doppler_bin = radial_velocity / p.doppler_bin_size;
    doppler_bin = fmodf(doppler_bin, doppler_span);
    if (doppler_bin >= 0.5f * doppler_span)
        doppler_bin -= doppler_span;
    if (doppler_bin < -0.5f * doppler_span)
        doppler_bin += doppler_span;
    const float doppler_centre = doppler_bin + 0.5f * doppler_span;

    const int taps = (int)min(p.taps, CH_RADAR_MAX_DEPOSIT_TAPS);
    const int range_first = (int)floorf(range_bin) - taps;
    const int doppler_first = (int)floorf(doppler_centre) - taps;
    const int num_taps = 2 * taps + 1;

    // The window responses separate, so they are evaluated once per path and reused for every
    // channel and every tap pair.
    float2 range_taps[2 * CH_RADAR_MAX_DEPOSIT_TAPS + 1];
    float2 doppler_taps[2 * CH_RADAR_MAX_DEPOSIT_TAPS + 1];
    for (int t = 0; t < num_taps; t++) {
        range_taps[t] = WindowResponse(p.range_window, range_bin - (float)(range_first + t),
                                       (float)p.range_transform_length);
        doppler_taps[t] = WindowResponse(p.doppler_window, doppler_centre - (float)(doppler_first + t),
                                         doppler_span);
    }

    // Strongest contributor to the peak cell, kept so a detection can be attributed to the
    // geometry that produced it.
    const int peak_range = (int)floorf(range_bin + 0.5f);
    const int peak_doppler = ((int)floorf(doppler_centre + 0.5f) % (int)p.num_doppler_bins +
                              (int)p.num_doppler_bins) %
                             (int)p.num_doppler_bins;
    if (peak_range >= 0 && peak_range < (int)p.num_range_bins) {
        const unsigned int bounces = path.bounces > 255u ? 255u : path.bounces;
        const unsigned long long key = ((unsigned long long)__float_as_uint(path.amplitude) << 32) |
                                       (unsigned long long)((bounces << 24) | (path.object_id & 0x00FFFFFFu));
        atomicMax(&provenance[peak_doppler * p.num_range_bins + peak_range], key);
    }

    for (unsigned int channel = 0; channel < p.num_channels; channel++) {
        // Far-field array phase. A path's phase is +2*pi*L/lambda, and a virtual element displaced
        // along the direction of arrival sits that much closer to the source, so its path is
        // shorter and its phase lower.
        const float3 element = p.channel_position[channel];
        const float projection = element.x * path.dir_rx.x + element.y * path.dir_rx.y + element.z * path.dir_rx.z;
        float phase = path.phase - k_two_pi * projection / p.wavelength;

        // Under time division multiplexing a transmitter's chirps start one slot late, which
        // shows up as a velocity-dependent phase offset between channels. Leaving it out is what
        // makes uncompensated time division arrays report ghost angles for fast targets.
        phase += p.tdm_slot_phase * radial_velocity * (float)p.channel_slot[channel];

        const float gain = p.channel_gain[channel];
        const float2 phasor = make_float2(path.amplitude * gain * cosf(phase), path.amplitude * gain * sinf(phase));

        RadarComplex* plane = cube + (size_t)channel * p.num_doppler_bins * p.num_range_bins;
        for (int dt = 0; dt < num_taps; dt++) {
            int d = doppler_first + dt;
            d = (d % (int)p.num_doppler_bins + (int)p.num_doppler_bins) % (int)p.num_doppler_bins;
            const float2 slow = ComplexMul(phasor, doppler_taps[dt]);
            for (int rt = 0; rt < num_taps; rt++) {
                const int r = range_first + rt;
                if (r < 0 || r >= (int)p.num_range_bins)
                    continue;
                const float2 cell = ComplexMul(slow, range_taps[rt]);
                RadarComplex* target = plane + (size_t)d * p.num_range_bins + r;
                atomicAdd(&target->re, cell.x);
                atomicAdd(&target->im, cell.y);
            }
        }
    }
}

__global__ void TotalPowerKernel(const RadarPath* __restrict__ paths, unsigned int num_paths, float* total) {
    extern __shared__ float shared[];
    const unsigned int tid = threadIdx.x;
    float sum = 0.f;
    for (unsigned int i = blockIdx.x * blockDim.x + tid; i < num_paths; i += gridDim.x * blockDim.x) {
        const float a = paths[i].amplitude;
        sum += a * a;
    }
    shared[tid] = sum;
    __syncthreads();
    for (unsigned int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s)
            shared[tid] += shared[tid + s];
        __syncthreads();
    }
    if (tid == 0)
        atomicAdd(total, shared[0]);
}

}  // namespace

void radar_clear_cube(RadarComplex* cube, unsigned int count, CUstream stream) {
    const unsigned int threads = 256;
    const unsigned int blocks = (count + threads - 1) / threads;
    ClearCubeKernel<<<blocks, threads, 0, stream>>>(cube, count);
}

void radar_deposit_paths(const RadarPath* paths,
                         unsigned int num_paths,
                         RadarComplex* cube,
                         unsigned long long* provenance,
                         const RadarDepositParams& params,
                         CUstream stream) {
    if (num_paths == 0)
        return;
    const unsigned int threads = 128;
    const unsigned int blocks = (num_paths + threads - 1) / threads;
    DepositKernel<<<blocks, threads, 0, stream>>>(paths, num_paths, cube, provenance, params);
}

void radar_total_path_power(const RadarPath* paths, unsigned int num_paths, float* total, CUstream stream) {
    cudaMemsetAsync(total, 0, sizeof(float), stream);
    if (num_paths == 0)
        return;
    const unsigned int threads = 256;
    const unsigned int blocks = min(1024u, (num_paths + threads - 1) / threads);
    TotalPowerKernel<<<blocks, threads, threads * sizeof(float), stream>>>(paths, num_paths, total);
}

}  // namespace sensor
}  // namespace chrono
