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

#include "chrono_sensor/cuda/radar_dsp.cuh"

namespace chrono {
namespace sensor {

namespace {

__device__ const float k_two_pi = 6.28318530718f;

__device__ __inline__ float clamp(float v, float low, float high) {
    return fminf(fmaxf(v, low), high);
}

__device__ __inline__ float AntennaGain(float peak, float az_exponent, float el_exponent, float az, float el) {
    const float ca = cosf(az);
    const float ce = cosf(el);
    if (ca <= 0.f || ce <= 0.f)
        return 0.f;
    return peak * powf(ca, az_exponent) * powf(ce, el_exponent);
}

__global__ void SteeringKernel(const float3* __restrict__ positions,
                               unsigned int num_channels,
                               float wavelength,
                               float azimuth_min,
                               float azimuth_step,
                               unsigned int num_azimuth_bins,
                               float2* __restrict__ steering) {
    const unsigned int beam = blockIdx.x * blockDim.x + threadIdx.x;
    if (beam >= num_azimuth_bins)
        return;
    const float azimuth = azimuth_min + beam * azimuth_step;
    // Beams are formed on a uniform grid in angle rather than in its sine, so the map reads
    // directly as a bearing and grating lobes show up where the array actually puts them.
    const float3 direction = make_float3(cosf(azimuth), sinf(azimuth), 0.f);
    for (unsigned int c = 0; c < num_channels; c++) {
        const float3 p = positions[c];
        // Conjugate of the phase the deposition applied, so a path arriving from this bearing
        // adds in phase across the array.
        const float phase =
            k_two_pi * (p.x * direction.x + p.y * direction.y + p.z * direction.z) / wavelength;
        steering[beam * num_channels + c] = make_float2(cosf(phase), sinf(phase));
    }
}

__global__ void BeamformKernel(const RadarComplex* __restrict__ cube,
                               const float2* __restrict__ steering,
                               float* __restrict__ angle_power,
                               float* __restrict__ power_map,
                               RadarDspParams p) {
    const unsigned int r = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int d = blockIdx.y * blockDim.y + threadIdx.y;
    if (r >= p.num_range_bins || d >= p.num_doppler_bins)
        return;

    const size_t cell = (size_t)d * p.num_range_bins + r;
    const size_t plane = (size_t)p.num_doppler_bins * p.num_range_bins;

    // Channels are few, so the cube is read once into registers and every beam reuses it.
    float2 channel[32];
    const unsigned int n = p.num_channels < 32u ? p.num_channels : 32u;
    for (unsigned int c = 0; c < n; c++) {
        const RadarComplex v = cube[c * plane + cell];
        channel[c] = make_float2(v.re, v.im);
    }

    const float normalization = 1.f / (float)n;
    float best = 0.f;
    for (unsigned int beam = 0; beam < p.num_azimuth_bins; beam++) {
        const float2* w = steering + beam * p.num_channels;
        float re = 0.f;
        float im = 0.f;
        for (unsigned int c = 0; c < n; c++) {
            re += channel[c].x * w[c].x - channel[c].y * w[c].y;
            im += channel[c].x * w[c].y + channel[c].y * w[c].x;
        }
        const float power = (re * re + im * im) * normalization;
        angle_power[(size_t)beam * plane + cell] = power;
        best = fmaxf(best, power);
    }
    power_map[cell] = best;
}

/// Number of training values at or below a threshold, used by the ordered statistic detector.
__device__ __inline__ unsigned int CountBelow(const float* __restrict__ power_map,
                                              const RadarDspParams& p,
                                              int r,
                                              int d,
                                              float threshold) {
    unsigned int count = 0;
    const int guard_r = (int)p.guard_range;
    const int guard_d = (int)p.guard_doppler;
    for (int dd = -(int)p.train_doppler - guard_d; dd <= (int)p.train_doppler + guard_d; dd++) {
        for (int dr = -(int)p.train_range - guard_r; dr <= (int)p.train_range + guard_r; dr++) {
            if (abs(dr) <= guard_r && abs(dd) <= guard_d)
                continue;
            const int rr = r + dr;
            if (rr < 0 || rr >= (int)p.num_range_bins)
                continue;
            const int ddd = (d + dd + (int)p.num_doppler_bins) % (int)p.num_doppler_bins;
            if (power_map[(size_t)ddd * p.num_range_bins + rr] <= threshold)
                count++;
        }
    }
    return count;
}

__global__ void CfarKernel(const float* __restrict__ power_map,
                           float* __restrict__ threshold_map,
                           RadarDspParams p) {
    const unsigned int r = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int d = blockIdx.y * blockDim.y + threadIdx.y;
    if (r >= p.num_range_bins || d >= p.num_doppler_bins)
        return;

    const int guard_r = (int)p.guard_range;
    const int guard_d = (int)p.guard_doppler;

    float statistic;
    if (p.cfar_type == 0) {
        float sum = 0.f;
        unsigned int count = 0;
        for (int dd = -(int)p.train_doppler - guard_d; dd <= (int)p.train_doppler + guard_d; dd++) {
            for (int dr = -(int)p.train_range - guard_r; dr <= (int)p.train_range + guard_r; dr++) {
                if (abs(dr) <= guard_r && abs(dd) <= guard_d)
                    continue;
                const int rr = (int)r + dr;
                if (rr < 0 || rr >= (int)p.num_range_bins)
                    continue;
                const int ddd = ((int)d + dd + (int)p.num_doppler_bins) % (int)p.num_doppler_bins;
                sum += power_map[(size_t)ddd * p.num_range_bins + rr];
                count++;
            }
        }
        statistic = count ? sum / count : p.noise_floor;
    } else {
        // The rank is found by bisecting the bit pattern of the training values, which is
        // monotonic for positive floats, so the selection is exact without sorting or scratch.
        unsigned int lo = 0u;
        unsigned int hi = 0x7F7FFFFFu;
        for (int i = 0; i < 26; i++) {
            const unsigned int mid = lo + ((hi - lo) >> 1);
            if (CountBelow(power_map, p, (int)r, (int)d, __uint_as_float(mid)) < p.os_rank)
                lo = mid + 1u;
            else
                hi = mid;
            if (lo >= hi)
                break;
        }
        statistic = __uint_as_float(lo);
        if (!(statistic > 0.f))
            statistic = p.noise_floor;
    }

    threshold_map[(size_t)d * p.num_range_bins + r] = p.threshold_factor * statistic;
}

/// Sub-bin peak position from three samples of a mainlobe, fitted in decibels.
__device__ __inline__ float ParabolicOffset(float left, float centre, float right) {
    const float l = __log10f(fmaxf(left, 1e-30f));
    const float c = __log10f(fmaxf(centre, 1e-30f));
    const float rr = __log10f(fmaxf(right, 1e-30f));
    const float denominator = l - 2.f * c + rr;
    if (fabsf(denominator) < 1e-12f)
        return 0.f;
    return clamp(0.5f * (l - rr) / denominator, -0.5f, 0.5f);
}

__global__ void ExtractKernel(const float* __restrict__ power_map,
                              const float* __restrict__ threshold_map,
                              const float* __restrict__ angle_power,
                              const unsigned long long* __restrict__ provenance,
                              RadarDetection* __restrict__ detections,
                              unsigned int* __restrict__ counter,
                              RadarDspParams p) {
    const unsigned int r = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int d = blockIdx.y * blockDim.y + threadIdx.y;
    if (r == 0 || d >= p.num_doppler_bins || r + 1 >= p.num_range_bins)
        return;

    const size_t cell = (size_t)d * p.num_range_bins + r;
    const float power = power_map[cell];
    if (power <= threshold_map[cell])
        return;

    // One detection per peak: a target spreads over its mainlobe, and reporting every cell above
    // the threshold would report the same target many times over.
    const int guard_r = (int)p.guard_range;
    const int guard_d = (int)p.guard_doppler;
    for (int dd = -guard_d; dd <= guard_d; dd++) {
        const int ddd = ((int)d + dd + (int)p.num_doppler_bins) % (int)p.num_doppler_bins;
        for (int dr = -guard_r; dr <= guard_r; dr++) {
            const int rr = (int)r + dr;
            if (rr < 0 || rr >= (int)p.num_range_bins || (dr == 0 && dd == 0))
                continue;
            const float neighbour = power_map[(size_t)ddd * p.num_range_bins + rr];
            if (neighbour > power || (neighbour == power && (ddd < (int)d || (ddd == (int)d && rr < (int)r))))
                return;
        }
    }

    const float range_offset = ParabolicOffset(power_map[cell - 1], power, power_map[cell + 1]);
    const int d_low = ((int)d - 1 + (int)p.num_doppler_bins) % (int)p.num_doppler_bins;
    const int d_high = ((int)d + 1) % (int)p.num_doppler_bins;
    const float doppler_offset = ParabolicOffset(power_map[(size_t)d_low * p.num_range_bins + r], power,
                                                 power_map[(size_t)d_high * p.num_range_bins + r]);

    const float range = ((float)r + range_offset) * p.range_bin_size;
    if (range < p.min_range)
        return;
    const float range_rate = ((float)d + doppler_offset - 0.5f * (float)p.num_doppler_bins) * p.doppler_bin_size;

    // Bearing of the strongest beam at this cell.
    const size_t plane = (size_t)p.num_doppler_bins * p.num_range_bins;
    unsigned int best_beam = 0;
    float best = -1.f;
    for (unsigned int beam = 0; beam < p.num_azimuth_bins; beam++) {
        const float v = angle_power[(size_t)beam * plane + cell];
        if (v > best) {
            best = v;
            best_beam = beam;
        }
    }
    float beam_offset = 0.f;
    if (best_beam > 0 && best_beam + 1 < p.num_azimuth_bins) {
        beam_offset = ParabolicOffset(angle_power[(size_t)(best_beam - 1) * plane + cell], best,
                                      angle_power[(size_t)(best_beam + 1) * plane + cell]);
    }
    const float azimuth = p.azimuth_min + ((float)best_beam + beam_offset) * p.azimuth_step;

    if (p.suppress_stationary) {
        // Ground and roadside furniture close the range at the ego speed projected on the beam.
        const float clutter_rate = -p.ego_speed * cosf(azimuth);
        if (fabsf(range_rate - clutter_rate) < p.stationary_tolerance)
            return;
    }

    const unsigned int slot = atomicAdd(counter, 1u);
    if (slot >= p.max_detections)
        return;

    const float noise = threshold_map[cell] / fmaxf(p.threshold_factor, 1e-12f);
    const float tx = AntennaGain(p.tx_gain, p.tx_az_exponent, p.tx_el_exponent, azimuth, 0.f);
    const float rx = AntennaGain(p.rx_gain, p.rx_az_exponent, p.rx_el_exponent, azimuth, 0.f);

    // Invert the radar equation on the peak cell. The beamformer's normalization leaves the
    // signal with the channel count as coherent gain, and the windows contribute their peaks.
    const float peak = p.window_peak_product * p.window_peak_product * (float)p.num_channels;
    const float received = power / fmaxf(peak, 1e-30f);
    const float r2 = range * range;
    const float gain = fmaxf(tx * rx, 1e-12f);
    const float rcs = received * r2 * r2 * p.rcs_scale / gain;

    const unsigned long long key = provenance[cell];

    RadarDetection det;
    det.range = range;
    det.range_rate = range_rate;
    det.azimuth = azimuth;
    det.elevation = 0.f;
    det.snr_db = 10.f * __log10f(fmaxf(power / fmaxf(noise, 1e-30f), 1e-12f));
    det.rcs_dbsm = 10.f * __log10f(fmaxf(rcs, 1e-12f));
    det.amplitude = sqrtf(power);
    det.object_id = RadarProvenanceObjectId(key);
    det.flags = 0;
    if (RadarProvenanceBounces(key) > 1u)
        det.flags |= RADAR_DET_MULTIPATH;
    if (fabsf(range_rate) > 0.9f * p.max_unambiguous_velocity)
        det.flags |= RADAR_DET_AMBIGUOUS_VELOCITY;
    detections[slot] = det;
}

__global__ void RangeAzimuthKernel(const float* __restrict__ angle_power,
                                   float* __restrict__ range_azimuth,
                                   RadarDspParams p) {
    const unsigned int r = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int beam = blockIdx.y * blockDim.y + threadIdx.y;
    if (r >= p.num_range_bins || beam >= p.num_azimuth_bins)
        return;
    const float* plane = angle_power + (size_t)beam * p.num_doppler_bins * p.num_range_bins;
    float best = 0.f;
    for (unsigned int d = 0; d < p.num_doppler_bins; d++)
        best = fmaxf(best, plane[(size_t)d * p.num_range_bins + r]);
    range_azimuth[(size_t)beam * p.num_range_bins + r] = best;
}

}  // namespace

void radar_range_azimuth_map(const float* angle_power,
                             float* range_azimuth,
                             const RadarDspParams& params,
                             CUstream stream) {
    const dim3 threads(64, 4);
    const dim3 blocks((params.num_range_bins + threads.x - 1) / threads.x,
                      (params.num_azimuth_bins + threads.y - 1) / threads.y);
    RangeAzimuthKernel<<<blocks, threads, 0, stream>>>(angle_power, range_azimuth, params);
}

void radar_build_steering(const float3* channel_positions,
                          unsigned int num_channels,
                          float wavelength,
                          float azimuth_min,
                          float azimuth_step,
                          unsigned int num_azimuth_bins,
                          float2* steering,
                          CUstream stream) {
    const unsigned int threads = 128;
    const unsigned int blocks = (num_azimuth_bins + threads - 1) / threads;
    SteeringKernel<<<blocks, threads, 0, stream>>>(channel_positions, num_channels, wavelength, azimuth_min,
                                                   azimuth_step, num_azimuth_bins, steering);
}

void radar_beamform(const RadarComplex* cube,
                    const float2* steering,
                    float* angle_power,
                    float* power_map,
                    const RadarDspParams& params,
                    CUstream stream) {
    const dim3 threads(64, 4);
    const dim3 blocks((params.num_range_bins + threads.x - 1) / threads.x,
                      (params.num_doppler_bins + threads.y - 1) / threads.y);
    BeamformKernel<<<blocks, threads, 0, stream>>>(cube, steering, angle_power, power_map, params);
}

void radar_cfar(const float* power_map, float* threshold_map, const RadarDspParams& params, CUstream stream) {
    const dim3 threads(32, 8);
    const dim3 blocks((params.num_range_bins + threads.x - 1) / threads.x,
                      (params.num_doppler_bins + threads.y - 1) / threads.y);
    CfarKernel<<<blocks, threads, 0, stream>>>(power_map, threshold_map, params);
}

void radar_extract_detections(const float* power_map,
                              const float* threshold_map,
                              const float* angle_power,
                              const unsigned long long* provenance,
                              RadarDetection* detections,
                              unsigned int* counter,
                              const RadarDspParams& params,
                              CUstream stream) {
    cudaMemsetAsync(counter, 0, sizeof(unsigned int), stream);
    const dim3 threads(32, 8);
    const dim3 blocks((params.num_range_bins + threads.x - 1) / threads.x,
                      (params.num_doppler_bins + threads.y - 1) / threads.y);
    ExtractKernel<<<blocks, threads, 0, stream>>>(power_map, threshold_map, angle_power, provenance, detections,
                                                  counter, params);
}

}  // namespace sensor
}  // namespace chrono
