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
// Detection chain of the wave-domain radar: beamforming, thresholding, peak extraction.
//
// =============================================================================

#ifndef CHRADARDSP_CUH
#define CHRADARDSP_CUH

#include <cuda.h>
#include <vector_types.h>

#include "chrono_sensor/sensors/radar/ChRadarTypes.h"

namespace chrono {
namespace sensor {

/// Constants the detection chain needs, packed for the device.
struct RadarDspParams {
    unsigned int num_range_bins;
    unsigned int num_doppler_bins;
    unsigned int num_channels;
    unsigned int num_azimuth_bins;

    float range_bin_size;    ///< [m]
    float doppler_bin_size;  ///< [m/s]
    float azimuth_min;       ///< azimuth of beam 0 [rad]
    float azimuth_step;      ///< [rad]
    float min_range;         ///< bins nearer than this are not reported [m]

    int cfar_type;  ///< 0 cell averaging, 1 ordered statistic
    unsigned int guard_range;
    unsigned int guard_doppler;
    unsigned int train_range;
    unsigned int train_doppler;
    float threshold_factor;  ///< multiplier on the training statistic for the configured false alarm rate
    unsigned int os_rank;    ///< one-based rank of the ordered statistic

    unsigned int max_detections;
    float noise_floor;  ///< thermal floor, used where the training window holds nothing

    float rcs_scale;            ///< (4*pi)^3 / (transmit power * wavelength^2)
    float tx_gain;
    float tx_az_exponent;
    float tx_el_exponent;
    float rx_gain;
    float rx_az_exponent;
    float rx_el_exponent;
    float window_peak_product;  ///< peak of the range window response times that of the Doppler one

    int suppress_stationary;
    float ego_speed;             ///< forward speed of the sensor [m/s]
    float stationary_tolerance;  ///< [m/s]
    float max_unambiguous_velocity;
};

/// Form beams across the virtual array and collapse them into a detection power map.
/// Beamformed power is normalized so the thermal floor of one cube cell is also the floor of the
/// map, which keeps the false alarm rate independent of the number of channels.
void radar_beamform(const RadarComplex* cube,
                    const float2* steering,
                    float* angle_power,
                    float* power_map,
                    const RadarDspParams& params,
                    CUstream stream);

/// Fill the per-cell detection threshold from the surrounding training cells.
void radar_cfar(const float* power_map, float* threshold_map, const RadarDspParams& params, CUstream stream);

/// Emit one detection per local maximum that crosses its threshold.
void radar_extract_detections(const float* power_map,
                              const float* threshold_map,
                              const float* angle_power,
                              const unsigned long long* provenance,
                              RadarDetection* detections,
                              unsigned int* counter,
                              const RadarDspParams& params,
                              CUstream stream);

/// Collapse the Doppler axis of the beamformed power, giving the range-azimuth image a radar
/// display shows. Output is [azimuth][range].
void radar_range_azimuth_map(const float* angle_power,
                             float* range_azimuth,
                             const RadarDspParams& params,
                             CUstream stream);

/// Steering weights for every beam and channel, laid out beam-major.
void radar_build_steering(const float3* channel_positions,
                          unsigned int num_channels,
                          float wavelength,
                          float azimuth_min,
                          float azimuth_step,
                          unsigned int num_azimuth_bins,
                          float2* steering,
                          CUstream stream);

}  // namespace sensor
}  // namespace chrono

#endif
