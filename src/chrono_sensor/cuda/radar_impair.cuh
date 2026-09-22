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
// Receiver noise and radio-frequency impairments of the wave-domain radar.
//
// =============================================================================

#ifndef CHRADARIMPAIR_CUH
#define CHRADARIMPAIR_CUH

#include <cuda.h>
#include <curand_kernel.h>

#include "chrono_sensor/sensors/radar/ChRadarTypes.h"

namespace chrono {
namespace sensor {

/// Add band-limited thermal noise to every cube cell.
/// @param noise_power total noise power in one cell; split evenly between the two quadratures
void radar_add_thermal_noise(RadarComplex* cube,
                             unsigned int count,
                             float noise_power,
                             curandState_t* states,
                             unsigned int num_states,
                             CUstream stream);

/// Parameters of the phase noise skirt.
struct RadarPhaseNoiseParams {
    unsigned int num_range_bins;
    unsigned int num_doppler_bins;
    unsigned int skirt_half_width;  ///< range bins either side the skirt is evaluated over

    float range_bin_size;        ///< [m]
    float beat_frequency_per_bin;///< [Hz] frequency step between adjacent range bins
    float level_at_reference;    ///< phase noise density at the reference offset [1/Hz]
    float reference_offset;      ///< offset that level is quoted at [Hz]
    float decade_slope;          ///< roll-off per decade, as a power ratio exponent
};

/// Spread the phase noise skirt of every strong cell across its range neighbours.
///
/// Dechirping correlates the transmitted and received phase noise, so a return contributes a
/// skirt suppressed by 4*sin^2(pi*f*tau) at offset f: a close target splashes almost nothing,
/// while a strong one at mid range raises the floor around itself and can bury a weaker
/// neighbour. Added as power, since the skirt is noise-like.
void radar_add_phase_noise_skirt(const float* power_map,
                                 float* out_power_map,
                                 const RadarPhaseNoiseParams& params,
                                 CUstream stream);

/// Raise the floor by the converter's quantization noise, which a strong return drags up with it.
void radar_add_uniform_floor(float* power_map, unsigned int count, float floor_power, CUstream stream);

}  // namespace sensor
}  // namespace chrono

#endif
