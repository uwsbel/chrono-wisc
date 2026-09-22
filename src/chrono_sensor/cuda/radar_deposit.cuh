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
// Authors: Project Chrono
// =============================================================================
//
// Deposition of coherent propagation paths into the range-Doppler cube.
//
// =============================================================================

#ifndef CHRADARDEPOSIT_CUH
#define CHRADARDEPOSIT_CUH

#include <cuda.h>
#include <vector_types.h>

#include "chrono_sensor/sensors/radar/ChRadarTypes.h"

namespace chrono {
namespace sensor {

/// Largest half-width of the deposited window response. Four taps either side already reach the
/// point where a raised-cosine window's response is far below the thermal floor.
static const unsigned int CH_RADAR_MAX_DEPOSIT_TAPS = 4;

/// Geometry and waveform constants the deposition needs, packed for the device.
struct RadarDepositParams {
    unsigned int num_range_bins;
    unsigned int num_doppler_bins;
    unsigned int num_channels;
    unsigned int range_transform_length;  ///< samples per chirp; sets the range window response
    unsigned int taps;                    ///< half-width of the deposited response, in bins;
                                          ///< clamped to CH_RADAR_MAX_DEPOSIT_TAPS

    float wavelength;             ///< [m]
    float range_bin_size;         ///< [m]
    float doppler_bin_size;       ///< [m/s]
    float doppler_range_coupling; ///< range bins of shift per m/s of radial velocity
    float tdm_slot_phase;         ///< slow-time phase per transmitter slot, per m/s
    int range_window;             ///< ChRadarWindowType
    int doppler_window;           ///< ChRadarWindowType

    const float3* channel_position;     ///< virtual element position of each channel [m]
    const float* channel_gain;          ///< voltage gain of each channel, nominally one
    const unsigned int* channel_slot;   ///< transmitter slot index of each channel
};

/// Zero a complex cube.
void radar_clear_cube(RadarComplex* cube, unsigned int count, CUstream stream);

/// Accumulate every path into the cube, coherently and per channel, and record which path
/// dominates each range-Doppler cell.
void radar_deposit_paths(const RadarPath* paths,
                         unsigned int num_paths,
                         RadarComplex* cube,
                         unsigned long long* provenance,
                         const RadarDepositParams& params,
                         CUstream stream);

/// Total power carried by the path list, which is what the converter's full scale tracks.
void radar_total_path_power(const RadarPath* paths, unsigned int num_paths, float* total, CUstream stream);

}  // namespace sensor
}  // namespace chrono

#endif
