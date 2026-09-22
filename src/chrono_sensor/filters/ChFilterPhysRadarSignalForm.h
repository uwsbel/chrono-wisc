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

#ifndef CHFILTERPHYSRADARSIGNALFORM_H
#define CHFILTERPHYSRADARSIGNALFORM_H

#include <memory>

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/cuda/radar_deposit.cuh"
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChPhysRadarSensor.h"

#include <cuda.h>
#include <curand_kernel.h>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// Turns the ray tracer's coherent paths into a range-Doppler cube with a receiver's noise on it.
///
/// This filter owns the radar frame: it allocates every array the later stages fill, so a frame
/// has one owner and one lifetime.
class CH_SENSOR_API ChFilterPhysRadarSignalForm : public ChFilter {
  public:
    ChFilterPhysRadarSignalForm(std::string name = "ChFilterPhysRadarSignalForm");
    ~ChFilterPhysRadarSignalForm();

    virtual void Apply() override;
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) override;

  private:
    std::shared_ptr<ChPhysRadarSensor> m_radar;
    std::shared_ptr<SensorDevicePhysRadarPathBuffer> m_buffer_in;
    std::shared_ptr<SensorDevicePhysRadarFrame> m_buffer_out;

    CUstream m_cuda_stream = nullptr;

    RadarDepositParams m_deposit;
    std::shared_ptr<float3[]> m_channel_position;
    std::shared_ptr<float[]> m_channel_gain;
    std::shared_ptr<unsigned int[]> m_channel_slot;

    /// Single synthetic path standing in for the transmitter leaking straight into the receiver.
    std::shared_ptr<RadarPath[]> m_leakage_path;
    bool m_has_leakage = false;

    std::shared_ptr<curandState_t[]> m_noise_rng;
    unsigned int m_num_noise_states = 0;
    std::shared_ptr<float[]> m_total_power;

    unsigned int m_cube_cells = 0;
    unsigned int m_map_cells = 0;
    float m_noise_power_per_cell = 0.f;
    float m_quantization_scale = 0.f;
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
