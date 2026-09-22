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

#ifndef CHFILTERPHYSRADARDETECT_H
#define CHFILTERPHYSRADARDETECT_H

#include <memory>

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/cuda/radar_dsp.cuh"
#include "chrono_sensor/cuda/radar_impair.cuh"
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChPhysRadarSensor.h"

#include <cuda.h>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// Beamforms the cube, raises the floor by the impairments that are noise-like, and reports the
/// cells that cross a constant false alarm rate threshold.
///
/// The threshold map is kept in the frame alongside the power map, because the distance between
/// the two is what decides whether a target is seen.
class CH_SENSOR_API ChFilterPhysRadarDetect : public ChFilter {
  public:
    ChFilterPhysRadarDetect(std::string name = "ChFilterPhysRadarDetect");
    ~ChFilterPhysRadarDetect();

    virtual void Apply() override;
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) override;

  private:
    std::shared_ptr<ChPhysRadarSensor> m_radar;
    std::shared_ptr<SensorDevicePhysRadarFrame> m_buffer;

    CUstream m_cuda_stream = nullptr;

    RadarDspParams m_dsp;
    RadarPhaseNoiseParams m_phase_noise;
    bool m_apply_phase_noise = false;

    std::shared_ptr<float3[]> m_channel_position;
    std::shared_ptr<float2[]> m_steering;
    std::shared_ptr<float[]> m_skirt_scratch;
    std::shared_ptr<unsigned int[]> m_detection_counter;

    unsigned int m_map_cells = 0;
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
