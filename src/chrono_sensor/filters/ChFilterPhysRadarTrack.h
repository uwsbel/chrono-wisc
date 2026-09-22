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

#ifndef CHFILTERPHYSRADARTRACK_H
#define CHFILTERPHYSRADARTRACK_H

#include <deque>
#include <memory>
#include <vector>

#include "chrono/core/ChMatrix.h"

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChPhysRadarSensor.h"

#include <cuda.h>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// Groups detections into targets and follows them from cycle to cycle.
///
/// A track is reported only once it has been hit on several of the last few cycles, which is why
/// a target entering the field of view is reported late rather than instantly: the delay is the
/// confirmation logic doing its job, not a latency model.
class CH_SENSOR_API ChFilterPhysRadarTrack : public ChFilter {
  public:
    ChFilterPhysRadarTrack(std::string name = "ChFilterPhysRadarTrack");
    ~ChFilterPhysRadarTrack();

    virtual void Apply() override;
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) override;

  private:
    /// One target, between the detections that feed it and the object list that reports it.
    struct Track {
        ChVectorN<double, 4> state;     ///< position and velocity in the sensor frame
        ChMatrixNM<double, 4, 4> covariance;
        double range_rate = 0.0;
        double rcs_dbsm = 0.0;
        unsigned int id = 0;
        unsigned int age = 0;
        unsigned int misses = 0;
        unsigned int object_id = 0;
        std::deque<bool> history;  ///< hit or miss on each of the last cycles
    };

    std::shared_ptr<ChPhysRadarSensor> m_radar;
    std::shared_ptr<SensorDevicePhysRadarFrame> m_buffer;

    CUstream m_cuda_stream = nullptr;

    std::vector<RadarDetection> m_detections;
    std::vector<Track> m_tracks;
    unsigned int m_next_id = 1;
    float m_last_time = 0.f;
    bool m_started = false;
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
