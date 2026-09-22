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

#ifndef CHFILTERPHYSRADARACCESS_H
#define CHFILTERPHYSRADARACCESS_H

#include <memory>
#include <mutex>
#include <queue>

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChPhysRadarSensor.h"

#include <cuda.h>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// How much of a radar frame is copied back to the host.
///
/// The beamformed power cube is by far the largest array in a frame, so what a caller wants to
/// read is worth saying rather than assuming.
enum class ChRadarFrameContents {
    DETECTIONS,  ///< detections and tracked objects only
    MAPS,        ///< also the detection power map, the threshold map and the cell provenance
    FULL         ///< also the complex cube and the beamformed power
};

/// Hands radar frames to the user, subject to the sensor's lag.
class CH_SENSOR_API ChFilterPhysRadarAccess : public ChFilter {
  public:
    ChFilterPhysRadarAccess(ChRadarFrameContents contents = ChRadarFrameContents::MAPS,
                            std::string name = "ChFilterPhysRadarAccess");
    ~ChFilterPhysRadarAccess();

    virtual void Apply() override;
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) override;

    /// Most recent frame whose lag has elapsed. Ownership of its arrays passes to the caller.
    UserPhysRadarFramePtr GetBuffer();

  private:
    std::shared_ptr<SensorHostPhysRadarFrame> MakeHostFrame() const;

    ChRadarFrameContents m_contents;
    std::weak_ptr<ChSensor> m_sensor;
    std::shared_ptr<SensorDevicePhysRadarFrame> m_buffer_in;
    UserPhysRadarFramePtr m_user_frame;

    std::queue<std::shared_ptr<SensorHostPhysRadarFrame>> m_lag_frames;
    unsigned int m_max_lag_frames = 1;
    std::mutex m_mutex;

    CUstream m_cuda_stream = nullptr;
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
