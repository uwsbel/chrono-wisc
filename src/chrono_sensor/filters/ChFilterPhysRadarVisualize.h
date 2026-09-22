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

#ifndef CHFILTERPHYSRADARVISUALIZE_H
#define CHFILTERPHYSRADARVISUALIZE_H

#include <memory>
#include <string>
#include <vector>

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/cuda/radar_dsp.cuh"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChPhysRadarSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// Opens a window onto the inside of the radar's signal chain.
///
/// Four panels, each one stage of the pipeline:
///  - range against Doppler, the cube the paths were deposited into;
///  - the same cells measured against their detection threshold, which is what decides a miss;
///  - range against bearing, the image the beamformer forms;
///  - a plan view of the detections and tracks, with mirror ghosts marked.
///
/// Frames can also be written to disk, which is how the view is used on a machine with no display.
class CH_SENSOR_API ChFilterPhysRadarVisualize : public ChFilterVisualize {
  public:
    /// @param w window width in pixels
    /// @param h window height in pixels
    /// @param name filter name, also the window title
    ChFilterPhysRadarVisualize(int w = 1280, int h = 800, std::string name = "Radar signal chain");
    ~ChFilterPhysRadarVisualize();

    virtual void Apply() override;
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) override;

    /// Also write every rendered frame to <directory>/<name>_<index>.png.
    void SetSaveDirectory(const std::string& directory) { m_save_directory = directory; }

    /// Range of the plan view [m]. Defaults to the radar's configured maximum range.
    void SetPlanViewRange(float range) { m_plan_range = range; }

    /// Span of the decibel colour scales, measured up from the thermal noise floor.
    void SetDynamicRange(float decibels) { m_dynamic_range = decibels; }

  private:
    void Compose();

    std::shared_ptr<ChPhysRadarSensor> m_radar;
    std::shared_ptr<SensorDevicePhysRadarFrame> m_buffer;

    CUstream m_cuda_stream = nullptr;
    RadarDspParams m_dsp;

    std::shared_ptr<float[]> m_range_azimuth_device;

    std::vector<float> m_power_map;
    std::vector<float> m_threshold_map;
    std::vector<float> m_range_azimuth;
    std::vector<RadarDetection> m_detections;
    std::vector<PixelRGBA8> m_image;

    std::string m_save_directory;
    unsigned int m_frame_index = 0;
    float m_plan_range = 0.f;
    float m_dynamic_range = 80.f;
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
