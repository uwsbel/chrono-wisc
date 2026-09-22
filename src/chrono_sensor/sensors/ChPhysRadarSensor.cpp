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

#include "chrono_sensor/sensors/ChPhysRadarSensor.h"

#include "chrono_sensor/filters/ChFilterPhysRadarDetect.h"
#include "chrono_sensor/filters/ChFilterPhysRadarSignalForm.h"
#include "chrono_sensor/filters/ChFilterPhysRadarTrack.h"

namespace chrono {
namespace sensor {

ChPhysRadarSensor::ChPhysRadarSensor(std::shared_ptr<ChBody> parent,
                                     float updateRate,
                                     ChFrame<double> offsetPose,
                                     const ChRadarModelConfig& config,
                                     bool build_default_chain)
    : ChOptixSensor(parent, updateRate, offsetPose, config.ray_tracing.rays_azimuth, config.ray_tracing.rays_elevation),
      m_config(config) {
    m_config.Validate();
    m_pipeline_type = PipelineType::PHYS_RADAR;

    // One kinematic snapshot per cycle. Motion inside the coherent processing interval is carried
    // analytically as a rate of change of path length, which is what puts targets in Doppler bins.
    SetCollectionWindow(0.f);
    SetLag(1.f / updateRate);

    if (build_default_chain) {
        PushFilter(chrono_types::make_shared<ChFilterPhysRadarSignalForm>());
        PushFilter(chrono_types::make_shared<ChFilterPhysRadarDetect>());
        PushFilter(chrono_types::make_shared<ChFilterPhysRadarTrack>());
    }
}

ChPhysRadarSensor::~ChPhysRadarSensor() {}

}  // namespace sensor
}  // namespace chrono
