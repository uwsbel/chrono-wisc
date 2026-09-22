// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// Container class for an GPS sensor
//
// =============================================================================

#include "chrono_sensor/sensors/ChGPSSensor.h"

namespace chrono {
namespace sensor {

ChGPSSensor::ChGPSSensor(std::shared_ptr<ChBody> parent,
                         float updateRate,
                         ChFrame<double> offsetPose,
                         ChVector3d gps_reference,
                         std::shared_ptr<ChNoiseModel> noise_model)
    : ChDynamicSensor(parent, updateRate, offsetPose), m_gps_reference(gps_reference) {
    m_filters.push_front(chrono_types::make_shared<ChFilterGPSUpdate>(gps_reference, noise_model));
}

ChGPSSensor::~ChGPSSensor() {}

void ChGPSSensor::PushKeyFrame() {
    const ChFrameMoving<double>& body = GetMountingFrame();
    KeyFrame keyframe;
    keyframe.time = (float)m_parent->GetSystem()->GetChTime();
    keyframe.position = body.TransformPointLocalToParent(m_offsetPose.GetPos());
    // The antenna is offset from the body origin, so its velocity picks up a lever-arm term from
    // the body rotation and is not simply the body velocity.
    keyframe.velocity = body.PointSpeedLocalToParent(m_offsetPose.GetPos());
    m_keyframes.Active().push_back(keyframe);
}

void ChGPSSensor::SetNominalFix(ChGPSFixType fix, double hdop, unsigned int num_satellites) {
    m_fix = fix;
    m_hdop = hdop;
    m_num_satellites = num_satellites;
}

}  // namespace sensor
}  // namespace chrono
