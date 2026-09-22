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
// Authors: Han Wang
// =============================================================================
//
// Tachometer model is parameterized by :
// 1. parent: the body the sensor is taking measurements
// 2. updateRate: frequency of data acquisition
// 3. axis: Axis of rotation to measure (X,Y,Z)
//
// =============================================================================

#include <algorithm>
#include <stdexcept>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_sensor/sensors/ChTachometerSensor.h"
#include "chrono_sensor/filters/ChFilterTachometerUpdate.h"

namespace chrono {
namespace sensor {

namespace {

/// Component of a sensor-frame angular rate about the selected axis.
double RateAboutAxis(const ChVector3d& rate, ChRotationAxis axis) {
    switch (axis) {
        case ChRotationAxis::X:
            return rate.x();
        case ChRotationAxis::Y:
            return rate.y();
        case ChRotationAxis::Z:
            return rate.z();
    }
    throw std::runtime_error("Rotation axis must be X, Y or Z");
}

}  // namespace

// -----------------------------------------------------------------------------
// ChTachometerSensor
// -----------------------------------------------------------------------------

ChTachometerSensor::ChTachometerSensor(std::shared_ptr<ChBody> parent,
                                       float updateRate,
                                       ChFrame<double> offsetPose,
                                       Axis axis)
    : ChTachometerSensor(parent, updateRate, offsetPose, axis, nullptr) {}

ChTachometerSensor::ChTachometerSensor(std::shared_ptr<ChBody> parent,
                                       float updateRate,
                                       ChFrame<double> offsetPose,
                                       Axis axis,
                                       std::shared_ptr<ChNoiseModel> noise_model)
    : ChDynamicSensor(parent, updateRate, offsetPose), m_axis(axis) {
    m_filters.push_front(chrono_types::make_shared<ChFilterTachometerUpdate>(noise_model));
}

void ChTachometerSensor::PushKeyFrame() {
    // The offset rotation takes the rate into the sensor frame, so a tachometer mounted rotated on
    // its parent measures about its own axis rather than the body's.
    m_keyframes.Active().push_back(m_offsetPose.GetRot().RotateBack(GetMountingFrame().GetAngVelLocal()));
}

// -----------------------------------------------------------------------------
// ChEncoderSensor
// -----------------------------------------------------------------------------

ChEncoderSensor::ChEncoderSensor(std::shared_ptr<ChBody> parent,
                                 float updateRate,
                                 ChFrame<double> offsetPose,
                                 ChRotationAxis axis,
                                 unsigned int counts_per_revolution,
                                 bool quadrature,
                                 std::shared_ptr<ChNoiseModel> noise_model)
    : ChDynamicSensor(parent, updateRate, offsetPose),
      m_axis(axis),
      m_counts_per_revolution(std::max(1u, counts_per_revolution) * (quadrature ? 4u : 1u)) {
    m_filters.push_front(chrono_types::make_shared<ChFilterEncoderUpdate>(noise_model));
}

void ChEncoderSensor::PushKeyFrame() {
    KeyFrame keyframe;
    keyframe.time = (float)m_parent->GetSystem()->GetChTime();
    keyframe.rate = RateAboutAxis(m_offsetPose.GetRot().RotateBack(GetMountingFrame().GetAngVelLocal()), m_axis);
    m_keyframes.Active().push_back(keyframe);
}

void ChEncoderSensor::SetMissingPulseProbability(double probability) {
    m_missing_pulse_probability = ChClamp(probability, 0.0, 1.0);
}

void ChEncoderSensor::SetEccentricity(double amplitude, double phase) {
    m_eccentricity_amplitude = amplitude;
    m_eccentricity_phase = phase;
}

}  // namespace sensor
}  // namespace chrono
