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
// Container classes for sensors that make up an IMU (accelerometer, gyroscope,
// magnetometer)
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystem.h"

#include "chrono_sensor/sensors/ChIMUSensor.h"

namespace chrono {
namespace sensor {

// -----------------------------------------------------------------------------
// ChIMULowPass
// -----------------------------------------------------------------------------

void ChIMULowPass::Configure(double cutoff_hz, ChIMUBandwidth order) {
    m_order = (cutoff_hz > 0) ? order : ChIMUBandwidth::NONE;
    m_cutoff = cutoff_hz;
    m_primed = false;
}

void ChIMULowPass::Advance(const ChVector3d& input, double dt) {
    if (m_order == ChIMUBandwidth::NONE)
        return;

    if (!m_primed) {
        // Start settled on the first input rather than at zero, so the filter does not spend its
        // first time constant reporting a transient that the physical sensor never had.
        m_primed = true;
        m_stage1 = input;
        m_stage2 = input;
        return;
    }

    // Exponential rather than the usual dt / (RC + dt) form, so the response is independent of the
    // simulation step size and stays stable when dt is large relative to the time constant.
    const double alpha = 1 - std::exp(-CH_2PI * m_cutoff * dt);

    m_stage1 += (input - m_stage1) * alpha;
    if (m_order == ChIMUBandwidth::SECOND)
        m_stage2 += (m_stage1 - m_stage2) * alpha;
    else
        m_stage2 = m_stage1;
}

// -----------------------------------------------------------------------------
// ChAccelerometerSensor
// -----------------------------------------------------------------------------

ChAccelerometerSensor::ChAccelerometerSensor(std::shared_ptr<ChBody> parent,
                                             float updateRate,
                                             ChFrame<double> offsetPose,
                                             std::shared_ptr<ChNoiseModel> noise_model)
    : ChDynamicSensor(parent, updateRate, offsetPose) {
    m_filters.push_front(chrono_types::make_shared<ChFilterAccelerometerUpdate>(noise_model));
}

ChVector3d ChAccelerometerSensor::SampleInstantaneous() const {
    const ChFrameMoving<double>& body = GetMountingFrame();

    // Specific force, not acceleration: an accelerometer measures the non-gravitational force per
    // unit mass supporting it, so subtracting gravity is what makes a level sensor at rest read
    // +9.81 m/s^2 upwards rather than zero.
    const ChVector3d acc_world = body.PointAccelerationLocalToParent(m_offsetPose.GetPos());
    const ChVector3d gravity = m_parent->GetSystem()->GetGravitationalAcceleration();

    // RotateBack is parent to local: the reading is expressed in the sensor frame, which is the
    // body reference frame composed with the offset rotation.
    const ChQuaternion<double> sensor_rot = body.GetRot() * m_offsetPose.GetRot();
    return sensor_rot.RotateBack(acc_world - gravity);
}

void ChAccelerometerSensor::SetBandwidth(double cutoff_hz, ChIMUBandwidth order) {
    m_bandwidth.Configure(cutoff_hz, order);
}

void ChAccelerometerSensor::AdvanceContinuousModel(double dt) {
    if (m_bandwidth.Enabled())
        m_bandwidth.Advance(SampleInstantaneous(), dt);
}

void ChAccelerometerSensor::PushKeyFrame() {
    m_keyframes.Active().push_back(m_bandwidth.Enabled() ? m_bandwidth.Get() : SampleInstantaneous());
}

// -----------------------------------------------------------------------------
// ChGyroscopeSensor
// -----------------------------------------------------------------------------

ChGyroscopeSensor::ChGyroscopeSensor(std::shared_ptr<ChBody> parent,
                                     float updateRate,
                                     ChFrame<double> offsetPose,
                                     std::shared_ptr<ChNoiseModel> noise_model)
    : ChDynamicSensor(parent, updateRate, offsetPose) {
    m_filters.push_front(chrono_types::make_shared<ChFilterGyroscopeUpdate>(noise_model));
}

ChVector3d ChGyroscopeSensor::SampleInstantaneous() const {
    // The angular velocity of a rigid body is the same about every point on it, so only the offset
    // rotation matters: it takes the rate from the body reference frame into the sensor frame.
    return m_offsetPose.GetRot().RotateBack(GetMountingFrame().GetAngVelLocal());
}

void ChGyroscopeSensor::SetBandwidth(double cutoff_hz, ChIMUBandwidth order) {
    m_bandwidth.Configure(cutoff_hz, order);
}

void ChGyroscopeSensor::AdvanceContinuousModel(double dt) {
    if (m_bandwidth.Enabled())
        m_bandwidth.Advance(SampleInstantaneous(), dt);
}

void ChGyroscopeSensor::PushKeyFrame() {
    m_keyframes.Active().push_back(m_bandwidth.Enabled() ? m_bandwidth.Get() : SampleInstantaneous());
}

// -----------------------------------------------------------------------------
// ChMagnetometerSensor
// -----------------------------------------------------------------------------

ChMagnetometerSensor::ChMagnetometerSensor(std::shared_ptr<ChBody> parent,
                                           float updateRate,
                                           ChFrame<double> offsetPose,
                                           std::shared_ptr<ChNoiseModel> noise_model,
                                           ChVector3d gps_reference)
    : ChDynamicSensor(parent, updateRate, offsetPose), m_gps_reference(gps_reference) {
    m_filters.push_front(chrono_types::make_shared<ChFilterMagnetometerUpdate>(noise_model, gps_reference));
}

void ChMagnetometerSensor::PushKeyFrame() {
    // The field depends on where the sensor is and the reading on how it is oriented, so the whole
    // sensor frame is needed rather than a single vector.
    const ChFrame<double>& body = GetMountingFrame();
    m_keyframes.Active().push_back(body.TransformLocalToParent(m_offsetPose));
}

}  // namespace sensor
}  // namespace chrono
