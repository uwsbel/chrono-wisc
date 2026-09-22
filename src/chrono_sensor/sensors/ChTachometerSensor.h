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

#ifndef CHTACHOMETER_H
#define CHTACHOMETER_H

#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Axis of the sensor frame about which a rotation rate is measured.
///
/// The axis belongs to the sensor frame, which is the parent body reference frame composed with the
/// sensor offset rotation, so a sensor mounted rotated on its parent measures about its own axis.
enum class ChRotationAxis { X, Y, Z };

/// Tachometer class.
/// This class queries the chrono system for the angular velocity of the parent body.
class CH_SENSOR_API ChTachometerSensor : public ChDynamicSensor {
  public:
    /// Tachometer axis of rotation. Alias of ChRotationAxis, kept so existing code compiles.
    using Axis = ChRotationAxis;

    /// Class constructor
    /// @param parent Body to which the sensor is attached
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of sensor relative to parent body
    /// @param axis Axis of rotation to measure (X,Y,Z)
    ChTachometerSensor(std::shared_ptr<ChBody> parent, float updateRate, ChFrame<double> offsetPose, Axis axis);

    /// Class constructor with a noise model
    /// @param parent Body to which the sensor is attached
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of sensor relative to parent body
    /// @param axis Axis of rotation to measure (X,Y,Z)
    /// @param noise_model Noise model applied to the measured rate, in rad/s
    ChTachometerSensor(std::shared_ptr<ChBody> parent,
                       float updateRate,
                       ChFrame<double> offsetPose,
                       Axis axis,
                       std::shared_ptr<ChNoiseModel> noise_model);

    ~ChTachometerSensor() {}

    virtual void PushKeyFrame() override;
    virtual ChKeyFrameStoreBase& KeyFrames() override { return m_keyframes; }

    /// Get the axis of rotation being measured.
    /// @return The sensor-frame axis
    Axis GetAxis() const { return m_axis; }

  private:
    ChKeyFrameStore<ChVector3d> m_keyframes;  ///< angular rate in the sensor frame, per keyframe
    Axis m_axis;                              ///< sensor-frame axis whose rate is reported

    friend class ChFilterTachometerUpdate;
};

/// Incremental rotary encoder class.
///
/// Counts quadrature edges of a shaft rotating about one axis of the sensor frame. Unlike the
/// tachometer, which samples the instantaneous rate, an encoder accumulates angle and reports the
/// mean speed over the collection window derived from whole counts. That quantisation, at
/// +-1 count, is the dominant error of a real encoder at low speed and short windows, and it falls
/// out of the model rather than being injected.
class CH_SENSOR_API ChEncoderSensor : public ChDynamicSensor {
  public:
    /// Class constructor
    /// @param parent Body whose rotation is counted
    /// @param updateRate Rate at which the sensor should update
    /// @param offsetPose Relative position and orientation of the sensor relative to the parent body
    /// @param axis Axis of rotation to measure (X,Y,Z)
    /// @param counts_per_revolution Lines per revolution of the encoder disc
    /// @param quadrature Whether both edges of both channels are counted, which multiplies the
    /// resolution by four
    /// @param noise_model Optional noise model applied to the angular rate before counting, in rad/s
    ChEncoderSensor(std::shared_ptr<ChBody> parent,
                    float updateRate,
                    ChFrame<double> offsetPose,
                    ChRotationAxis axis,
                    unsigned int counts_per_revolution,
                    bool quadrature = true,
                    std::shared_ptr<ChNoiseModel> noise_model = nullptr);

    ~ChEncoderSensor() {}

    virtual void PushKeyFrame() override;
    virtual ChKeyFrameStoreBase& KeyFrames() override { return m_keyframes; }

    /// Get the axis of rotation being measured.
    /// @return The sensor-frame axis
    ChRotationAxis GetAxis() const { return m_axis; }

    /// Get the number of counts the encoder registers per revolution, including the quadrature
    /// multiplier when it is enabled.
    /// @return Counts per revolution
    unsigned int GetCountsPerRevolution() const { return m_counts_per_revolution; }

    /// Get the angle one count corresponds to.
    /// @return The angular resolution in radians
    double GetResolution() const { return CH_2PI / m_counts_per_revolution; }

    /// Model pulses that fail to be registered, as happens with a dirty or damaged disc.
    /// @param probability Probability that any one count is dropped, in [0, 1]
    void SetMissingPulseProbability(double probability);

    /// Get the probability that a count is dropped.
    /// @return The configured probability
    double GetMissingPulseProbability() const { return m_missing_pulse_probability; }

    /// Model eccentricity of the encoder disc, which shifts the apparent angle once per revolution.
    /// @param amplitude Peak angular error in radians
    /// @param phase Angle at which the error peaks, in radians
    void SetEccentricity(double amplitude, double phase = 0);

    /// Get the amplitude of the once-per-revolution eccentricity error.
    /// @return The peak angular error in radians
    double GetEccentricityAmplitude() const { return m_eccentricity_amplitude; }

  private:
    /// One shaft sample: the time it was taken and the rate about the measured axis.
    struct KeyFrame {
        float time;
        double rate;
    };

    ChKeyFrameStore<KeyFrame> m_keyframes;  ///< shaft samples over the collection window
    ChRotationAxis m_axis;                  ///< sensor-frame axis whose rotation is counted
    unsigned int m_counts_per_revolution;   ///< counts per revolution, quadrature included

    double m_missing_pulse_probability = 0;  ///< probability that a count is dropped
    double m_eccentricity_amplitude = 0;     ///< peak once-per-revolution angular error, in radians
    double m_eccentricity_phase = 0;         ///< angle at which the eccentricity error peaks

    friend class ChFilterEncoderUpdate;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
