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
// Container class for an IMU sensor
//
// =============================================================================

#ifndef CHIMUSENSOR_H
#define CHIMUSENSOR_H

#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/filters/ChFilterIMUUpdate.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/utils/ChMagneticField.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Order of the analogue low-pass an inertial sensor applies before its sampler.
enum class ChIMUBandwidth {
    NONE,   ///< no bandwidth model; the sample is the mean of the keyframes in the collection window
    FIRST,  ///< single-pole low-pass
    SECOND  ///< critically damped two-pole low-pass
};

/// Low-pass modelling the analogue bandwidth of an inertial sensor.
///
/// Advanced once per simulation step and read at the update instant, so it sees the whole signal
/// rather than only the part inside the collection window. Without it the effective bandwidth is set
/// by whatever update rate the user happens to pick, and content above half that rate aliases down
/// into the band instead of being attenuated.
class CH_SENSOR_API ChIMULowPass {
  public:
    /// Configure the filter.
    /// @param cutoff_hz -3 dB cutoff frequency in Hz
    /// @param order Order of the filter; ChIMUBandwidth::NONE disables it
    void Configure(double cutoff_hz, ChIMUBandwidth order);

    /// Advance the filter state by dt seconds with the given instantaneous input.
    /// @param input The unfiltered quantity, in sensor units
    /// @param dt Elapsed simulation time in seconds
    void Advance(const ChVector3d& input, double dt);

    /// Get the filtered value.
    /// @return The filter output, in sensor units
    const ChVector3d& Get() const { return m_stage2; }

    /// Whether a bandwidth model is configured.
    /// @return True if the filter is doing anything
    bool Enabled() const { return m_order != ChIMUBandwidth::NONE; }

  private:
    ChIMUBandwidth m_order = ChIMUBandwidth::NONE;
    double m_cutoff = 0;                     ///< cutoff frequency in Hz
    ChVector3d m_stage1 = ChVector3d(0, 0, 0);  ///< first pole state
    ChVector3d m_stage2 = ChVector3d(0, 0, 0);  ///< second pole state, and the filter output
    bool m_primed = false;                   ///< whether the state has seen its first input
};

/// Accelerometer class.
///
/// Reports the specific force at the sensor, resolved in the sensor frame:
/// `f = q_sensor^-1 * (a_world - g_world)`, where `q_sensor` is the parent body reference frame
/// rotation composed with the offset pose rotation. This is what an accelerometer measures: at rest
/// it reads the reaction to gravity, not gravity itself, so a level sensor reads +9.81 m/s^2 on its
/// up axis. Lever-arm terms from the offset position are included.
///
/// The data is collected from the physical quantities computed for the parent object accounting for the offset pose.
/// This sensor operates in lock-step with the Chrono simulation.
class CH_SENSOR_API ChAccelerometerSensor : public ChDynamicSensor {
  public:
    /// Class constructor for an accelerometer sensor
    /// @param parent Body to which the sensor is attached.
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    /// @param noise_model Noise model for the sensor to use when augmenting data
    ChAccelerometerSensor(std::shared_ptr<ChBody> parent,
                          float updateRate,
                          ChFrame<double> offsetPose,
                          std::shared_ptr<ChNoiseModel> noise_model);

    ~ChAccelerometerSensor() {}

    virtual void PushKeyFrame() override;
    virtual ChKeyFrameStoreBase& KeyFrames() override { return m_keyframes; }

    /// Model the analogue bandwidth ahead of the sampler.
    ///
    /// Off by default, in which case the sample is the mean of the keyframes collected over the
    /// window, as before. A representative cutoff is 0.4 times the update rate.
    /// @param cutoff_hz -3 dB cutoff frequency in Hz
    /// @param order Order of the filter; ChIMUBandwidth::NONE restores the default behaviour
    void SetBandwidth(double cutoff_hz, ChIMUBandwidth order = ChIMUBandwidth::SECOND);

    /// Get the instantaneous specific force at the sensor, in the sensor frame and in m/s^2.
    /// @return The unfiltered, noise-free reading the sensor would give right now
    ChVector3d SampleInstantaneous() const;

  protected:
    virtual void AdvanceContinuousModel(double dt) override;

  private:
    ChKeyFrameStore<ChVector3d> m_keyframes;  ///< stores keyframes for sensor
    ChIMULowPass m_bandwidth;                 ///< optional analogue bandwidth model

    friend class ChFilterAccelerometerUpdate;
};

/// Gyroscope class.
///
/// Reports the angular velocity of the parent body resolved in the sensor frame, so a gyroscope
/// mounted rotated relative to the body reports about its own axes rather than the body's.
///
/// The data is collected from the physical quantities computed for the parent object accounting for the offset pose.
/// This sensor operates in lock-step with the Chrono simulation.
class CH_SENSOR_API ChGyroscopeSensor : public ChDynamicSensor {
  public:
    /// Class constructor for a gyroscope
    /// @param parent Body to which the sensor is attached.
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    /// @param noise_model Noise model for the sensor to use when augmentating data
    ChGyroscopeSensor(std::shared_ptr<ChBody> parent,
                      float updateRate,
                      ChFrame<double> offsetPose,
                      std::shared_ptr<ChNoiseModel> noise_model);

    ~ChGyroscopeSensor() {}

    virtual void PushKeyFrame() override;
    virtual ChKeyFrameStoreBase& KeyFrames() override { return m_keyframes; }

    /// Model the analogue bandwidth ahead of the sampler. See ChAccelerometerSensor::SetBandwidth.
    /// @param cutoff_hz -3 dB cutoff frequency in Hz
    /// @param order Order of the filter; ChIMUBandwidth::NONE restores the default behaviour
    void SetBandwidth(double cutoff_hz, ChIMUBandwidth order = ChIMUBandwidth::SECOND);

    /// Get the instantaneous angular velocity at the sensor, in the sensor frame and in rad/s.
    /// @return The unfiltered, noise-free reading the sensor would give right now
    ChVector3d SampleInstantaneous() const;

  protected:
    virtual void AdvanceContinuousModel(double dt) override;

  private:
    ChKeyFrameStore<ChVector3d> m_keyframes;  ///< stores keyframes for sensor
    ChIMULowPass m_bandwidth;                 ///< optional analogue bandwidth model

    friend class ChFilterGyroscopeUpdate;
};

/// Magnetometer class.
///
/// Reports the Earth magnetic field at the sensor position, resolved in the sensor frame, in Tesla.
/// By default the field comes from the World Magnetic Model 2025 evaluated at the sensor's latitude,
/// longitude and altitude; a constant measured field can be supplied instead.
///
/// The data is collected from the physical quantities computed for the parent object accounting for the offset pose.
/// This sensor operates in lock-step with the Chrono simulation.
class CH_SENSOR_API ChMagnetometerSensor : public ChDynamicSensor {
  public:
    /// Class constructor for a magnetometer
    /// @param parent Body to which the sensor is attached.
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    /// @param noise_model Noise model for the sensor to use when augmenting data
    /// @param gps_reference GPS coordinates of the simulation origin, as
    /// (LONGITUDE, LATITUDE, ALTITUDE) in degrees and metres. Note the ordering: longitude first.
    ChMagnetometerSensor(std::shared_ptr<ChBody> parent,
                         float updateRate,
                         ChFrame<double> offsetPose,
                         std::shared_ptr<ChNoiseModel> noise_model,
                         ChVector3d gps_reference);

    ~ChMagnetometerSensor() {}

    virtual void PushKeyFrame() override;
    virtual ChKeyFrameStoreBase& KeyFrames() override { return m_keyframes; }

    /// Get the GPS reference location.
    /// @return The simulation origin as (longitude, latitude, altitude) in degrees and metres
    const ChVector3d GetGPSReference() const { return m_gps_reference; }

    /// Select the source of the field.
    /// @param model The field model to evaluate
    void SetFieldModel(ChMagneticFieldModel model) { m_field_model = model; }

    /// Get the source of the field.
    /// @return The field model in use
    ChMagneticFieldModel GetFieldModel() const { return m_field_model; }

    /// Use a constant field instead of a global model, and select ChMagneticFieldModel::LOCAL.
    ///
    /// Appropriate when the field has been measured at the site. The vector is in the simulation
    /// ENU frame, so in the northern hemisphere its Z component is negative.
    /// @param field_tesla The local field in Tesla
    void SetLocalField(const ChVector3d& field_tesla) {
        m_local_field = field_tesla;
        m_field_model = ChMagneticFieldModel::LOCAL;
    }

    /// Get the constant field used by ChMagneticFieldModel::LOCAL.
    /// @return The configured local field in Tesla
    const ChVector3d& GetLocalField() const { return m_local_field; }

    /// Set the time at which the global field model is evaluated. The secular variation of the
    /// field is slow enough that the default, the model epoch, suits almost every simulation.
    /// @param decimal_year Time in decimal years
    void SetEpoch(double decimal_year) { m_epoch = decimal_year; }

    /// Get the time at which the global field model is evaluated.
    /// @return The configured time in decimal years
    double GetEpoch() const { return m_epoch; }

  private:
    ChKeyFrameStore<ChFrame<double>> m_keyframes;  ///< sensor keyframes
    const ChVector3d m_gps_reference;  ///< reference location in GPS coordinates (longitude, latitude, altitude)

    ChMagneticFieldModel m_field_model = ChMagneticFieldModel::WMM2025;  ///< source of the field
    ChVector3d m_local_field = ChVector3d(0, 0, 0);  ///< constant field used by the LOCAL model, in Tesla
    double m_epoch = WMM2025_EPOCH;                  ///< evaluation time of the global model, in decimal years

    friend class ChFilterMagnetometerUpdate;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
