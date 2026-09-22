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
// =============================================================================

#ifndef CHFILTERTACHOMETERUPDATE_H
#define CHFILTERTACHOMETERUPDATE_H

#include <memory>
#include <random>

#include "chrono_sensor/filters/ChFilter.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;
class ChNoiseModel;
class ChTachometerSensor;
class ChEncoderSensor;

/// @addtogroup sensor_filters
/// @{

/// Class for generating tachometer data
class CH_SENSOR_API ChFilterTachometerUpdate : public ChFilter {
  public:
    /// Class constructor
    /// @param noise_model The noise model applied to the angular rate, in rad/s
    ChFilterTachometerUpdate(std::shared_ptr<ChNoiseModel> noise_model = nullptr);

    /// Apply function. Generates tachometer data.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    /// @param bufferInOut pointer to the process buffer
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<ChTachometerSensor> m_tachSensor;
    std::shared_ptr<SensorHostTachometerBuffer> m_bufferOut;
    std::shared_ptr<ChNoiseModel> m_noise_model;  ///< The noise model for augmenting data
    float m_last_sample_time = 0;                 ///< sample time of the previous update
    bool m_have_previous_sample = false;          ///< whether m_last_sample_time holds a real time
};

/// Class for generating encoder data
class CH_SENSOR_API ChFilterEncoderUpdate : public ChFilter {
  public:
    /// Class constructor
    /// @param noise_model The noise model applied to the angular rate before counting, in rad/s
    ChFilterEncoderUpdate(std::shared_ptr<ChNoiseModel> noise_model = nullptr);

    /// Apply function. Generates encoder data.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    /// @param bufferInOut pointer to the process buffer
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<ChEncoderSensor> m_encoderSensor;
    std::shared_ptr<SensorHostEncoderBuffer> m_bufferOut;
    std::shared_ptr<ChNoiseModel> m_noise_model;  ///< The noise model for augmenting data

    /// Dropped-pulse draws, kept separate from the noise model's stream: both belong to the same
    /// sensor and filter, so sharing an RngUsage would give them identical sequences.
    std::mt19937 m_generator;

    double m_angle = 0;             ///< integrated shaft angle, in radians
    long long m_raw_counts = 0;     ///< counts the disc has passed, before any are dropped
    long long m_counts = 0;         ///< counts actually registered
    float m_last_sample_time = 0;   ///< sample time of the previous update
    double m_last_rate = 0;         ///< rate at the previous sample, for trapezoidal integration
    bool m_started = false;         ///< whether a previous sample exists to integrate from
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
