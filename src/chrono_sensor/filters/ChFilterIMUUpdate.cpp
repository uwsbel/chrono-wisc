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
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

#include "chrono_sensor/filters/ChFilterIMUUpdate.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/utils/ChGPSUtils.h"
#include "chrono_sensor/utils/ChMagneticField.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"

namespace chrono {
namespace sensor {

namespace {

/// Mean of the keyframes collected over one window, or zero if the window was empty.
///
/// A boxcar average over the window when no bandwidth model is configured. When one is, every
/// keyframe already carries the filter output, so this only averages out the residual.
ChVector3d MeanOfKeyFrames(const std::vector<ChVector3d>& keyframes) {
    if (keyframes.empty())
        return ChVector3d(0, 0, 0);

    ChVector3d sum(0, 0, 0);
    for (const auto& keyframe : keyframes)
        sum += keyframe;
    return sum / (double)keyframes.size();
}

}  // namespace

// -----------------------------------------------------------------------------
// ChFilterAccelerometerUpdate
// -----------------------------------------------------------------------------

ChFilterAccelerometerUpdate::ChFilterAccelerometerUpdate(std::shared_ptr<ChNoiseModel> noise_model)
    : ChFilter("Accelerometer Updater"), m_noise_model(noise_model) {}

CH_SENSOR_API void ChFilterAccelerometerUpdate::Apply() {
    ChVector3d acc = MeanOfKeyFrames(m_accSensor->m_keyframes.Active());

    const float sample_time = m_accSensor->GetSampleTime();
    if (m_noise_model) {
        // The first sample has no preceding one to measure an interval against, so it uses the
        // nominal update period. Measuring from a default-constructed zero would hand the model an
        // interval of one simulation step and, for a model that scales noise by the sampling
        // interval, overstate the first sample.
        if (m_have_previous_sample)
            m_noise_model->AddNoise(acc, m_last_sample_time, sample_time);
        else
            m_noise_model->AddNoise(acc);
    }
    m_last_sample_time = sample_time;
    m_have_previous_sample = true;

    m_bufferOut->Buffer[0].X = acc.x();
    m_bufferOut->Buffer[0].Y = acc.y();
    m_bufferOut->Buffer[0].Z = acc.z();

    m_bufferOut->LaunchedCount = m_accSensor->GetNumLaunches();
    // The time the sample describes, which with a nonzero lag precedes the time it became visible.
    m_bufferOut->TimeStamp = sample_time;
}

CH_SENSOR_API void ChFilterAccelerometerUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                           std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("Accelerometer update filter must be applied first in filter graph");
    }
    m_accSensor = std::dynamic_pointer_cast<ChAccelerometerSensor>(pSensor);
    if (!m_accSensor) {
        throw std::runtime_error("Accelerometer Update filter can only be used on an accelerometer\n");
    }

    if (m_noise_model)
        m_noise_model->Initialize(pSensor, RngUsage::AccelerometerNoise, GetRngStreamIndex());

    m_bufferOut = chrono_types::make_shared<SensorHostAccelBuffer>();
    m_bufferOut->Buffer = std::make_unique<AccelData[]>(1);
    m_bufferOut->Width = m_bufferOut->Height = 1;
    m_bufferOut->LaunchedCount = m_accSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = 0.f;

    bufferInOut = m_bufferOut;
}

// -----------------------------------------------------------------------------
// ChFilterGyroscopeUpdate
// -----------------------------------------------------------------------------

ChFilterGyroscopeUpdate::ChFilterGyroscopeUpdate(std::shared_ptr<ChNoiseModel> noise_model)
    : ChFilter("Gyroscope Updater"), m_noise_model(noise_model) {}

CH_SENSOR_API void ChFilterGyroscopeUpdate::Apply() {
    ChVector3d ang_vel = MeanOfKeyFrames(m_gyroSensor->m_keyframes.Active());

    const float sample_time = m_gyroSensor->GetSampleTime();
    if (m_noise_model) {
        // The first sample has no preceding one to measure an interval against, so it uses the
        // nominal update period. Measuring from a default-constructed zero would hand the model an
        // interval of one simulation step and, for a model that scales noise by the sampling
        // interval, overstate the first sample.
        if (m_have_previous_sample)
            m_noise_model->AddNoise(ang_vel, m_last_sample_time, sample_time);
        else
            m_noise_model->AddNoise(ang_vel);
    }
    m_last_sample_time = sample_time;
    m_have_previous_sample = true;

    m_bufferOut->Buffer[0].Roll = ang_vel.x();
    m_bufferOut->Buffer[0].Pitch = ang_vel.y();
    m_bufferOut->Buffer[0].Yaw = ang_vel.z();

    m_bufferOut->LaunchedCount = m_gyroSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = sample_time;
}

CH_SENSOR_API void ChFilterGyroscopeUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                       std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("Gyroscope update filter must be applied first in filter graph");
    }

    m_gyroSensor = std::dynamic_pointer_cast<ChGyroscopeSensor>(pSensor);
    if (!m_gyroSensor) {
        throw std::runtime_error("Gyroscope update filter can only be used on a gyroscope\n");
    }

    if (m_noise_model)
        m_noise_model->Initialize(pSensor, RngUsage::GyroscopeNoise, GetRngStreamIndex());

    m_bufferOut = chrono_types::make_shared<SensorHostGyroBuffer>();
    m_bufferOut->Buffer = std::make_unique<GyroData[]>(1);
    m_bufferOut->Width = m_bufferOut->Height = 1;
    m_bufferOut->LaunchedCount = m_gyroSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = 0.f;
    bufferInOut = m_bufferOut;
}

// -----------------------------------------------------------------------------
// ChFilterMagnetometerUpdate
// -----------------------------------------------------------------------------

ChFilterMagnetometerUpdate::ChFilterMagnetometerUpdate(std::shared_ptr<ChNoiseModel> noise_model,
                                                       ChVector3d gps_reference)
    : ChFilter("Magnetometer Updater"), m_noise_model(noise_model), m_gps_reference(gps_reference) {}

CH_SENSOR_API void ChFilterMagnetometerUpdate::Apply() {
    const auto& keyframes = m_magSensor->m_keyframes.Active();
    if (keyframes.empty())
        return;

    // The field is a point property of a position on Earth, not something to integrate over a
    // window, so the sample is taken at the instant the window closed.
    const ChFrame<double>& sensor_frame = keyframes.back();

    ChVector3d field_world(0, 0, 0);
    if (m_magSensor->GetFieldModel() == ChMagneticFieldModel::LOCAL) {
        field_world = m_magSensor->GetLocalField();
    } else {
        ChVector3d coords = sensor_frame.GetPos();
        Cartesian2GPS(coords, m_gps_reference);
        field_world = WMM2025Field(coords.y(), coords.x(), coords.z(), m_magSensor->GetEpoch());
    }

    // RotateBack is parent to local: the reading is the world field expressed in the sensor frame.
    ChVector3d field_sensor = sensor_frame.GetRot().RotateBack(field_world);

    const float sample_time = m_magSensor->GetSampleTime();
    if (m_noise_model) {
        // The first sample has no preceding one to measure an interval against, so it uses the
        // nominal update period. Measuring from a default-constructed zero would hand the model an
        // interval of one simulation step and, for a model that scales noise by the sampling
        // interval, overstate the first sample.
        if (m_have_previous_sample)
            m_noise_model->AddNoise(field_sensor, m_last_sample_time, sample_time);
        else
            m_noise_model->AddNoise(field_sensor);
    }
    m_last_sample_time = sample_time;
    m_have_previous_sample = true;

    m_bufferOut->Buffer[0].X = field_sensor.x();
    m_bufferOut->Buffer[0].Y = field_sensor.y();
    m_bufferOut->Buffer[0].Z = field_sensor.z();

    m_bufferOut->LaunchedCount = m_magSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = sample_time;
}

CH_SENSOR_API void ChFilterMagnetometerUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                          std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("Magnetometer update filter must be applied first in filter graph");
    }

    m_magSensor = std::dynamic_pointer_cast<ChMagnetometerSensor>(pSensor);
    if (!m_magSensor) {
        throw std::runtime_error("Magnetometer update filter can only be used on a magnetometer\n");
    }

    if (m_noise_model)
        m_noise_model->Initialize(pSensor, RngUsage::MagnetometerNoise, GetRngStreamIndex());

    m_bufferOut = chrono_types::make_shared<SensorHostMagnetBuffer>();
    m_bufferOut->Buffer = std::make_unique<MagnetData[]>(1);
    m_bufferOut->Width = m_bufferOut->Height = 1;
    m_bufferOut->LaunchedCount = m_magSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = 0.f;

    bufferInOut = m_bufferOut;
}

}  // namespace sensor
}  // namespace chrono
