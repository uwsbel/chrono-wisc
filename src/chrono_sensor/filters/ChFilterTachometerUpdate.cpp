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

#include <cmath>
#include <cstdlib>
#include <stdexcept>

#include "chrono/utils/ChConstants.h"

#include "chrono_sensor/filters/ChFilterTachometerUpdate.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/sensors/ChTachometerSensor.h"

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
// ChFilterTachometerUpdate
// -----------------------------------------------------------------------------

ChFilterTachometerUpdate::ChFilterTachometerUpdate(std::shared_ptr<ChNoiseModel> noise_model)
    : ChFilter("Tachometer Updater"), m_noise_model(noise_model) {}

CH_SENSOR_API void ChFilterTachometerUpdate::Apply() {
    const auto& keyframes = m_tachSensor->m_keyframes.Active();
    if (keyframes.empty())
        return;

    ChVector3d rate(0, 0, 0);
    for (const auto& keyframe : keyframes)
        rate += keyframe;
    rate /= (double)keyframes.size();

    const float sample_time = m_tachSensor->GetSampleTime();
    // Noise is applied to the whole rate vector before the axis is selected, so a cross-axis term
    // in the noise model reaches the reported axis the way it would in a real part. The first sample
    // has no preceding one to measure an interval against and uses the nominal update period.
    if (m_noise_model) {
        if (m_have_previous_sample)
            m_noise_model->AddNoise(rate, m_last_sample_time, sample_time);
        else
            m_noise_model->AddNoise(rate);
    }
    m_last_sample_time = sample_time;
    m_have_previous_sample = true;

    m_bufferOut->Buffer[0].rpm = (float)(RateAboutAxis(rate, m_tachSensor->GetAxis()) * CH_RAD_S_TO_RPM);
    m_bufferOut->LaunchedCount = m_tachSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = sample_time;
}

CH_SENSOR_API void ChFilterTachometerUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                        std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("Tachometer update filter must be applied first in filter graph");
    }
    m_tachSensor = std::dynamic_pointer_cast<ChTachometerSensor>(pSensor);
    if (!m_tachSensor) {
        throw std::runtime_error("Tachometer update filter can only be used on a tachometer");
    }

    if (m_noise_model)
        m_noise_model->Initialize(pSensor, RngUsage::TachometerNoise, GetRngStreamIndex());

    m_bufferOut = chrono_types::make_shared<SensorHostTachometerBuffer>();
    m_bufferOut->Buffer = std::make_unique<TachometerData[]>(1);
    m_bufferOut->Width = m_bufferOut->Height = 1;
    m_bufferOut->LaunchedCount = m_tachSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = 0.f;

    bufferInOut = m_bufferOut;
}

// -----------------------------------------------------------------------------
// ChFilterEncoderUpdate
// -----------------------------------------------------------------------------

ChFilterEncoderUpdate::ChFilterEncoderUpdate(std::shared_ptr<ChNoiseModel> noise_model)
    : ChFilter("Encoder Updater"), m_noise_model(noise_model) {}

CH_SENSOR_API void ChFilterEncoderUpdate::Apply() {
    const auto& keyframes = m_encoderSensor->m_keyframes.Active();
    if (keyframes.empty())
        return;

    // Integrate the shaft rate to an angle. The integration spans the gap between updates, not just
    // the collection window, because an encoder counts every edge the disc passes; with the default
    // window of zero the window itself holds a single sample and would otherwise contribute nothing.
    float time = m_last_sample_time;
    double rate = m_last_rate;
    for (const auto& keyframe : keyframes) {
        double sample_rate = keyframe.rate;
        if (m_noise_model) {
            ChVector3d noisy(sample_rate, 0, 0);
            m_noise_model->AddNoise(noisy, time, keyframe.time);
            sample_rate = noisy.x();
        }
        if (m_started)
            m_angle += 0.5 * (rate + sample_rate) * ((double)keyframe.time - (double)time);
        m_started = true;
        time = keyframe.time;
        rate = sample_rate;
    }

    const long long previous_counts = m_counts;
    const float previous_time = m_last_sample_time;
    m_last_sample_time = time;
    m_last_rate = rate;

    // Eccentricity of the disc displaces the apparent angle once per revolution.
    double measured_angle = m_angle;
    if (m_encoderSensor->GetEccentricityAmplitude() != 0)
        measured_angle += m_encoderSensor->GetEccentricityAmplitude() *
                          std::sin(m_angle + m_encoderSensor->m_eccentricity_phase);

    // Quantisation to whole counts. This is the encoder's dominant error at low speed and is a
    // property of the device, not noise added on top of it.
    const long long raw_counts = std::llround(measured_angle / m_encoderSensor->GetResolution());
    long long delta = raw_counts - m_raw_counts;
    m_raw_counts = raw_counts;

    // A dropped pulse is lost for good: the count never catches up, which is what makes a dirty
    // disc show as accumulating position error rather than as noise.
    const double drop = m_encoderSensor->GetMissingPulseProbability();
    if (drop > 0 && delta != 0) {
        std::binomial_distribution<long long> dropped(std::llabs(delta), drop);
        delta -= (delta > 0 ? 1 : -1) * dropped(m_generator);
    }
    m_counts += delta;

    const double resolution = m_encoderSensor->GetResolution();
    const double elapsed = (double)m_last_sample_time - (double)previous_time;
    const long long counted = m_counts - previous_counts;

    m_bufferOut->Buffer[0].angle = m_counts * resolution;
    m_bufferOut->Buffer[0].counts = m_counts;
    // Mean speed from whole counts over a fixed interval, which is how an encoder is actually read.
    m_bufferOut->Buffer[0].rpm =
        (elapsed > 0) ? counted * resolution / elapsed * CH_RAD_S_TO_RPM : 0.0;
    m_bufferOut->Buffer[0].direction = (counted > 0) - (counted < 0);

    m_bufferOut->LaunchedCount = m_encoderSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = m_last_sample_time;
}

CH_SENSOR_API void ChFilterEncoderUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                     std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("Encoder update filter must be applied first in filter graph");
    }
    m_encoderSensor = std::dynamic_pointer_cast<ChEncoderSensor>(pSensor);
    if (!m_encoderSensor) {
        throw std::runtime_error("Encoder update filter can only be used on an encoder");
    }

    if (m_noise_model)
        m_noise_model->Initialize(pSensor, RngUsage::EncoderNoise, GetRngStreamIndex());

    const unsigned long long seed =
        ChSensorManager::GetDeterministicSeed(pSensor, RngUsage::EncoderPulseDropout, GetRngStreamIndex());
    std::seed_seq seq{(unsigned int)(seed & 0xFFFFFFFFull), (unsigned int)(seed >> 32)};
    m_generator.seed(seq);

    m_bufferOut = chrono_types::make_shared<SensorHostEncoderBuffer>();
    m_bufferOut->Buffer = std::make_unique<EncoderData[]>(1);
    m_bufferOut->Width = m_bufferOut->Height = 1;
    m_bufferOut->LaunchedCount = m_encoderSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = 0.f;

    bufferInOut = m_bufferOut;
}

}  // namespace sensor
}  // namespace chrono
