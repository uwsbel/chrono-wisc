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
// =============================================================================

#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono/utils/ChUtils.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChSensor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

namespace chrono {
namespace sensor {

namespace {

/// Draw one axis of a first-order Gauss-Markov process from its stationary distribution.
///
/// Used to initialize the state, because a sensor that has been powered on for a while already has
/// a fully developed error. Starting from zero instead would make the first few correlation times
/// of every run unrepresentatively quiet.
double SampleGaussMarkovSteadyState(double stdev, std::mt19937& gen) {
    if (stdev <= 0)
        return 0;
    return std::normal_distribution<double>(0.0, stdev)(gen);
}

/// Advance one axis of a first-order Gauss-Markov process by dt seconds. The update is exact for
/// any dt, which matters because a sensor and its noise model need not share a step size.
double AdvanceGaussMarkov(double state, double dt, double stdev, double tau, std::mt19937& gen) {
    if (stdev <= 0)
        return 0;
    if (tau <= std::numeric_limits<double>::epsilon())
        return std::normal_distribution<double>(0.0, stdev)(gen);

    const double decay = std::exp(-dt / tau);
    std::normal_distribution<double> step(0.0, stdev * std::sqrt(std::max(0.0, 1 - decay * decay)));
    return decay * state + step(gen);
}

}  // namespace

// -----------------------------------------------------------------------------
// ChNoiseModel
// -----------------------------------------------------------------------------

ChNoiseModel::ChNoiseModel() {
    // Clock seeding preserves the historical default for a model used outside a sensor. Initialize
    // replaces it whenever the model reaches a filter, which is every path through ChSensorManager.
    m_generator.seed((unsigned int)std::chrono::high_resolution_clock::now().time_since_epoch().count());
}

void ChNoiseModel::Initialize(const std::shared_ptr<ChSensor>& sensor,
                              RngUsage usage,
                              unsigned int filter_stream_index) {
    if (m_initialized) {
        std::cerr << "WARNING: noise model instance initialized twice, which happens when one instance is shared "
                     "between sensors. The generator and the bias state belong to the instance, so the sensors "
                     "sharing it see one correlated error rather than independent errors. Give each sensor its "
                     "own noise model.\n";
    }
    m_initialized = true;

    // Consecutive stream ids differ by small integers, so seed through a seed_seq rather than
    // handing the low word straight to the engine: that both mixes the two halves of the 64-bit id
    // and decorrelates the early output of streams whose ids are adjacent.
    const unsigned long long id = ChSensorManager::GetDeterministicSeed(sensor, usage, filter_stream_index);
    std::seed_seq seq{(unsigned int)(id & 0xFFFFFFFFull), (unsigned int)(id >> 32)};
    m_generator.seed(seq);
}

// -----------------------------------------------------------------------------
// ChNoiseNormal
// -----------------------------------------------------------------------------

ChNoiseNormal::ChNoiseNormal(ChVector3d mean, ChVector3d stdev) : ChNoiseModel(), m_mean(mean), m_stdev(stdev) {}

void ChNoiseNormal::AddNoise(ChVector3d& data) {
    std::normal_distribution<double> dist_x(m_mean.x(), m_stdev.x());
    std::normal_distribution<double> dist_y(m_mean.y(), m_stdev.y());
    std::normal_distribution<double> dist_z(m_mean.z(), m_stdev.z());
    data += ChVector3d(dist_x(m_generator), dist_y(m_generator), dist_z(m_generator));
}

void ChNoiseNormal::AddNoise(ChVector3d& data, float last_ch_time, float ch_time) {
    AddNoise(data);
}

// -----------------------------------------------------------------------------
// ChNoiseNormalDrift
// -----------------------------------------------------------------------------

ChNoiseNormalDrift::ChNoiseNormalDrift(double updateRate,
                                       ChVector3d mean,
                                       ChVector3d stdev,
                                       double drift_bias,
                                       double tau_drift)
    : ChNoiseModel(),
      m_bias_prev(0, 0, 0),
      m_updateRate(updateRate),
      m_mean(mean),
      m_stdev(stdev),
      m_drift_bias(drift_bias),
      m_tau_drift(tau_drift) {}

void ChNoiseNormalDrift::AddNoise(ChVector3d& data) {
    std::normal_distribution<double> dist_a_x(m_mean.x(), m_stdev.x());
    std::normal_distribution<double> dist_a_y(m_mean.y(), m_stdev.y());
    std::normal_distribution<double> dist_a_z(m_mean.z(), m_stdev.z());
    ChVector3d eta_a = {dist_a_x(m_generator), dist_a_y(m_generator), dist_a_z(m_generator)};

    ChVector3d eta_b = {0, 0, 0};
    // The update rate divides the random-walk step, so guard it as well as the two parameters that
    // already gate this branch: a default-constructed or mis-set rate of zero would give infinity.
    if (m_tau_drift > std::numeric_limits<double>::epsilon() &&
        m_drift_bias > std::numeric_limits<double>::epsilon() &&
        m_updateRate > std::numeric_limits<double>::epsilon()) {
        std::normal_distribution<double> dist_b(0.0, m_drift_bias * std::sqrt(1 / (m_updateRate * m_tau_drift)));
        eta_b = {dist_b(m_generator), dist_b(m_generator), dist_b(m_generator)};
    }
    m_bias_prev += eta_b;
    data += eta_a + m_bias_prev;
}

void ChNoiseNormalDrift::AddNoise(ChVector3d& data, float last_ch_time, float ch_time) {
    AddNoise(data);
}

// -----------------------------------------------------------------------------
// ChNoiseRandomWalks
// -----------------------------------------------------------------------------

ChNoiseRandomWalks::ChNoiseRandomWalks(float mean, float sigma, float noise_model_update_rate, ChVector3d gps_reference)
    : ChNoiseRandomWalks(mean, sigma, noise_model_update_rate, 0.03, 0.005, gps_reference) {}

ChNoiseRandomWalks::ChNoiseRandomWalks(float mean,
                                       float sigma,
                                       float noise_model_update_rate,
                                       double max_velocity,
                                       double max_acceleration,
                                       ChVector3d gps_reference)
    : ChNoiseModel(),
      m_mean(mean),
      m_sigma(sigma),
      // The noise model integrates on its own step, which is generally finer than the GPS update
      // period. Guard the reciprocal so a rate of zero cannot make the step infinite and hang the
      // integration loop.
      m_step_size(noise_model_update_rate > 0 ? 1.0 / noise_model_update_rate : 1.0),
      m_max_velocity(max_velocity),
      m_max_acceleration(max_acceleration),
      m_gps_reference(gps_reference),
      m_last_updated_ch_time(0),
      m_prev_error_p(0, 0, 0),
      m_prev_error_v(0, 0, 0) {}

void ChNoiseRandomWalks::AddNoise(ChVector3d& data, float last_ch_time, float next_ch_time) {
    double curr_time = m_last_updated_ch_time;

    while (curr_time < next_ch_time) {
        const double time_step = std::min(m_step_size, (double)next_ch_time - curr_time);

        // Restoring term: the drift of the white noise is pulled back towards zero in proportion to
        // the error already accumulated, which is what keeps the doubly integrated walk bounded.
        const ChVector3d restore = m_prev_error_p * (-0.2 * m_sigma);

        std::normal_distribution<double> dist_x(m_mean + restore.x(), m_sigma);
        std::normal_distribution<double> dist_y(m_mean + restore.y(), m_sigma);
        std::normal_distribution<double> dist_z(m_mean + restore.z(), m_sigma);
        ChVector3d white_noise = {dist_x(m_generator), dist_y(m_generator), dist_z(m_generator)};

        white_noise.x() = ChClamp(white_noise.x(), -m_max_acceleration, m_max_acceleration);
        white_noise.y() = ChClamp(white_noise.y(), -m_max_acceleration, m_max_acceleration);
        white_noise.z() = ChClamp(white_noise.z(), -m_max_acceleration, m_max_acceleration);

        m_prev_error_v += white_noise * time_step;
        if (m_prev_error_v.Length() > m_max_velocity)
            m_prev_error_v = m_prev_error_v / m_prev_error_v.Length() * m_max_velocity;

        m_prev_error_p += m_prev_error_v * time_step;

        curr_time += time_step;
    }

    m_last_updated_ch_time = next_ch_time;
    data += m_prev_error_p;
}

// -----------------------------------------------------------------------------
// ChNoiseIMU
// -----------------------------------------------------------------------------

ChNoiseIMU::ChNoiseIMU(double update_rate, const ChNoiseIMUParams& params)
    : ChNoiseModel(),
      m_params(params),
      m_update_rate(update_rate > 0 ? update_rate : 1.0),
      m_turn_on_bias(0, 0, 0),
      m_gm_bias(0, 0, 0),
      m_rrw_bias(0, 0, 0),
      m_turn_on_drawn(false) {
    // (I + M) * diag(1 + s), with the misalignment diagonal dropped because the scale factor already
    // carries it. Precomputed because it is constant for the life of the model.
    m_transform = m_params.misalignment;
    m_transform.diagonal().setZero();
    m_transform += ChMatrix33d(1.0);
    for (unsigned int col = 0; col < 3; col++)
        m_transform.col(col) *= 1 + m_params.scale_factor_error[col];
}

void ChNoiseIMU::Initialize(const std::shared_ptr<ChSensor>& sensor,
                            RngUsage usage,
                            unsigned int filter_stream_index) {
    ChNoiseModel::Initialize(sensor, usage, filter_stream_index);

    // Redraw after re-seeding, so the turn-on bias belongs to the deterministic stream rather than
    // to the clock-seeded one the constructor set up.
    m_turn_on_drawn = false;
    DrawTurnOnBias();
    SeedSteadyState();
}

void ChNoiseIMU::DrawTurnOnBias() {
    if (m_turn_on_drawn)
        return;
    m_turn_on_drawn = true;

    if (m_params.turn_on_bias_stdev <= 0) {
        m_turn_on_bias = ChVector3d(0, 0, 0);
        return;
    }
    std::normal_distribution<double> dist(0.0, m_params.turn_on_bias_stdev);
    m_turn_on_bias = ChVector3d(dist(m_generator), dist(m_generator), dist(m_generator));
}

void ChNoiseIMU::SeedSteadyState() {
    m_gm_bias = ChVector3d(SampleGaussMarkovSteadyState(m_params.bias_instability, m_generator),
                           SampleGaussMarkovSteadyState(m_params.bias_instability, m_generator),
                           SampleGaussMarkovSteadyState(m_params.bias_instability, m_generator));
}

void ChNoiseIMU::AddNoise(ChVector3d& data) {
    Apply(data, 1 / m_update_rate);
}

void ChNoiseIMU::AddNoise(ChVector3d& data, float last_ch_time, float ch_time) {
    const double dt = (double)ch_time - (double)last_ch_time;
    Apply(data, dt > 0 ? dt : 1 / m_update_rate);
}

void ChNoiseIMU::Apply(ChVector3d& data, double dt) {
    if (!m_turn_on_drawn) {
        DrawTurnOnBias();
        SeedSteadyState();
    }

    m_gm_bias = ChVector3d(
        AdvanceGaussMarkov(m_gm_bias.x(), dt, m_params.bias_instability, m_params.bias_correlation_time, m_generator),
        AdvanceGaussMarkov(m_gm_bias.y(), dt, m_params.bias_instability, m_params.bias_correlation_time, m_generator),
        AdvanceGaussMarkov(m_gm_bias.z(), dt, m_params.bias_instability, m_params.bias_correlation_time, m_generator));

    if (m_params.rate_random_walk > 0) {
        std::normal_distribution<double> step(0.0, m_params.rate_random_walk * std::sqrt(dt));
        m_rrw_bias += ChVector3d(step(m_generator), step(m_generator), step(m_generator));
    }

    ChVector3d measured = m_transform * data + m_turn_on_bias + m_gm_bias + m_rrw_bias;

    // A noise density is a continuous-time quantity; sampling it over dt gives density / sqrt(dt),
    // which is why changing the update rate leaves the modelled part unchanged.
    if (m_params.noise_density > 0) {
        std::normal_distribution<double> white(0.0, m_params.noise_density / std::sqrt(dt));
        measured += ChVector3d(white(m_generator), white(m_generator), white(m_generator));
    }

    if (m_params.resolution > 0)
        for (unsigned int i = 0; i < 3; i++)
            measured[i] = std::round(measured[i] / m_params.resolution) * m_params.resolution;

    if (m_params.range > 0)
        for (unsigned int i = 0; i < 3; i++)
            measured[i] = ChClamp(measured[i], -m_params.range, m_params.range);

    data = measured;
}

ChNoiseIMUParams ChNoiseIMU::AccelerometerPreset(ChIMUGrade grade) {
    using namespace imu_units;

    ChNoiseIMUParams p;
    switch (grade) {
        case ChIMUGrade::CONSUMER_MEMS:
            p.noise_density = MicroGPerSqrtHz(200);
            p.turn_on_bias_stdev = MilliG(40);
            p.bias_instability = MicroG(60);
            p.bias_correlation_time = 100;
            p.range = MilliG(16000);
            break;
        case ChIMUGrade::INDUSTRIAL_MEMS:
            p.noise_density = MicroGPerSqrtHz(40);
            p.turn_on_bias_stdev = MilliG(3);
            p.bias_instability = MicroG(10);
            p.bias_correlation_time = 300;
            p.range = MilliG(8000);
            break;
        case ChIMUGrade::TACTICAL:
            p.noise_density = MicroGPerSqrtHz(20);
            p.turn_on_bias_stdev = MilliG(0.75);
            p.bias_instability = MicroG(3);
            p.bias_correlation_time = 1000;
            p.range = MilliG(10000);
            break;
    }
    return p;
}

ChNoiseIMUParams ChNoiseIMU::GyroscopePreset(ChIMUGrade grade) {
    using namespace imu_units;

    ChNoiseIMUParams p;
    switch (grade) {
        case ChIMUGrade::CONSUMER_MEMS:
            p.noise_density = DegPerSecPerSqrtHz(0.01);
            p.turn_on_bias_stdev = DegPerSec(1.0);
            p.bias_instability = DegPerHour(12);
            p.bias_correlation_time = 100;
            p.range = DegPerSec(2000);
            break;
        case ChIMUGrade::INDUSTRIAL_MEMS:
            p.noise_density = DegPerSecPerSqrtHz(0.003);
            p.turn_on_bias_stdev = DegPerSec(0.15);
            p.bias_instability = DegPerHour(1.5);
            p.bias_correlation_time = 300;
            p.range = DegPerSec(450);
            break;
        case ChIMUGrade::TACTICAL:
            p.noise_density = DegPerSecPerSqrtHz(0.001);
            p.turn_on_bias_stdev = DegPerSec(0.02);
            p.bias_instability = DegPerHour(0.1);
            p.bias_correlation_time = 1000;
            p.range = DegPerSec(500);
            break;
    }
    return p;
}

// -----------------------------------------------------------------------------
// ChNoiseGPS
// -----------------------------------------------------------------------------

ChNoiseGPS::ChNoiseGPS(const ChNoiseGPSParams& params)
    : ChNoiseModel(),
      m_params(params),
      m_gm_error(0, 0, 0),
      m_slow_error(0, 0, 0),
      m_outage_rate(0),
      m_outage_mean_length(0),
      m_outage_remaining(0),
      m_last_time(0),
      m_started(false) {}

void ChNoiseGPS::SetNominalState(ChGPSFixType fix, double hdop, unsigned int num_satellites) {
    m_nominal_state.fix = fix;
    m_nominal_state.hdop = hdop;
    m_nominal_state.num_satellites = num_satellites;
    m_nominal_state.valid = (fix != ChGPSFixType::NONE);
    if (m_outage_remaining <= 0)
        m_state = m_nominal_state;
}

void ChNoiseGPS::SetOutageModel(double rate, double mean_duration) {
    m_outage_rate = std::max(0.0, rate);
    m_outage_mean_length = std::max(0.0, mean_duration);
}

void ChNoiseGPS::AddNoise(ChVector3d& data, float last_ch_time, float ch_time) {
    // The first call establishes the time origin. Advancing from a default-constructed zero would
    // otherwise run the processes over the whole elapsed simulation in one step, which for a long
    // warm-up puts the Gauss-Markov terms straight at their steady state.
    if (!m_started) {
        m_started = true;
        m_last_time = ch_time;
        SeedSteadyState();
    } else if (ch_time > m_last_time) {
        Advance((double)ch_time - (double)m_last_time);
        m_last_time = ch_time;
    }

    data += m_gm_error + m_slow_error;

    if (m_params.white_stdev > 0) {
        std::normal_distribution<double> white(0.0, m_params.white_stdev);
        data += ChVector3d(white(m_generator), white(m_generator), white(m_generator));
    }
}

void ChNoiseGPS::SeedSteadyState() {
    m_gm_error = ChVector3d(SampleGaussMarkovSteadyState(m_params.horizontal_stdev, m_generator),
                            SampleGaussMarkovSteadyState(m_params.horizontal_stdev, m_generator),
                            SampleGaussMarkovSteadyState(m_params.vertical_stdev, m_generator));
    m_slow_error = ChVector3d(SampleGaussMarkovSteadyState(m_params.slow_bias_stdev, m_generator),
                              SampleGaussMarkovSteadyState(m_params.slow_bias_stdev, m_generator),
                              SampleGaussMarkovSteadyState(m_params.slow_bias_stdev, m_generator));
}

void ChNoiseGPS::Advance(double dt) {
    m_gm_error = ChVector3d(AdvanceGaussMarkov(m_gm_error.x(), dt, m_params.horizontal_stdev,
                                               m_params.correlation_time, m_generator),
                            AdvanceGaussMarkov(m_gm_error.y(), dt, m_params.horizontal_stdev,
                                               m_params.correlation_time, m_generator),
                            AdvanceGaussMarkov(m_gm_error.z(), dt, m_params.vertical_stdev,
                                               m_params.correlation_time, m_generator));

    if (m_params.slow_bias_stdev > 0) {
        m_slow_error = ChVector3d(AdvanceGaussMarkov(m_slow_error.x(), dt, m_params.slow_bias_stdev,
                                                     m_params.slow_bias_correlation_time, m_generator),
                                  AdvanceGaussMarkov(m_slow_error.y(), dt, m_params.slow_bias_stdev,
                                                     m_params.slow_bias_correlation_time, m_generator),
                                  AdvanceGaussMarkov(m_slow_error.z(), dt, m_params.slow_bias_stdev,
                                                     m_params.slow_bias_correlation_time, m_generator));
    }

    if (m_outage_remaining > 0) {
        m_outage_remaining -= dt;
        if (m_outage_remaining <= 0) {
            m_outage_remaining = 0;
            m_state = m_nominal_state;
        }
        return;
    }

    if (m_outage_rate > 0 && m_outage_mean_length > 0 && dt > 0) {
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        if (uniform(m_generator) < 1 - std::exp(-m_outage_rate * dt)) {
            m_outage_remaining = std::exponential_distribution<double>(1 / m_outage_mean_length)(m_generator);
            m_state.fix = ChGPSFixType::NONE;
            m_state.hdop = 99.99;
            m_state.num_satellites = 0;
            m_state.valid = false;
        }
    }
}

ChNoiseGPSParams ChNoiseGPS::Preset(ChGPSReceiverClass receiver_class) {
    ChNoiseGPSParams p;
    switch (receiver_class) {
        case ChGPSReceiverClass::SPS:
            p.horizontal_stdev = 2.0;
            p.vertical_stdev = 3.0;
            p.correlation_time = 200;
            p.white_stdev = 0.4;
            break;
        case ChGPSReceiverClass::SBAS:
            p.horizontal_stdev = 0.7;
            p.vertical_stdev = 1.2;
            p.correlation_time = 200;
            p.white_stdev = 0.2;
            break;
        case ChGPSReceiverClass::RTK_FLOAT:
            p.horizontal_stdev = 0.3;
            p.vertical_stdev = 0.5;
            p.correlation_time = 60;
            p.white_stdev = 0.05;
            break;
        case ChGPSReceiverClass::RTK_FIXED:
            p.horizontal_stdev = 0.015;
            p.vertical_stdev = 0.025;
            p.correlation_time = 60;
            p.white_stdev = 0.005;
            break;
    }
    return p;
}

ChGPSFixType ChNoiseGPS::PresetFixType(ChGPSReceiverClass receiver_class) {
    switch (receiver_class) {
        case ChGPSReceiverClass::SPS:
            return ChGPSFixType::SPS;
        case ChGPSReceiverClass::SBAS:
            return ChGPSFixType::DGPS;
        case ChGPSReceiverClass::RTK_FLOAT:
            return ChGPSFixType::RTK_FLOAT;
        case ChGPSReceiverClass::RTK_FIXED:
            return ChGPSFixType::RTK_FIXED;
    }
    return ChGPSFixType::SPS;
}

}  // namespace sensor
}  // namespace chrono
