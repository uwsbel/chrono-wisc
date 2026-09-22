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

#ifndef CH_NOISE_MODEL_H
#define CH_NOISE_MODEL_H

#include <memory>
#include <random>

#include "chrono_sensor/ChApiSensor.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChConstants.h"

namespace chrono {
namespace sensor {

// Forward declared rather than included: ChSensorManager.h pulls in the render backends, and a
// noise model is used from headers that must stay free of them. RngUsage has a fixed underlying
// type, so it can be declared without its definition.
class ChSensor;
enum class RngUsage : unsigned int;

/// @addtogroup sensor_sensors
/// @{

/// Noise model base class.
class CH_SENSOR_API ChNoiseModel {
  public:
    virtual ~ChNoiseModel() {}

    /// Function for adding noise to data.
    /// @param data data to augment
    virtual void AddNoise(ChVector3d& data) = 0;

    /// Function for adding noise over a time interval.
    /// @param data data to augment
    /// @param last_ch_time the last time the data was updated
    /// @param ch_time the current time
    virtual void AddNoise(ChVector3d& data, float last_ch_time, float ch_time) = 0;

    /// Seed this model's random stream from the owning sensor's deterministic stream identity.
    ///
    /// Called once by the update filter that owns the model. Without it a model seeds from the wall
    /// clock and ChSensorManager::SetRandomSeed cannot reach it, so a run with a fixed seed is not
    /// reproducible.
    ///
    /// A model instance carries bias state as well as a generator, so one instance shared between
    /// two sensors gives them a single correlated error rather than two independent ones. That case
    /// is reported here rather than left to produce quietly wrong results.
    ///
    /// @param sensor the sensor owning the filter that owns this model
    /// @param usage the RngUsage constant naming this model's purpose
    /// @param filter_stream_index the owning filter's ChFilter::GetRngStreamIndex()
    virtual void Initialize(const std::shared_ptr<ChSensor>& sensor,
                            RngUsage usage,
                            unsigned int filter_stream_index);

  protected:
    ChNoiseModel();

    /// Random number generator, seeded from the wall clock at construction and re-seeded
    /// deterministically by Initialize.
    std::mt19937 m_generator;

  private:
    bool m_initialized = false;  ///< whether Initialize has already run, to detect a shared instance
};

/// Class implementing no noise model.
class CH_SENSOR_API ChNoiseNone : public ChNoiseModel {
  public:
    ChNoiseNone() {}
    ~ChNoiseNone() {}

    /// Function for adding noise to a set of values.
    virtual void AddNoise(ChVector3d& data) override {}

    /// Function for adding noise over a time interval.
    virtual void AddNoise(ChVector3d& data, float last_ch_time, float ch_time) override {}
};

/// Noise model based on a normal distribution.
///
/// Draws an independent sample per axis per update, with no correlation between updates. For a GPS
/// this understates reality badly -- real position error is dominated by terms that stay correlated
/// for tens of seconds to hours -- so ChNoiseGPS is the better choice for anything that feeds a
/// state estimator. This model remains for magnetometers, for simple studies, and for compatibility.
class CH_SENSOR_API ChNoiseNormal : public ChNoiseModel {
  public:
    /// Class constructor.
    /// @param mean The mean of the normal distribution
    /// @param stdev The standard deviation of the normal distribution
    ChNoiseNormal(ChVector3d mean, ChVector3d stdev);

    ~ChNoiseNormal() {}

    /// Noise addition function.
    virtual void AddNoise(ChVector3d& data) override;

    /// Function for adding noise over a time interval. The distribution has no state, so the times
    /// are not used and this is equivalent to the single-argument form.
    virtual void AddNoise(ChVector3d& data, float last_ch_time, float ch_time) override;

  private:
    ChVector3d m_mean;   ///< mean of the normal distribution
    ChVector3d m_stdev;  ///< standard deviation of the normal distribution
};

/// IMU noise model: Gaussian white noise plus an unbounded random-walk bias.
///
/// Per update the model adds `N(mean, stdev)` and a bias that accumulates
/// `N(0, drift_bias * sqrt(1 / (updateRate * tau_drift)))`. Both parameters are per-sample
/// quantities, so changing the update rate silently changes the amount of noise, and the bias grows
/// without bound because nothing pulls it back towards zero.
///
/// Neither property matches a real inertial sensor, and neither can be set from a datasheet. Use
/// ChNoiseIMU for new work; this model is kept so existing studies reproduce.
class CH_SENSOR_API ChNoiseNormalDrift : public ChNoiseModel {
  public:
    /// Class constructor.
    /// @param updateRate Rate at which the owning sensor updates, in Hz
    /// @param mean Mean of the per-sample white noise
    /// @param stdev Standard deviation of the per-sample white noise
    /// @param drift_bias Scale of the bias random walk
    /// @param tau_drift Time constant dividing the bias random-walk step; despite the name, nothing
    /// relaxes the bias back towards zero
    ChNoiseNormalDrift(double updateRate, ChVector3d mean, ChVector3d stdev, double drift_bias, double tau_drift);

    ~ChNoiseNormalDrift() {}

    /// Function for adding noise to data.
    virtual void AddNoise(ChVector3d& data) override;

    /// Function for adding noise over a time interval. The model advances one step per call by
    /// construction, so the times are not used and this is equivalent to the single-argument form.
    virtual void AddNoise(ChVector3d& data, float last_ch_time, float ch_time) override;

  private:
    ChVector3d m_bias_prev;  ///< accumulated bias

    double m_updateRate;  ///< update rate of the owning sensor
    ChVector3d m_mean;    ///< mean of the per-sample white noise
    ChVector3d m_stdev;   ///< standard deviation of the per-sample white noise
    double m_drift_bias;  ///< scale of the bias random walk
    double m_tau_drift;   ///< time constant dividing the bias random-walk step
};

/// GPS noise model based on a bounded random walk.
///
/// Integrates clipped white noise twice, with a restoring term proportional to the accumulated
/// error, giving a smooth wander that stays bounded. The shape is qualitatively right for GNSS
/// error, but none of the parameters maps to a receiver specification, and the clipping makes the
/// result non-Gaussian. Use ChNoiseGPS to work from a receiver's stated accuracy; this model is
/// kept so existing studies reproduce.
class CH_SENSOR_API ChNoiseRandomWalks : public ChNoiseModel {
  public:
    /// Class constructor with default tuning parameters.
    /// @param mean The mean of the normal distribution of the acceleration that is integrated twice to provide the
    /// random walk
    /// @param sigma The standard deviation of the normal distribution of the acceleration that is integrated twice to
    /// provide the random walk
    /// @param noise_model_update_rate The update rate of the noise model which defines the integration step size. Note:
    /// This is different from the sensor update rate.
    /// @param gps_reference Retained for signature compatibility; the model operates in metres and
    /// does not use it
    ChNoiseRandomWalks(float mean, float sigma, float noise_model_update_rate, ChVector3d gps_reference);

    /// Class constructor with custom tuning parameters.
    /// @param mean The mean of the normal distribution of the acceleration that is integrated twice to provide the
    /// random walk
    /// @param sigma The standard deviation of the normal distribution of the acceleration that is integrated twice to
    /// provide the random walk
    /// @param noise_model_update_rate The update rate of the noise model which defines the integration step size. Note:
    /// This is different from the sensor update rate.
    /// @param max_velocity The maximum allowable velocity for the random walk
    /// @param max_acceleration The maximum allowable acceleration for the random walk
    /// @param gps_reference Retained for signature compatibility; the model operates in metres and
    /// does not use it
    ChNoiseRandomWalks(float mean,
                       float sigma,
                       float noise_model_update_rate,
                       double max_velocity,
                       double max_acceleration,
                       ChVector3d gps_reference);

    ~ChNoiseRandomWalks() {}

    /// The walk advances over an interval, so this form has nothing to advance over and leaves the
    /// data unchanged. Use the three-argument form.
    virtual void AddNoise(ChVector3d& data) override {}

    /// Function for adding noise over a time interval.
    virtual void AddNoise(ChVector3d& data, float last_ch_time, float ch_time) override;

  private:
    double m_mean;                 ///< mean of the white noise driving the walk
    double m_sigma;                ///< standard deviation of the white noise driving the walk
    double m_step_size;            ///< integration step of the walk
    double m_max_velocity;         ///< bound on the first derivative of the walk
    double m_max_acceleration;     ///< bound on the white noise driving the walk
    ChVector3d m_gps_reference;    ///< retained for signature compatibility; unused
    float m_last_updated_ch_time;  ///< time up to which the walk has been integrated
    ChVector3d m_prev_error_p;     ///< accumulated position error
    ChVector3d m_prev_error_v;     ///< accumulated first derivative of the position error
};

// -----------------------------------------------------------------------------
// Datasheet-parameterised models
// -----------------------------------------------------------------------------

/// Unit conversions for typing inertial datasheet figures directly into ChNoiseIMUParams, whose
/// fields are all SI: m/s^2 for an accelerometer, rad/s for a gyroscope.
namespace imu_units {

/// Accelerometer noise density, ug/sqrt(Hz) -> (m/s^2)/sqrt(Hz).
constexpr double MicroGPerSqrtHz(double v) {
    return v * 1e-6 * 9.80665;
}

/// Accelerometer bias, ug -> m/s^2.
constexpr double MicroG(double v) {
    return v * 1e-6 * 9.80665;
}

/// Accelerometer bias, mg -> m/s^2.
constexpr double MilliG(double v) {
    return v * 1e-3 * 9.80665;
}

/// Gyroscope noise density, (deg/s)/sqrt(Hz) -> (rad/s)/sqrt(Hz).
constexpr double DegPerSecPerSqrtHz(double v) {
    return v * CH_DEG_TO_RAD;
}

/// Gyroscope bias, deg/h -> rad/s.
constexpr double DegPerHour(double v) {
    return v * CH_DEG_TO_RAD / 3600.0;
}

/// Gyroscope bias or rate, deg/s -> rad/s.
constexpr double DegPerSec(double v) {
    return v * CH_DEG_TO_RAD;
}

}  // namespace imu_units

/// Grade of inertial sensor, selecting a set of representative error parameters.
///
/// The presets are order-of-magnitude figures for each class, not any specific part. Copy the
/// numbers from the datasheet of the part being modelled whenever they are available.
enum class ChIMUGrade {
    CONSUMER_MEMS,    ///< phone-class and robot-kit parts
    INDUSTRIAL_MEMS,  ///< ADIS / STIM class
    TACTICAL          ///< fibre-optic and ring-laser gyroscopes
};

/// Error terms of an inertial sensor, in the quantities a datasheet quotes.
///
/// All fields are SI and per axis: m/s^2 for an accelerometer, rad/s for a gyroscope. The imu_units
/// helpers convert the units datasheets actually use.
struct CH_SENSOR_API ChNoiseIMUParams {
    /// Velocity or angular random walk expressed as a power spectral density, in sensor units per
    /// sqrt(Hz). This is a density, not a per-sample standard deviation: the per-sample value is
    /// derived from it and the sampling interval, so changing the update rate leaves the underlying
    /// noise unchanged. Appears as the slope -1/2 branch of an Allan deviation plot.
    double noise_density = 0.0;

    /// Standard deviation of the constant bias drawn once per run. For MEMS parts this turn-on bias
    /// is usually the largest single error term, and it is the one a white-noise model omits.
    double turn_on_bias_stdev = 0.0;

    /// Steady-state standard deviation of the bias instability, modelled as a first-order
    /// Gauss-Markov process. Unlike a random walk this is bounded, which is what a real
    /// flicker-dominated bias looks like over hours. Appears as the floor of an Allan deviation plot.
    double bias_instability = 0.0;

    /// Correlation time of the bias instability, in seconds.
    double bias_correlation_time = 100.0;

    /// Rate random walk, in sensor units per sqrt(s^3): the unbounded component, and the slope +1/2
    /// branch at the right of an Allan deviation plot.
    double rate_random_walk = 0.0;

    /// Scale factor error per axis, dimensionless; 1e-6 per ppm.
    ChVector3d scale_factor_error = ChVector3d(0, 0, 0);

    /// Cross-axis misalignment in radians. Entry (i, j) is the leakage of true axis j into measured
    /// axis i; the diagonal is ignored because scale_factor_error covers it.
    ChMatrix33d misalignment = ChMatrix33d(0.0);

    /// Measurement range in sensor units. Readings are clamped to +-range; zero disables clamping.
    double range = 0.0;

    /// Quantisation step of the digital output, in sensor units. Zero disables quantisation.
    double resolution = 0.0;
};

/// Inertial sensor noise model parameterised by datasheet quantities.
///
/// Applied per axis to the specific force of an accelerometer or the angular rate of a gyroscope:
///
///     measured = (I + M) * diag(1 + s) * true + b_turnon + b_gm + b_rrw + N(0, sigma_w)
///
/// with `sigma_w = noise_density / sqrt(dt)`, `b_gm` a bounded first-order Gauss-Markov process and
/// `b_rrw` a random walk. The result is quantised to the configured resolution and clamped to the
/// configured range.
///
/// Gyroscope g-sensitivity is deliberately absent: it needs the specific force acting on the part,
/// which a gyroscope model has no access to.
class CH_SENSOR_API ChNoiseIMU : public ChNoiseModel {
  public:
    /// Class constructor.
    /// @param update_rate Rate at which the owning sensor updates, in Hz. Used to derive the
    /// per-sample white noise when a call supplies no interval.
    /// @param params The error terms of the part being modelled
    ChNoiseIMU(double update_rate, const ChNoiseIMUParams& params);

    ~ChNoiseIMU() {}

    /// Add one sample of noise, advancing the bias processes by one update period.
    virtual void AddNoise(ChVector3d& data) override;

    /// Add one sample of noise, advancing the bias processes over the given interval.
    virtual void AddNoise(ChVector3d& data, float last_ch_time, float ch_time) override;

    /// Re-seed, and redraw the turn-on bias so it belongs to the deterministic stream.
    virtual void Initialize(const std::shared_ptr<ChSensor>& sensor,
                            RngUsage usage,
                            unsigned int filter_stream_index) override;

    /// Get the total bias currently applied, summing the turn-on, Gauss-Markov and random-walk
    /// terms. Exposed so a test or an estimator study can compare against the truth it injected.
    /// @return The bias per axis, in sensor units
    ChVector3d GetBias() const { return m_turn_on_bias + m_gm_bias + m_rrw_bias; }

    /// Get the parameters this model was built with.
    /// @return The error terms in use
    const ChNoiseIMUParams& GetParameters() const { return m_params; }

    /// Representative accelerometer parameters for a grade of part, in m/s^2.
    /// @param grade The grade of part to model
    /// @return Parameters to hand to the constructor
    static ChNoiseIMUParams AccelerometerPreset(ChIMUGrade grade);

    /// Representative gyroscope parameters for a grade of part, in rad/s.
    /// @param grade The grade of part to model
    /// @return Parameters to hand to the constructor
    static ChNoiseIMUParams GyroscopePreset(ChIMUGrade grade);

  private:
    /// Advance the bias processes and add one sample of noise over an interval of dt seconds.
    void Apply(ChVector3d& data, double dt);

    /// Draw the turn-on bias, once per run, after the generator has its final seed.
    void DrawTurnOnBias();

    /// Put the bias instability at its stationary distribution, so a run does not begin with an
    /// unrepresentatively quiet first few correlation times.
    void SeedSteadyState();

    ChNoiseIMUParams m_params;  ///< error terms of the modelled part
    double m_update_rate;       ///< update rate of the owning sensor, in Hz
    ChMatrix33d m_transform;    ///< (I + M) * diag(1 + s), precomputed

    ChVector3d m_turn_on_bias;  ///< constant bias for this run
    ChVector3d m_gm_bias;       ///< bias instability state
    ChVector3d m_rrw_bias;      ///< rate random walk state
    bool m_turn_on_drawn;       ///< whether the turn-on bias has been drawn yet
};

/// Class of GNSS receiver, selecting a set of representative error parameters.
///
/// The presets are representative of each class rather than of any specific receiver. Prefer the
/// accuracy figures published for the receiver being modelled.
enum class ChGPSReceiverClass {
    SPS,        ///< single-frequency standard positioning service
    SBAS,       ///< satellite-based augmentation, or local differential
    RTK_FLOAT,  ///< real-time kinematic with a float ambiguity solution
    RTK_FIXED   ///< real-time kinematic with a fixed ambiguity solution
};

/// Quality of the position solution reported alongside a GPS sample.
enum class ChGPSFixType {
    NONE = 0,       ///< no position solution
    SPS = 1,        ///< standard positioning service
    DGPS = 2,       ///< differential or satellite-based augmentation
    RTK_FLOAT = 3,  ///< real-time kinematic, float ambiguities
    RTK_FIXED = 4   ///< real-time kinematic, fixed ambiguities
};

/// Error terms of a GNSS receiver, in the quantities a receiver specification quotes.
///
/// All lengths are metres of position error in the local ENU frame.
struct CH_SENSOR_API ChNoiseGPSParams {
    /// Standard deviation of the correlated horizontal error: the ionospheric, tropospheric,
    /// ephemeris and multipath terms that dominate real GNSS error.
    double horizontal_stdev = 2.0;

    /// Standard deviation of the correlated vertical error, typically 1.5 to 2 times horizontal
    /// because satellite geometry constrains height more weakly than position.
    double vertical_stdev = 3.0;

    /// Correlation time of that error, in seconds.
    double correlation_time = 200.0;

    /// Standard deviation of the receiver tracking noise, uncorrelated between epochs.
    double white_stdev = 0.4;

    /// Standard deviation of a second, much slower correlated term, for runs long enough that
    /// ionospheric conditions change. Zero disables it.
    double slow_bias_stdev = 0.0;

    /// Correlation time of the slow term, in seconds.
    double slow_bias_correlation_time = 3600.0;
};

/// GNSS receiver position error parameterised by a receiver's stated accuracy.
///
/// Applied to the antenna position in metres in the local ENU frame, before the conversion to
/// latitude and longitude, which is where a real receiver's error enters:
///
///     error = b_gm + b_slow + N(0, white_stdev)
///
/// `b_gm` is a first-order Gauss-Markov process with the receiver's stated accuracy as its
/// steady-state standard deviation and a correlation time of tens of seconds to hours. That
/// correlation is the point: modelling GNSS error as white noise per epoch, as ChNoiseNormal does,
/// makes a downstream estimator look far better than it would on real data.
class CH_SENSOR_API ChNoiseGPS : public ChNoiseModel {
  public:
    /// Receiver status accompanying a sample.
    struct State {
        ChGPSFixType fix = ChGPSFixType::SPS;  ///< quality of the position solution
        double hdop = 1.0;                     ///< horizontal dilution of precision
        unsigned int num_satellites = 10;      ///< satellites used in the solution
        bool valid = true;                     ///< whether the solution is usable
    };

    /// Class constructor.
    /// @param params The error terms of the receiver being modelled
    ChNoiseGPS(const ChNoiseGPSParams& params);

    ~ChNoiseGPS() {}

    /// The error evolves over an interval, so this form has nothing to advance over and leaves the
    /// data unchanged. Use the three-argument form.
    virtual void AddNoise(ChVector3d& data) override {}

    /// Advance the error processes to ch_time and add the accumulated error to data.
    virtual void AddNoise(ChVector3d& data, float last_ch_time, float ch_time) override;

    /// Set the status reported while the receiver has a fix.
    /// @param fix Quality of the position solution
    /// @param hdop Horizontal dilution of precision
    /// @param num_satellites Satellites used in the solution
    void SetNominalState(ChGPSFixType fix, double hdop, unsigned int num_satellites);

    /// Model loss of fix.
    ///
    /// At each epoch an outage begins with probability `rate * dt` and lasts an exponentially
    /// distributed interval of the given mean. The error processes keep evolving during an outage,
    /// so reacquiring does not snap the solution back towards truth.
    ///
    /// @param rate Expected number of outages per second; zero disables the model
    /// @param mean_duration Mean length of an outage, in seconds
    void SetOutageModel(double rate, double mean_duration);

    /// Get the receiver status accompanying the most recent sample.
    /// @return The current fix type, dilution of precision, satellite count and validity
    const State& GetState() const { return m_state; }

    /// Representative parameters for a class of receiver.
    /// @param receiver_class The class of receiver to model
    /// @return Parameters to hand to the constructor
    static ChNoiseGPSParams Preset(ChGPSReceiverClass receiver_class);

    /// The fix type a class of receiver reports while it has a fix.
    /// @param receiver_class The class of receiver
    /// @return The corresponding fix type
    static ChGPSFixType PresetFixType(ChGPSReceiverClass receiver_class);

  private:
    /// Advance the error processes by dt seconds.
    void Advance(double dt);

    /// Put the correlated errors at their stationary distribution, so a run does not begin with an
    /// unrepresentatively accurate fix.
    void SeedSteadyState();

    ChNoiseGPSParams m_params;  ///< error terms of the modelled receiver

    ChVector3d m_gm_error;    ///< correlated error state
    ChVector3d m_slow_error;  ///< slow correlated error state

    double m_outage_rate;          ///< expected outages per second
    double m_outage_mean_length;   ///< mean outage length, in seconds
    double m_outage_remaining;     ///< time left in the current outage, in seconds

    State m_state;         ///< status accompanying the current sample
    State m_nominal_state; ///< status reported while the receiver has a fix

    float m_last_time;     ///< time up to which the error processes have been advanced
    bool m_started;        ///< whether m_last_time holds a real time yet
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
