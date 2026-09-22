IMU Sensor Model {#IMU_sensor}
=================================

\tableofcontents

Chrono::Sensor supports three sensors commonly used collectively as an IMU. These are accelerometer, gyroscope, and magnetometer.

## Conventions

These apply to all three sensors, and getting them wrong is the most common source of readings that
look plausible but are not.

- **Mounting frame.** `offsetPose` is resolved against the parent body's *reference* (REF) frame,
  `ChBody::GetFrameRefToAbs()` — the frame visual and collision shapes are placed in, and the frame
  render sensors mount on. For a plain `ChBody` this is the centroidal frame. For a `ChBodyAuxRef`,
  which every Chrono::Vehicle chassis is, it is not, so a camera and an IMU given the same
  `offsetPose` sit at the same physical point.
- **Sensor frame.** Every reading is expressed in the sensor's own frame: the body reference frame
  composed with `offsetPose.GetRot()`. A sensor mounted rotated on its parent reports about its own
  axes.
- **Units.** Accelerometer m/s², gyroscope rad/s, magnetometer **Tesla**.
- **World frame.** The simulation frame is East-North-Up: +X east, +Y north, +Z up.

## Accelerometer

An accelerometer reports **specific force**, not acceleration: the non-gravitational force per unit
mass supporting the sensor, resolved in the sensor frame.

\f[ f_s = (R_{body} R_{offset})^{-1} (a_{world} - g_{world}) \f]

A level sensor at rest therefore reads \f$+9.81\ \mathrm{m/s^2}\f$ on its up axis, not zero and not
\f$-9.81\f$. The lever arm from a nonzero `offsetPose` position is included, so a sensor offset from
the axis of a rotating body sees the centripetal term.

~~~{.cpp}
// Error terms taken from the grade of part; see "Noise models" below.
auto noise_model = chrono_types::make_shared<ChNoiseIMU>(
    100.0,                                                             // sensor update rate [Hz]
    ChNoiseIMU::AccelerometerPreset(ChIMUGrade::INDUSTRIAL_MEMS)
);

auto acc = chrono_types::make_shared<ChAccelerometerSensor>(
    parent_body,        // body to which the IMU is attached
    imu_update_rate,    // update rate
    imu_offset_pose,    // offset pose from body
    noise_model         // noise model
);

acc->SetName("Accelerometer");
acc->SetLag(.001);               // 1 millisecond lag before the data reaches the user
acc->SetCollectionWindow(.001);  // 1 millisecond collection time

acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
manager->AddSensor(acc);
~~~

### Bandwidth

By default the reported sample is the mean of the keyframes collected over the collection window, so
the effective bandwidth is set by whatever update rate is chosen and content above half that rate
aliases into the band. `SetBandwidth` models the analogue low-pass a real part applies ahead of its
sampler, advanced every simulation step rather than only inside the collection window:

~~~{.cpp}
acc->SetBandwidth(40.0, ChIMUBandwidth::SECOND);  // -3 dB at 40 Hz, two poles
~~~

A cutoff of about 0.4 times the update rate is a reasonable starting point.

### A note on contact spikes

Acceleration comes from the integrator. Under NSC contacts it is an impulse divided by the step
size, so contact events appear as spikes whose amplitude depends on the step size. Real
accelerometers do see impacts, but the amplitude here is a numerical artifact rather than physics.
A bandwidth model damps it; an SMC contact formulation avoids it.

## Gyroscope

Reports the angular velocity of the parent body, resolved in the sensor frame. Angular velocity is
the same about every point of a rigid body, so only the offset *rotation* affects the reading.

~~~{.cpp}
auto noise_model = chrono_types::make_shared<ChNoiseIMU>(
    100.0, ChNoiseIMU::GyroscopePreset(ChIMUGrade::INDUSTRIAL_MEMS));

auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(
    my_body,            // body to which the IMU is attached
    imu_update_rate,    // update rate
    imu_offset_pose,    // offset pose from body
    noise_model         // noise model
);

gyro->SetName("Gyroscope");
gyro->SetLag(.001);
gyro->SetCollectionWindow(.001);

gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
manager->AddSensor(gyro);
~~~

`SetBandwidth` is available here too, with the same meaning.

## Magnetometer

Reports the Earth magnetic field at the sensor position, resolved in the sensor frame, **in Tesla**.

Two field models are available:

- `ChMagneticFieldModel::WMM2025` (default) — the World Magnetic Model 2025, a degree-12 spherical
  harmonic expansion published by NOAA NCEI and the British Geological Survey, valid from 2025.0 to
  2030.0 and accurate to roughly 150 nT and half a degree of declination anywhere on Earth. It is
  evaluated at the sensor's latitude, longitude and altitude, derived from the simulation position
  and the `gps_reference`.
- `ChMagneticFieldModel::LOCAL` — a constant vector supplied by the caller, in the simulation ENU
  frame. Appropriate when the field has been measured at the site, or when the study needs a field
  that does not vary with position.

In the northern hemisphere the field points north and **into the ground**, so its ENU Z component is
negative.

~~~{.cpp}
auto noise_model = chrono_types::make_shared<ChNoiseNormal>(
    ChVector3d({0, 0, 0}),           // mean [T]
    ChVector3d({2e-7, 2e-7, 2e-7})   // stdev [T], a few hundred nT
);

// gps_reference is (LONGITUDE, LATITUDE, ALTITUDE) -- longitude first. See the GPS manual.
ChVector3d gps_reference(-89.400, 43.070, 260.0);  // Madison, WI

auto mag = chrono_types::make_shared<ChMagnetometerSensor>(
    my_body,            // body to which the IMU is attached
    100.f,              // update rate of 100 Hz
    imu_offset_pose,    // offset pose from body
    noise_model,        // noise model
    gps_reference       // GPS coordinates of the simulation origin
);

mag->SetName("Magnetometer");
mag->SetLag(.001);
mag->SetCollectionWindow(.001);

// Optional: pin a measured local field instead of the global model.
// mag->SetLocalField(ChVector3d(-9.6e-7, 1.87e-5, -5.01e-5));

mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
manager->AddSensor(mag);
~~~

## Noise models

Two families are available. Both seed from ChSensorManager::SetRandomSeed when one is set, so a run
with a fixed seed reproduces exactly. Give each sensor **its own** model instance: a model carries
bias state as well as a generator, so a shared instance gives the sensors one correlated error
instead of two independent ones.

### ChNoiseIMU

Parameterised by the quantities a datasheet quotes, so numbers can be copied from the part being
modelled:

\f[ \text{measured} = (I + M)\,\mathrm{diag}(1 + s)\,\text{true} + b_{turnon} + b_{gm} + b_{rrw} + N(0, \sigma_w) \f]

| Field | Meaning |
| --- | --- |
| `noise_density` | Velocity or angular random walk, as a power spectral density in sensor units per √Hz. A density, not a per-sample standard deviation, so changing the update rate leaves the modelled part unchanged. |
| `turn_on_bias_stdev` | Constant bias drawn once per run. For MEMS parts this is usually the largest single error term. |
| `bias_instability`, `bias_correlation_time` | Bias instability as a bounded first-order Gauss-Markov process, which is what a real flicker-dominated bias looks like over hours. |
| `rate_random_walk` | The unbounded component, in sensor units per √(s³). |
| `scale_factor_error`, `misalignment` | Diagonal scale error and cross-axis leakage. |
| `range`, `resolution` | Saturation and quantisation of the digital output. |

`ChNoiseIMU::AccelerometerPreset` and `ChNoiseIMU::GyroscopePreset` return representative parameters
for `ChIMUGrade::CONSUMER_MEMS`, `INDUSTRIAL_MEMS` or `TACTICAL`. They are order-of-magnitude figures
for each class, not any specific part. The `imu_units` helpers convert from datasheet units:

~~~{.cpp}
using namespace chrono::sensor::imu_units;

ChNoiseIMUParams params;
params.noise_density      = MicroGPerSqrtHz(40);   // 40 ug/sqrt(Hz)  -> (m/s^2)/sqrt(Hz)
params.turn_on_bias_stdev = MilliG(3);             // +-3 mg          -> m/s^2
params.bias_instability   = MicroG(10);            // 10 ug           -> m/s^2
params.bias_correlation_time = 300;                // seconds
params.range              = MilliG(8000);          // +-8 g
~~~

For a gyroscope, `DegPerSecPerSqrtHz`, `DegPerHour` and `DegPerSec` do the same job.

Gyroscope g-sensitivity is deliberately absent: it needs the specific force acting on the part,
which a gyroscope model has no access to.

### ChNoiseNormalDrift

The original model: Gaussian white noise plus an *unbounded* random-walk bias. Both parameters are
per-sample quantities, so changing the update rate silently changes the amount of noise, and the bias
grows without bound because nothing pulls it back towards zero. Neither property matches a real
inertial sensor and neither can be set from a datasheet. It is kept so that existing studies
reproduce; prefer `ChNoiseIMU` for new work.

## IMU Data Access

~~~{.cpp}
ChWriterCSV imu_csv(" ");
UserAccelBufferPtr bufferAcc;
UserGyroBufferPtr bufferGyro;
UserMagnetBufferPtr bufferMag;
unsigned int imu_last_launch = 0;
while () {
  bufferAcc = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
  bufferGyro = gyro->GetMostRecentBuffer<UserGyroBufferPtr>();
  bufferMag = mag->GetMostRecentBuffer<UserMagnetBufferPtr>();
  if (bufferAcc->Buffer && bufferGyro->Buffer && bufferMag->Buffer &&
      bufferMag->LaunchedCount > imu_last_launch) {
      // Save the imu data to file
      AccelData acc_data = bufferAcc->Buffer[0];
      GyroData gyro_data = bufferGyro->Buffer[0];
      MagnetData mag_data = bufferMag->Buffer[0];
      imu_csv << std::fixed << std::setprecision(6);
      imu_csv << acc_data.X;      // m/s^2
      imu_csv << acc_data.Y;
      imu_csv << acc_data.Z;
      imu_csv << gyro_data.Roll;  // rad/s
      imu_csv << gyro_data.Pitch;
      imu_csv << gyro_data.Yaw;
      imu_csv << mag_data.X;      // Tesla
      imu_csv << mag_data.Y;
      imu_csv << mag_data.Z;
      imu_csv << std::endl;
      imu_last_launch = bufferMag->LaunchedCount;
  }
}
imu_csv.WriteToFile(imu_file);
~~~

`TimeStamp` on the buffer is the simulation time the sample *describes*, which is the instant the
collection window closed. With a nonzero lag that precedes the time at which the data became visible.
