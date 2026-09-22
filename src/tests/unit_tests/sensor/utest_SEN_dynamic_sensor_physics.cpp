// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Analytic ground truth for the non-rendering sensors: accelerometer, gyroscope, magnetometer, GPS,
// tachometer and encoder.
//
// Every case here compares a noise-free reading against a value derived in closed form, rather than
// against a recorded output or against "a buffer arrived". Frame errors are what these catch: an
// accelerometer that rotates gravity the wrong way still produces plausible-looking data at every
// timestep and only disagrees with the truth once the body is tilted.
//
// No render backend is required.
//
// =============================================================================

#include <cmath>
#include <functional>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChConstants.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/sensors/ChTachometerSensor.h"
#include "chrono_sensor/utils/ChGPSUtils.h"
#include "chrono_sensor/utils/ChMagneticField.h"

using namespace chrono;
using namespace chrono::sensor;

namespace {

const double GRAVITY = 9.81;
const double STEP = 1e-3;

/// Madison, WI as the sensor module orders it: longitude, latitude, altitude.
const ChVector3d MADISON(-89.400, 43.070, 260.0);

/// Step the system and the sensor manager until the sensor has produced at least one sample beyond
/// `previous_launches`, or the time limit is reached.
bool RunUntilNewSample(ChSystem& sys,
                       ChSensorManager& manager,
                       unsigned int previous_launches,
                       const std::function<unsigned int()>& launches,
                       double limit = 1.0) {
    const double end = sys.GetChTime() + limit;
    while (sys.GetChTime() < end) {
        sys.DoStepDynamics(STEP);
        manager.Update();
        if (launches() > previous_launches)
            return true;
    }
    return false;
}

/// A fixed body at a prescribed pose, with a sensor manager ready to update.
struct FixedBodyRig {
    ChSystemNSC sys;
    std::shared_ptr<ChBody> body;
    std::shared_ptr<ChSensorManager> manager;

    FixedBodyRig(const ChVector3d& pos, const ChQuaternion<>& rot) {
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, -GRAVITY));
        body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
        body->SetPos(pos);
        body->SetRot(rot);
        body->SetFixed(true);
        sys.Add(body);
        manager = chrono_types::make_shared<ChSensorManager>(&sys);
    }
};

}  // namespace

// -----------------------------------------------------------------------------
// Accelerometer
// -----------------------------------------------------------------------------

// A stationary accelerometer reads the reaction to gravity, resolved in its own frame. This is the
// case that distinguishes a correct implementation from one that rotates gravity local-to-parent
// instead of parent-to-local: both agree only when the body is unrotated.
TEST(ChSensorDynamicPhysics, accelerometer_static_orientations) {
    // Orientations chosen to break every wrong-way rotation and every axis swap independently.
    const ChQuaternion<> orientations[] = {
        QUNIT,
        QuatFromAngleX(CH_PI_2),
        QuatFromAngleX(-CH_PI_2),
        QuatFromAngleY(CH_PI_2),
        QuatFromAngleZ(CH_PI_2),
        QuatFromAngleX(0.3) * QuatFromAngleY(-0.7) * QuatFromAngleZ(1.1),
    };
    const ChFrame<double> offsets[] = {
        ChFrame<double>(),
        ChFrame<double>(ChVector3d(0.3, -0.2, 0.1), QUNIT),
        ChFrame<double>(ChVector3d(0.1, 0.4, -0.3), QuatFromAngleX(CH_PI_2)),
        ChFrame<double>(ChVector3d(0, 0, 0), QuatFromAngleY(0.9) * QuatFromAngleZ(-0.4)),
    };

    for (const auto& body_rot : orientations) {
        for (const auto& offset : offsets) {
            FixedBodyRig rig(ChVector3d(0, 0, 5), body_rot);

            auto acc = chrono_types::make_shared<ChAccelerometerSensor>(
                rig.body, 100.f, offset, chrono_types::make_shared<ChNoiseNone>());
            acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
            rig.manager->AddSensor(acc);

            ASSERT_TRUE(RunUntilNewSample(rig.sys, *rig.manager, 0, [&] { return acc->GetNumLaunches(); }));

            auto buffer = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
            ASSERT_TRUE(buffer && buffer->Buffer);

            // Specific force of a body at rest: the support reaction, which is -g expressed in the
            // sensor frame.
            const ChQuaternion<> sensor_rot = body_rot * offset.GetRot();
            const ChVector3d expected = sensor_rot.RotateBack(ChVector3d(0, 0, GRAVITY));

            EXPECT_NEAR(buffer->Buffer[0].X, expected.x(), 1e-9);
            EXPECT_NEAR(buffer->Buffer[0].Y, expected.y(), 1e-9);
            EXPECT_NEAR(buffer->Buffer[0].Z, expected.z(), 1e-9);
            // The magnitude is rotation invariant, so a wrong rotation that happens to preserve it
            // still fails the component checks above.
            EXPECT_NEAR(ChVector3d(buffer->Buffer[0].X, buffer->Buffer[0].Y, buffer->Buffer[0].Z).Length(), GRAVITY,
                        1e-9);
        }
    }
}

// An accelerometer offset from the axis of a spinning body reads the centripetal term, which exists
// only because the lever arm is carried through PointAccelerationLocalToParent.
TEST(ChSensorDynamicPhysics, accelerometer_centripetal) {
    const double omega = 4.0;
    const double radius = 0.75;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));  // isolate the rotational term

    // A sphere so the inertia is isotropic and a free body keeps a constant angular velocity.
    auto body = chrono_types::make_shared<ChBodyEasySphere>(0.5, 1000, false, false);
    body->SetPos(ChVector3d(0, 0, 0));
    body->SetAngVelLocal(ChVector3d(0, 0, omega));
    sys.Add(body);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(
        body, 100.f, ChFrame<double>(ChVector3d(radius, 0, 0), QUNIT), chrono_types::make_shared<ChNoiseNone>());
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
    manager->AddSensor(acc);

    ASSERT_TRUE(RunUntilNewSample(sys, *manager, 0, [&] { return acc->GetNumLaunches(); }));

    auto buffer = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
    ASSERT_TRUE(buffer && buffer->Buffer);

    // The support force on the sensor points toward the axis, so the reading is negative along the
    // outward radial axis.
    EXPECT_NEAR(buffer->Buffer[0].X, -omega * omega * radius, 1e-4);
    EXPECT_NEAR(buffer->Buffer[0].Y, 0.0, 1e-4);
    EXPECT_NEAR(buffer->Buffer[0].Z, 0.0, 1e-4);
}

// -----------------------------------------------------------------------------
// Gyroscope
// -----------------------------------------------------------------------------

// A gyroscope mounted rotated relative to its parent reports about its own axes. Ignoring the offset
// rotation puts the whole rate on the body's axis instead.
TEST(ChSensorDynamicPhysics, gyroscope_offset_rotation) {
    const double omega = 2.5;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    auto body = chrono_types::make_shared<ChBodyEasySphere>(0.5, 1000, false, false);
    body->SetAngVelLocal(ChVector3d(0, 0, omega));
    sys.Add(body);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);

    // Rotating the sensor +90 degrees about X carries the body's +Z onto the sensor's +Y.
    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(
        body, 100.f, ChFrame<double>(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2)),
        chrono_types::make_shared<ChNoiseNone>());
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
    manager->AddSensor(gyro);

    ASSERT_TRUE(RunUntilNewSample(sys, *manager, 0, [&] { return gyro->GetNumLaunches(); }));

    auto buffer = gyro->GetMostRecentBuffer<UserGyroBufferPtr>();
    ASSERT_TRUE(buffer && buffer->Buffer);

    EXPECT_NEAR(buffer->Buffer[0].Roll, 0.0, 1e-6);
    EXPECT_NEAR(buffer->Buffer[0].Pitch, omega, 1e-6);
    EXPECT_NEAR(buffer->Buffer[0].Yaw, 0.0, 1e-6);
}

// -----------------------------------------------------------------------------
// Magnetometer
// -----------------------------------------------------------------------------

// With a supplied local field the reading is that field expressed in the sensor frame, so an
// unrotated sensor returns it unchanged and any rotation preserves its magnitude.
TEST(ChSensorDynamicPhysics, magnetometer_local_field_rotation) {
    const ChVector3d field(-8.0e-7, 1.77e-5, -5.15e-5);  // ENU Tesla, roughly Madison

    const ChQuaternion<> orientations[] = {QUNIT, QuatFromAngleX(CH_PI_2), QuatFromAngleZ(0.8),
                                           QuatFromAngleX(0.3) * QuatFromAngleY(-1.2)};

    for (const auto& body_rot : orientations) {
        FixedBodyRig rig(ChVector3d(0, 0, 0), body_rot);

        auto mag = chrono_types::make_shared<ChMagnetometerSensor>(
            rig.body, 100.f, ChFrame<double>(), chrono_types::make_shared<ChNoiseNone>(), MADISON);
        mag->SetLocalField(field);
        mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
        rig.manager->AddSensor(mag);

        ASSERT_TRUE(RunUntilNewSample(rig.sys, *rig.manager, 0, [&] { return mag->GetNumLaunches(); }));

        auto buffer = mag->GetMostRecentBuffer<UserMagnetBufferPtr>();
        ASSERT_TRUE(buffer && buffer->Buffer);

        const ChVector3d expected = body_rot.RotateBack(field);
        EXPECT_NEAR(buffer->Buffer[0].X, expected.x(), 1e-15);
        EXPECT_NEAR(buffer->Buffer[0].Y, expected.y(), 1e-15);
        EXPECT_NEAR(buffer->Buffer[0].Z, expected.z(), 1e-15);
        EXPECT_NEAR(ChVector3d(buffer->Buffer[0].X, buffer->Buffer[0].Y, buffer->Buffer[0].Z).Length(),
                    field.Length(), 1e-15);
    }
}

// WMM2025 against values published for the model. Declination near Madison is a few degrees WEST;
// a centered dipole puts it about 14 degrees east, so this fails loudly for a dipole approximation.
TEST(ChSensorDynamicPhysics, magnetometer_wmm_matches_published_field) {
    // Test values published with WMM2025.COF by NOAA NCEI, as
    // {latitude, longitude, height above the ellipsoid in km, north, east, down in nT}.
    struct TestValue {
        double lat_deg, lon_deg, alt_km, north_nT, east_nT, down_nT;
    };
    const TestValue published[] = {
        {89, -121, 28, -255.388723, -1482.460628, 56194.288771},
        {80, -96, 48, 1875.982280, -1079.269389, 55623.044051},
        {82, 87, 54, 1324.336929, 1883.428620, 56740.772059},
        {43, 93, 65, 24299.852822, 210.517066, 50037.923998},
        {0, 21, 18, 29274.811882, 659.800118, -14316.722540},
        {-33, 109, 51, 21737.778822, -2090.274098, -52710.003920},
    };

    for (const auto& v : published) {
        const ChVector3d enu = WMM2025Field(v.lat_deg, v.lon_deg, v.alt_km * 1000.0, WMM2025_EPOCH);
        // The buffer frame is ENU in Tesla; the published values are north-east-down in nT.
        EXPECT_NEAR(enu.y() * 1e9, v.north_nT, 0.1) << "north at " << v.lat_deg << ", " << v.lon_deg;
        EXPECT_NEAR(enu.x() * 1e9, v.east_nT, 0.1) << "east at " << v.lat_deg << ", " << v.lon_deg;
        EXPECT_NEAR(-enu.z() * 1e9, v.down_nT, 0.1) << "down at " << v.lat_deg << ", " << v.lon_deg;
    }

    // Madison: the field points north and steeply down, with a small westward declination. A
    // centered dipole puts the declination about 14 degrees EAST, so it cannot pass this.

    const ChVector3d madison = WMM2025Field(MADISON.y(), MADISON.x(), MADISON.z(), WMM2025_EPOCH);
    EXPECT_GT(madison.y(), 0.0);   // north component positive
    EXPECT_LT(madison.z(), 0.0);   // field points into the ground in the northern hemisphere
    const double madison_decl = std::atan2(madison.x(), madison.y()) * CH_RAD_TO_DEG;
    EXPECT_NEAR(madison_decl, -2.5, 2.0);
    const double madison_incl =
        std::atan2(-madison.z(), std::hypot(madison.x(), madison.y())) * CH_RAD_TO_DEG;
    EXPECT_NEAR(madison_incl, 70.5, 2.0);
    EXPECT_NEAR(madison.Length(), 5.3e-5, 5e-6);
}

// -----------------------------------------------------------------------------
// GPS
// -----------------------------------------------------------------------------

// A kilometre of travel must move latitude and longitude by the WGS-84 radii of curvature. The
// spherical mapping this replaced is off by about 0.9 m per km north and 2.7 m per km east.
TEST(ChSensorDynamicPhysics, gps_wgs84_scale) {
    const double lat = MADISON.y() * CH_DEG_TO_RAD;
    const double sin_lat = std::sin(lat);
    const double e2 = WGS84_FLATTENING * (2 - WGS84_FLATTENING);

    const double prime_vertical = WGS84_SEMI_MAJOR_AXIS / std::sqrt(1 - e2 * sin_lat * sin_lat);
    const double meridional = WGS84_SEMI_MAJOR_AXIS * (1 - e2) / std::pow(1 - e2 * sin_lat * sin_lat, 1.5);

    ChVector3d north(0, 1000, 0);
    Cartesian2GPS(north, MADISON);
    // Tolerance covers the second-order term the first-order radius formula omits over a kilometre.
    EXPECT_NEAR(north.y() - MADISON.y(), 1000.0 / meridional * CH_RAD_TO_DEG, 1e-6);

    ChVector3d east(1000, 0, 0);
    Cartesian2GPS(east, MADISON);
    EXPECT_NEAR(east.x() - MADISON.x(), 1000.0 / (prime_vertical * std::cos(lat)) * CH_RAD_TO_DEG, 1e-6);

    // A sphere of mean radius would misplace the northward kilometre by most of a metre.
    const double spherical_error = (1000.0 / EARTH_RADIUS - 1000.0 / meridional) * meridional;
    EXPECT_GT(std::abs(spherical_error), 0.5);
}

TEST(ChSensorDynamicPhysics, gps_round_trip) {
    const ChVector3d coords[] = {{0, 0, 0}, {-100, -100, -100}, {100, 100, 100}, {5000, -7000, 300}};
    const ChVector3d refs[] = {{0, 0, 0}, {-75, 70, 500}, {20, -60, -200}, MADISON};

    for (const auto& c0 : coords) {
        for (const auto& ref : refs) {
            ChVector3d c = c0;
            Cartesian2GPS(c, ref);
            GPS2Cartesian(c, ref);
            EXPECT_NEAR(c.x(), c0.x(), 1e-6);
            EXPECT_NEAR(c.y(), c0.y(), 1e-6);
            EXPECT_NEAR(c.z(), c0.z(), 1e-6);
        }
    }
}

// The antenna position must include the lever arm, and the buffer timestamp must be the time the
// fix describes rather than a stale keyframe time.
TEST(ChSensorDynamicPhysics, gps_lever_arm_and_timestamp) {
    const ChVector3d body_pos(120.0, -80.0, 3.0);
    const ChVector3d offset(2.0, 0.0, 1.5);
    const ChQuaternion<> body_rot = QuatFromAngleZ(CH_PI_2);

    FixedBodyRig rig(body_pos, body_rot);

    auto gps = chrono_types::make_shared<ChGPSSensor>(rig.body, 10.f, ChFrame<double>(offset, QUNIT), MADISON,
                                                      chrono_types::make_shared<ChNoiseNone>());
    gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    rig.manager->AddSensor(gps);

    ASSERT_TRUE(RunUntilNewSample(rig.sys, *rig.manager, 0, [&] { return gps->GetNumLaunches(); }));

    auto buffer = gps->GetMostRecentBuffer<UserGPSBufferPtr>();
    ASSERT_TRUE(buffer && buffer->Buffer);

    ChVector3d expected = body_pos + body_rot.Rotate(offset);
    Cartesian2GPS(expected, MADISON);

    EXPECT_NEAR(buffer->Buffer[0].Longitude, expected.x(), 1e-12);
    EXPECT_NEAR(buffer->Buffer[0].Latitude, expected.y(), 1e-12);
    EXPECT_NEAR(buffer->Buffer[0].Altitude, expected.z(), 1e-6);

    // Both the buffer timestamp and the per-sample time must name the instant of the fix. The
    // timestamp used to carry the previous keyframe's time, which with the default collection
    // window of zero meant it was always zero.
    EXPECT_GT(buffer->TimeStamp, 0.0f);
    EXPECT_NEAR(buffer->Buffer[0].Time, buffer->TimeStamp, 1e-6);
    EXPECT_TRUE(buffer->Buffer[0].Valid);
}

// -----------------------------------------------------------------------------
// Mounting frame
// -----------------------------------------------------------------------------

// A ChBodyAuxRef resolves the offset pose against its reference frame, the same frame render
// sensors mount on. Resolving against the centroidal frame instead displaces the sensor by the
// centre-of-mass offset, which on a vehicle chassis is tens of centimetres.
TEST(ChSensorDynamicPhysics, mounting_frame_is_the_body_reference_frame) {
    const ChVector3d com_offset(0.4, 0.1, -0.2);
    const ChVector3d ref_pos(10.0, 20.0, 1.0);

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -GRAVITY));

    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    body->SetFrameRefToAbs(ChFrame<double>(ref_pos, QUNIT));
    body->SetFrameCOMToRef(ChFrame<double>(com_offset, QUNIT));
    body->SetMass(100);
    body->SetInertiaXX(ChVector3d(1, 1, 1));
    body->SetFixed(true);
    sys.Add(body);

    ASSERT_GT((body->GetFrameCOMToAbs().GetPos() - body->GetFrameRefToAbs().GetPos()).Length(), 0.1)
        << "test body must have distinct COM and REF frames to be meaningful";

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    auto gps = chrono_types::make_shared<ChGPSSensor>(body, 10.f, ChFrame<double>(), MADISON,
                                                      chrono_types::make_shared<ChNoiseNone>());
    gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    manager->AddSensor(gps);

    ASSERT_TRUE(RunUntilNewSample(sys, *manager, 0, [&] { return gps->GetNumLaunches(); }));

    auto buffer = gps->GetMostRecentBuffer<UserGPSBufferPtr>();
    ASSERT_TRUE(buffer && buffer->Buffer);

    ChVector3d expected = ref_pos;
    Cartesian2GPS(expected, MADISON);
    EXPECT_NEAR(buffer->Buffer[0].Longitude, expected.x(), 1e-12);
    EXPECT_NEAR(buffer->Buffer[0].Latitude, expected.y(), 1e-12);
}

// -----------------------------------------------------------------------------
// Lag
// -----------------------------------------------------------------------------

// SetLag stored a value that ChDynamicsManager never read, so dynamic sensor data appeared with no
// latency at all. Data must now become visible one lag after the collection window closes.
TEST(ChSensorDynamicPhysics, lag_delays_data_for_dynamic_sensors) {
    const float lag = 0.05f;
    const float rate = 10.f;

    FixedBodyRig rig(ChVector3d(0, 0, 0), QUNIT);

    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(rig.body, rate, ChFrame<double>(),
                                                               chrono_types::make_shared<ChNoiseNone>());
    acc->SetLag(lag);
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
    rig.manager->AddSensor(acc);

    double first_sample_at = -1;
    float first_sample_stamp = -1;
    while (rig.sys.GetChTime() < 0.5) {
        rig.sys.DoStepDynamics(STEP);
        rig.manager->Update();
        if (acc->GetNumLaunches() > 0 && first_sample_at < 0) {
            first_sample_at = rig.sys.GetChTime();
            first_sample_stamp = acc->GetMostRecentBuffer<UserAccelBufferPtr>()->TimeStamp;
        }
    }

    ASSERT_GT(first_sample_at, 0) << "accelerometer produced no data at all";
    // The first window closes on the first step, so the first sample must surface one lag later.
    EXPECT_NEAR(first_sample_at, lag + STEP, 2 * STEP);

    // The sample still describes the instant the window closed, not the instant it became visible.
    EXPECT_LT(first_sample_stamp, first_sample_at - 0.5 * lag);

    // With no lag the same sensor delivers its first sample immediately, which is what makes the
    // delay above attributable to the lag rather than to warm-up.
    FixedBodyRig prompt(ChVector3d(0, 0, 0), QUNIT);
    auto prompt_acc = chrono_types::make_shared<ChAccelerometerSensor>(prompt.body, rate, ChFrame<double>(),
                                                                      chrono_types::make_shared<ChNoiseNone>());
    prompt_acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
    prompt.manager->AddSensor(prompt_acc);
    prompt.sys.DoStepDynamics(STEP);
    prompt.manager->Update();
    EXPECT_GT(prompt_acc->GetNumLaunches(), 0u);
}

// -----------------------------------------------------------------------------
// Tachometer and encoder
// -----------------------------------------------------------------------------

// The tachometer used to re-read the live body rate at apply time and never stamp its buffer, so a
// consumer could not tell one sample from the next.
TEST(ChSensorDynamicPhysics, tachometer_reports_rate_on_its_own_axis) {
    const double omega = 3.0;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    auto body = chrono_types::make_shared<ChBodyEasySphere>(0.5, 1000, false, false);
    body->SetAngVelLocal(ChVector3d(0, 0, omega));
    sys.Add(body);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);

    // A +90 degree rotation about X carries the body's spin axis onto the sensor's +Y.
    auto tach = chrono_types::make_shared<ChTachometerSensor>(
        body, 20.f, ChFrame<double>(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2)), ChRotationAxis::Y);
    tach->PushFilter(chrono_types::make_shared<ChFilterTachometerAccess>());
    manager->AddSensor(tach);

    ASSERT_TRUE(RunUntilNewSample(sys, *manager, 0, [&] { return tach->GetNumLaunches(); }));

    auto buffer = tach->GetMostRecentBuffer<UserTachometerBufferPtr>();
    ASSERT_TRUE(buffer && buffer->Buffer);
    EXPECT_NEAR(buffer->Buffer[0].rpm, omega * CH_RAD_S_TO_RPM, 1e-3);
    EXPECT_GT(buffer->TimeStamp, 0.0f);
    EXPECT_GT(buffer->LaunchedCount, 0u);

    // A second sample must be distinguishable from the first.
    const unsigned int launches = tach->GetNumLaunches();
    ASSERT_TRUE(RunUntilNewSample(sys, *manager, launches, [&] { return tach->GetNumLaunches(); }));
    EXPECT_GT(tach->GetMostRecentBuffer<UserTachometerBufferPtr>()->LaunchedCount, launches);
}

// An encoder accumulates whole counts, so its angle tracks the shaft to within one count and its
// speed is the count rate rather than an instantaneous sample.
TEST(ChSensorDynamicPhysics, encoder_counts_and_quantisation) {
    const double omega = 5.0;
    const unsigned int lines = 1024;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    auto body = chrono_types::make_shared<ChBodyEasySphere>(0.5, 1000, false, false);
    body->SetAngVelLocal(ChVector3d(0, 0, omega));
    sys.Add(body);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    auto encoder = chrono_types::make_shared<ChEncoderSensor>(body, 50.f, ChFrame<double>(), ChRotationAxis::Z, lines);
    encoder->PushFilter(chrono_types::make_shared<ChFilterEncoderAccess>());
    manager->AddSensor(encoder);

    EXPECT_EQ(encoder->GetCountsPerRevolution(), lines * 4);  // quadrature is on by default
    EXPECT_NEAR(encoder->GetResolution(), CH_2PI / (lines * 4), 1e-12);

    auto run_to = [&](double t) {
        while (sys.GetChTime() < t) {
            sys.DoStepDynamics(STEP);
            manager->Update();
        }
        auto buffer = encoder->GetMostRecentBuffer<UserEncoderBufferPtr>();
        EXPECT_TRUE(buffer && buffer->Buffer);
        return buffer;
    };

    auto first = run_to(1.0);
    const double angle_1 = first->Buffer[0].angle;
    const float time_1 = first->TimeStamp;

    auto buffer = run_to(2.0);

    // The angle accumulated between two samples is the shaft rotation over that interval, to within
    // the count the quantiser can be out at each end. The absolute angle is not compared: an
    // encoder counts from where it started, not from the simulation origin.
    EXPECT_NEAR(buffer->Buffer[0].angle - angle_1, omega * (buffer->TimeStamp - time_1),
                2 * encoder->GetResolution());
    EXPECT_EQ(buffer->Buffer[0].counts, (long long)std::llround(buffer->Buffer[0].angle / encoder->GetResolution()));
    EXPECT_EQ(buffer->Buffer[0].direction, 1);

    // The reported speed comes from whole counts over the window, so it is within one count of the
    // true rate rather than exactly equal to it.
    const double count_rpm = encoder->GetResolution() * 50.0 * CH_RAD_S_TO_RPM;
    EXPECT_NEAR(buffer->Buffer[0].rpm, omega * CH_RAD_S_TO_RPM, 2 * count_rpm);
}

// -----------------------------------------------------------------------------
// Noise models
// -----------------------------------------------------------------------------

// Noise models seeded from the wall clock ignored ChSensorManager::SetRandomSeed entirely, so a run
// with a fixed seed was not reproducible.
TEST(ChSensorDynamicPhysics, noise_models_honour_the_fixed_seed) {
    auto run = [](unsigned int seed) {
        ChSensorManager::SetRandomSeed(seed);

        FixedBodyRig rig(ChVector3d(0, 0, 0), QUNIT);
        auto acc = chrono_types::make_shared<ChAccelerometerSensor>(
            rig.body, 100.f, ChFrame<double>(),
            chrono_types::make_shared<ChNoiseIMU>(100.0, ChNoiseIMU::AccelerometerPreset(ChIMUGrade::CONSUMER_MEMS)));
        acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
        rig.manager->AddSensor(acc);

        std::vector<double> samples;
        unsigned int launches = 0;
        while (samples.size() < 20 && rig.sys.GetChTime() < 2.0) {
            rig.sys.DoStepDynamics(STEP);
            rig.manager->Update();
            if (acc->GetNumLaunches() > launches) {
                launches = acc->GetNumLaunches();
                auto buffer = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
                if (buffer && buffer->Buffer)
                    samples.push_back(buffer->Buffer[0].X);
            }
        }
        ChSensorManager::ClearRandomSeed();
        return samples;
    };

    const auto a = run(7);
    const auto b = run(7);
    const auto c = run(8);

    ASSERT_GE(a.size(), 20u);
    ASSERT_EQ(a.size(), b.size());
    for (size_t i = 0; i < a.size(); i++)
        EXPECT_DOUBLE_EQ(a[i], b[i]) << "same seed must reproduce the same noise, sample " << i;

    // A different seed must actually change the stream, or the reproducibility above is vacuous.
    bool differs = false;
    for (size_t i = 0; i < std::min(a.size(), c.size()); i++)
        differs = differs || (a[i] != c[i]);
    EXPECT_TRUE(differs) << "a different seed produced an identical noise sequence";
}

// Two sensors must draw independent noise, which is what the per-sensor stream identity provides.
TEST(ChSensorDynamicPhysics, two_sensors_draw_independent_noise) {
    ChSensorManager::SetRandomSeed(11);

    FixedBodyRig rig(ChVector3d(0, 0, 0), QUNIT);

    auto make = [&] {
        auto acc = chrono_types::make_shared<ChAccelerometerSensor>(
            rig.body, 100.f, ChFrame<double>(),
            chrono_types::make_shared<ChNoiseIMU>(100.0, ChNoiseIMU::AccelerometerPreset(ChIMUGrade::CONSUMER_MEMS)));
        acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
        rig.manager->AddSensor(acc);
        return acc;
    };
    auto first = make();
    auto second = make();

    int compared = 0;
    unsigned int launches = 0;
    while (compared < 10 && rig.sys.GetChTime() < 2.0) {
        rig.sys.DoStepDynamics(STEP);
        rig.manager->Update();
        if (first->GetNumLaunches() > launches) {
            launches = first->GetNumLaunches();
            auto a = first->GetMostRecentBuffer<UserAccelBufferPtr>();
            auto b = second->GetMostRecentBuffer<UserAccelBufferPtr>();
            if (a && a->Buffer && b && b->Buffer) {
                EXPECT_NE(a->Buffer[0].X, b->Buffer[0].X);
                compared++;
            }
        }
    }
    ChSensorManager::ClearRandomSeed();
    EXPECT_GE(compared, 10);
}

// The Gauss-Markov bias of ChNoiseIMU is bounded, unlike the unbounded random walk of
// ChNoiseNormalDrift. Over many correlation times the bias must stay near its configured spread
// rather than wander away.
TEST(ChSensorDynamicPhysics, imu_bias_instability_stays_bounded) {
    ChNoiseIMUParams params;
    params.bias_instability = 0.01;
    params.bias_correlation_time = 1.0;
    ChNoiseIMU model(100.0, params);

    double worst = 0;
    double sum_sq = 0;
    const int samples = 200000;
    for (int i = 0; i < samples; i++) {
        ChVector3d data(0, 0, 0);
        model.AddNoise(data);
        worst = std::max(worst, std::abs(model.GetBias().x()));
        sum_sq += model.GetBias().x() * model.GetBias().x();
    }

    // Steady-state standard deviation must match the configured bias instability.
    EXPECT_NEAR(std::sqrt(sum_sq / samples), params.bias_instability, 0.15 * params.bias_instability);
    // A random walk over 2000 correlation times would be far outside this.
    EXPECT_LT(worst, 8 * params.bias_instability);
}

// A GPS error correlated over minutes is the point of ChNoiseGPS: consecutive epochs must be close
// to each other while still spanning the configured spread over a long run.
TEST(ChSensorDynamicPhysics, gps_noise_is_correlated_between_epochs) {
    ChNoiseGPSParams params;
    params.horizontal_stdev = 2.0;
    params.vertical_stdev = 3.0;
    params.correlation_time = 100.0;
    params.white_stdev = 0.0;  // isolate the correlated term
    ChNoiseGPS model(params);

    std::vector<double> east;
    const double dt = 0.1;
    for (int i = 0; i < 20000; i++) {
        ChVector3d data(0, 0, 0);
        model.AddNoise(data, (float)(i * dt), (float)((i + 1) * dt));
        east.push_back(data.x());
    }

    double sum_sq = 0;
    double step_sq = 0;
    for (size_t i = 1; i < east.size(); i++) {
        sum_sq += east[i] * east[i];
        step_sq += (east[i] - east[i - 1]) * (east[i] - east[i - 1]);
    }
    const double stdev = std::sqrt(sum_sq / (east.size() - 1));
    const double step_stdev = std::sqrt(step_sq / (east.size() - 1));

    EXPECT_NEAR(stdev, params.horizontal_stdev, 0.4 * params.horizontal_stdev);
    // One step of dt over a correlation time of 100 s moves the error by sqrt(2 * dt / tau) of its
    // spread, which is far less than a white sequence of the same standard deviation would.
    EXPECT_LT(step_stdev, 0.2 * stdev);
}
