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
// GPS utils
//
// =============================================================================

#include <cmath>

#include "chrono/utils/ChConstants.h"

#include "chrono_sensor/utils/ChGPSUtils.h"

namespace chrono {
namespace sensor {

namespace {

/// First eccentricity squared of the WGS-84 ellipsoid.
const double WGS84_E2 = WGS84_FLATTENING * (2 - WGS84_FLATTENING);

/// Second eccentricity squared, used by Bowring's latitude formula.
const double WGS84_EP2 = WGS84_E2 / ((1 - WGS84_FLATTENING) * (1 - WGS84_FLATTENING));

/// Semi-minor axis of the WGS-84 ellipsoid, in metres.
const double WGS84_SEMI_MINOR_AXIS = WGS84_SEMI_MAJOR_AXIS * (1 - WGS84_FLATTENING);

/// Radius of curvature in the prime vertical at a given latitude, in metres.
double PrimeVerticalRadius(double sin_lat) {
    return WGS84_SEMI_MAJOR_AXIS / std::sqrt(1 - WGS84_E2 * sin_lat * sin_lat);
}

/// Wrap a longitude in degrees to (-180, 180].
double WrapLongitude(double lon_deg) {
    while (lon_deg <= -180.0)
        lon_deg += 360.0;
    while (lon_deg > 180.0)
        lon_deg -= 360.0;
    return lon_deg;
}

}  // namespace

ChVector3d Geodetic2ECEF(double lat_rad, double lon_rad, double alt) {
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double N = PrimeVerticalRadius(sin_lat);

    return ChVector3d((N + alt) * cos_lat * std::cos(lon_rad),   //
                      (N + alt) * cos_lat * std::sin(lon_rad),   //
                      (N * (1 - WGS84_E2) + alt) * sin_lat);
}

void ECEF2Geodetic(const ChVector3d& ecef, double& lat_rad, double& lon_rad, double& alt) {
    lon_rad = std::atan2(ecef.y(), ecef.x());

    const double p = std::hypot(ecef.x(), ecef.y());
    if (p < 1e-9) {
        // On the polar axis, where longitude is undefined and the latitude formula divides by p.
        lat_rad = (ecef.z() >= 0 ? CH_PI_2 : -CH_PI_2);
        alt = std::abs(ecef.z()) - WGS84_SEMI_MINOR_AXIS;
        return;
    }

    // Bowring's parametric-latitude estimate, then fixed-point refinement. The estimate is already
    // accurate to a fraction of a millimetre at terrestrial altitudes; two refinements take the
    // round trip below the nanometre tolerance the unit test asserts.
    const double theta = std::atan2(ecef.z() * WGS84_SEMI_MAJOR_AXIS, p * WGS84_SEMI_MINOR_AXIS);
    lat_rad = std::atan2(ecef.z() + WGS84_EP2 * WGS84_SEMI_MINOR_AXIS * std::pow(std::sin(theta), 3),
                         p - WGS84_E2 * WGS84_SEMI_MAJOR_AXIS * std::pow(std::cos(theta), 3));

    for (int i = 0; i < 3; i++) {
        const double N = PrimeVerticalRadius(std::sin(lat_rad));
        alt = p / std::cos(lat_rad) - N;
        lat_rad = std::atan2(ecef.z(), p * (1 - WGS84_E2 * N / (N + alt)));
    }

    alt = p / std::cos(lat_rad) - PrimeVerticalRadius(std::sin(lat_rad));
}

ChVector3d ENU2ECEFDelta(const ChVector3d& enu, double lat_rad, double lon_rad) {
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double sin_lon = std::sin(lon_rad);
    const double cos_lon = std::cos(lon_rad);

    return ChVector3d(-sin_lon * enu.x() - sin_lat * cos_lon * enu.y() + cos_lat * cos_lon * enu.z(),
                      cos_lon * enu.x() - sin_lat * sin_lon * enu.y() + cos_lat * sin_lon * enu.z(),
                      cos_lat * enu.y() + sin_lat * enu.z());
}

ChVector3d ECEF2ENUDelta(const ChVector3d& ecef, double lat_rad, double lon_rad) {
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double sin_lon = std::sin(lon_rad);
    const double cos_lon = std::cos(lon_rad);

    return ChVector3d(-sin_lon * ecef.x() + cos_lon * ecef.y(),
                      -sin_lat * cos_lon * ecef.x() - sin_lat * sin_lon * ecef.y() + cos_lat * ecef.z(),
                      cos_lat * cos_lon * ecef.x() + cos_lat * sin_lon * ecef.y() + sin_lat * ecef.z());
}

void Cartesian2GPS(ChVector3d& coords, const ChVector3d& ref) {
    const double ref_lon = ref.x() * CH_DEG_TO_RAD;
    const double ref_lat = ref.y() * CH_DEG_TO_RAD;

    const ChVector3d ecef =
        Geodetic2ECEF(ref_lat, ref_lon, ref.z()) + ENU2ECEFDelta(coords, ref_lat, ref_lon);

    double lat, lon, alt;
    ECEF2Geodetic(ecef, lat, lon, alt);

    coords = ChVector3d(WrapLongitude(lon * CH_RAD_TO_DEG), lat * CH_RAD_TO_DEG, alt);
}

void GPS2Cartesian(ChVector3d& coords, const ChVector3d& ref) {
    const double ref_lon = ref.x() * CH_DEG_TO_RAD;
    const double ref_lat = ref.y() * CH_DEG_TO_RAD;

    const ChVector3d delta = Geodetic2ECEF(coords.y() * CH_DEG_TO_RAD, coords.x() * CH_DEG_TO_RAD, coords.z()) -
                             Geodetic2ECEF(ref_lat, ref_lon, ref.z());

    coords = ECEF2ENUDelta(delta, ref_lat, ref_lon);
}

// The simulation frame is defined as local ENU, so these four are conversions between a frame and
// itself. They are kept as named no-ops so that call sites which want to state the frame explicitly
// can, and so that existing code continues to compile.

void Cartesian2ENU(ChVector3d& coords, const ChVector3d& ref) {}

void ENU2Cartesian(ChVector3d& coords, const ChVector3d& ref) {}

void GPS2ENU(ChVector3d& coords, const ChVector3d& ref) {
    GPS2Cartesian(coords, ref);
}

void ENU2GPS(ChVector3d& coords, const ChVector3d& ref) {
    Cartesian2GPS(coords, ref);
}

}  // namespace sensor
}  // namespace chrono
