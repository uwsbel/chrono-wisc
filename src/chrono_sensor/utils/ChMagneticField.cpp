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
// Earth magnetic field models for the magnetometer sensor.
//
// The WMM2025 coefficients below are reproduced verbatim from WMM2025.COF, published by NOAA NCEI
// and the British Geological Survey. The model is a work of the United States government and is in
// the public domain. Replacing the model at its 2030.0 expiry means replacing this table and the
// epoch, and nothing else.
//
// =============================================================================

#include <cmath>
#include <iostream>

#include "chrono/utils/ChConstants.h"

#include "chrono_sensor/utils/ChGPSUtils.h"
#include "chrono_sensor/utils/ChMagneticField.h"

namespace chrono {
namespace sensor {

namespace {

/// Maximum spherical harmonic degree of the model.
const int WMM_MAX_DEGREE = 12;

/// Geomagnetic reference radius, in metres. Defined by the model and distinct from the WGS-84
/// semi-major axis, which is used only to place the observer on the ellipsoid.
const double WMM_REFERENCE_RADIUS = 6371200.0;

/// One Gauss coefficient pair and its secular variation, in nT and nT/year.
struct WMMCoefficient {
    int n;
    int m;
    double g;
    double h;
    double g_dot;
    double h_dot;
};

/// WMM2025 main field coefficients, epoch 2025.0, released 2024-11-13.
const WMMCoefficient WMM2025_COEFFICIENTS[] = {
    // degree 1
    { 1,  0,   -29351.8,        0.0,   12.0,    0.0},
    { 1,  1,    -1410.8,     4545.4,    9.7,  -21.5},
    // degree 2
    { 2,  0,    -2556.6,        0.0,  -11.6,    0.0},
    { 2,  1,     2951.1,    -3133.6,   -5.2,  -27.7},
    { 2,  2,     1649.3,     -815.1,   -8.0,  -12.1},
    // degree 3
    { 3,  0,     1361.0,        0.0,   -1.3,    0.0},
    { 3,  1,    -2404.1,      -56.6,   -4.2,    4.0},
    { 3,  2,     1243.8,      237.5,    0.4,   -0.3},
    { 3,  3,      453.6,     -549.5,  -15.6,   -4.1},
    // degree 4
    { 4,  0,      895.0,        0.0,   -1.6,    0.0},
    { 4,  1,      799.5,      278.6,   -2.4,   -1.1},
    { 4,  2,       55.7,     -133.9,   -6.0,    4.1},
    { 4,  3,     -281.1,      212.0,    5.6,    1.6},
    { 4,  4,       12.1,     -375.6,   -7.0,   -4.4},
    // degree 5
    { 5,  0,     -233.2,        0.0,    0.6,    0.0},
    { 5,  1,      368.9,       45.4,    1.4,   -0.5},
    { 5,  2,      187.2,      220.2,    0.0,    2.2},
    { 5,  3,     -138.7,     -122.9,    0.6,    0.4},
    { 5,  4,     -142.0,       43.0,    2.2,    1.7},
    { 5,  5,       20.9,      106.1,    0.9,    1.9},
    // degree 6
    { 6,  0,       64.4,        0.0,   -0.2,    0.0},
    { 6,  1,       63.8,      -18.4,   -0.4,    0.3},
    { 6,  2,       76.9,       16.8,    0.9,   -1.6},
    { 6,  3,     -115.7,       48.8,    1.2,   -0.4},
    { 6,  4,      -40.9,      -59.8,   -0.9,    0.9},
    { 6,  5,       14.9,       10.9,    0.3,    0.7},
    { 6,  6,      -60.7,       72.7,    0.9,    0.9},
    // degree 7
    { 7,  0,       79.5,        0.0,   -0.0,    0.0},
    { 7,  1,      -77.0,      -48.9,   -0.1,    0.6},
    { 7,  2,       -8.8,      -14.4,   -0.1,    0.5},
    { 7,  3,       59.3,       -1.0,    0.5,   -0.8},
    { 7,  4,       15.8,       23.4,   -0.1,    0.0},
    { 7,  5,        2.5,       -7.4,   -0.8,   -1.0},
    { 7,  6,      -11.1,      -25.1,   -0.8,    0.6},
    { 7,  7,       14.2,       -2.3,    0.8,   -0.2},
    // degree 8
    { 8,  0,       23.2,        0.0,   -0.1,    0.0},
    { 8,  1,       10.8,        7.1,    0.2,   -0.2},
    { 8,  2,      -17.5,      -12.6,    0.0,    0.5},
    { 8,  3,        2.0,       11.4,    0.5,   -0.4},
    { 8,  4,      -21.7,       -9.7,   -0.1,    0.4},
    { 8,  5,       16.9,       12.7,    0.3,   -0.5},
    { 8,  6,       15.0,        0.7,    0.2,   -0.6},
    { 8,  7,      -16.8,       -5.2,   -0.0,    0.3},
    { 8,  8,        0.9,        3.9,    0.2,    0.2},
    // degree 9
    { 9,  0,        4.6,        0.0,   -0.0,    0.0},
    { 9,  1,        7.8,      -24.8,   -0.1,   -0.3},
    { 9,  2,        3.0,       12.2,    0.1,    0.3},
    { 9,  3,       -0.2,        8.3,    0.3,   -0.3},
    { 9,  4,       -2.5,       -3.3,   -0.3,    0.3},
    { 9,  5,      -13.1,       -5.2,    0.0,    0.2},
    { 9,  6,        2.4,        7.2,    0.3,   -0.1},
    { 9,  7,        8.6,       -0.6,   -0.1,   -0.2},
    { 9,  8,       -8.7,        0.8,    0.1,    0.4},
    { 9,  9,      -12.9,       10.0,   -0.1,    0.1},
    // degree 10
    {10,  0,       -1.3,        0.0,    0.1,    0.0},
    {10,  1,       -6.4,        3.3,    0.0,    0.0},
    {10,  2,        0.2,        0.0,    0.1,   -0.0},
    {10,  3,        2.0,        2.4,    0.1,   -0.2},
    {10,  4,       -1.0,        5.3,   -0.0,    0.1},
    {10,  5,       -0.6,       -9.1,   -0.3,   -0.1},
    {10,  6,       -0.9,        0.4,    0.0,    0.1},
    {10,  7,        1.5,       -4.2,   -0.1,    0.0},
    {10,  8,        0.9,       -3.8,   -0.1,   -0.1},
    {10,  9,       -2.7,        0.9,   -0.0,    0.2},
    {10, 10,       -3.9,       -9.1,   -0.0,   -0.0},
    // degree 11
    {11,  0,        2.9,        0.0,    0.0,    0.0},
    {11,  1,       -1.5,        0.0,   -0.0,   -0.0},
    {11,  2,       -2.5,        2.9,    0.0,    0.1},
    {11,  3,        2.4,       -0.6,    0.0,   -0.0},
    {11,  4,       -0.6,        0.2,    0.0,    0.1},
    {11,  5,       -0.1,        0.5,   -0.1,   -0.0},
    {11,  6,       -0.6,       -0.3,    0.0,   -0.0},
    {11,  7,       -0.1,       -1.2,   -0.0,    0.1},
    {11,  8,        1.1,       -1.7,   -0.1,   -0.0},
    {11,  9,       -1.0,       -2.9,   -0.1,    0.0},
    {11, 10,       -0.2,       -1.8,   -0.1,    0.0},
    {11, 11,        2.6,       -2.3,   -0.1,    0.0},
    // degree 12
    {12,  0,       -2.0,        0.0,    0.0,    0.0},
    {12,  1,       -0.2,       -1.3,    0.0,   -0.0},
    {12,  2,        0.3,        0.7,   -0.0,    0.0},
    {12,  3,        1.2,        1.0,   -0.0,   -0.1},
    {12,  4,       -1.3,       -1.4,   -0.0,    0.1},
    {12,  5,        0.6,       -0.0,   -0.0,   -0.0},
    {12,  6,        0.6,        0.6,    0.1,   -0.0},
    {12,  7,        0.5,       -0.1,   -0.0,   -0.0},
    {12,  8,       -0.1,        0.8,    0.0,    0.0},
    {12,  9,       -0.4,        0.1,    0.0,   -0.0},
    {12, 10,       -0.2,       -1.0,   -0.1,   -0.0},
    {12, 11,       -1.3,        0.1,   -0.0,    0.0},
    {12, 12,       -0.7,        0.2,   -0.1,   -0.1},
};

/// Schmidt semi-normalized associated Legendre functions and the two derived quantities the field
/// expansion needs, evaluated at one geocentric colatitude.
///
/// The expansion divides the longitudinal term by sin(colatitude), which is singular at the poles.
/// `p_over_sin` avoids that by carrying P / sin(theta) through its own recursion: every term with
/// m >= 1 contains a factor sin^m(theta), so the ratio is finite everywhere, and only m >= 1 terms
/// have a nonzero longitudinal derivative.
struct LegendreTable {
    double p[WMM_MAX_DEGREE + 1][WMM_MAX_DEGREE + 1];           ///< P(n, m)
    double dp[WMM_MAX_DEGREE + 1][WMM_MAX_DEGREE + 1];          ///< dP(n, m) / d(theta)
    double p_over_sin[WMM_MAX_DEGREE + 1][WMM_MAX_DEGREE + 1];  ///< P(n, m) / sin(theta), m >= 1

    LegendreTable(double sin_theta, double cos_theta) {
        for (int n = 0; n <= WMM_MAX_DEGREE; n++)
            for (int m = 0; m <= WMM_MAX_DEGREE; m++)
                p[n][m] = dp[n][m] = p_over_sin[n][m] = 0;

        p[0][0] = 1;

        for (int n = 1; n <= WMM_MAX_DEGREE; n++) {
            // Sectoral term. The Schmidt normalization factor is 1 rather than sqrt(1 / 2) at n = 1,
            // where the (2 - kronecker_delta) factor of the normalization changes.
            const double c = (n == 1) ? 1.0 : std::sqrt((2.0 * n - 1) / (2.0 * n));
            p[n][n] = c * sin_theta * p[n - 1][n - 1];
            dp[n][n] = c * (sin_theta * dp[n - 1][n - 1] + cos_theta * p[n - 1][n - 1]);
            p_over_sin[n][n] = c * p[n - 1][n - 1];

            for (int m = 0; m < n; m++) {
                const double denom = std::sqrt((double)(n * n - m * m));
                const double a1 = (2.0 * n - 1) / denom;

                p[n][m] = a1 * cos_theta * p[n - 1][m];
                dp[n][m] = a1 * (cos_theta * dp[n - 1][m] - sin_theta * p[n - 1][m]);
                if (m >= 1)
                    p_over_sin[n][m] = a1 * cos_theta * p_over_sin[n - 1][m];

                if (n >= 2) {
                    const double a2 = std::sqrt((double)((n - 1) * (n - 1) - m * m)) / denom;
                    p[n][m] -= a2 * p[n - 2][m];
                    dp[n][m] -= a2 * dp[n - 2][m];
                    if (m >= 1)
                        p_over_sin[n][m] -= a2 * p_over_sin[n - 2][m];
                }
            }
        }
    }
};

/// Report a time outside the model validity window once per process. Extrapolating the secular
/// variation stays usable for a year or so past the window and degrades from there, which is worth
/// saying once but not on every sample.
void WarnOnceIfOutsideValidity(double decimal_year) {
    static bool warned = false;
    if (warned || (decimal_year >= WMM2025_EPOCH && decimal_year <= WMM2025_VALID_UNTIL))
        return;
    warned = true;
    std::cerr << "WARNING: magnetic field requested at decimal year " << decimal_year
              << ", outside the WMM2025 validity window [" << WMM2025_EPOCH << ", " << WMM2025_VALID_UNTIL
              << "]. The secular variation is being extrapolated and the field will drift from reality.\n";
}

}  // namespace

ChVector3d WMM2025Field(double lat_deg, double lon_deg, double alt_m, double decimal_year) {
    WarnOnceIfOutsideValidity(decimal_year);

    const double lat = lat_deg * CH_DEG_TO_RAD;
    const double lon = lon_deg * CH_DEG_TO_RAD;
    const double dt = decimal_year - WMM2025_EPOCH;

    // Geodetic to geocentric spherical. The model is expanded about the Earth centre, while the
    // observer is positioned on the WGS-84 ellipsoid, so the two latitudes differ by up to 0.19 deg.
    const ChVector3d ecef = Geodetic2ECEF(lat, lon, alt_m);
    const double r = ecef.Length();
    const double geocentric_lat = std::asin(ecef.z() / r);

    const double sin_theta = std::cos(geocentric_lat);  // theta is colatitude
    const double cos_theta = std::sin(geocentric_lat);
    const LegendreTable legendre(sin_theta, cos_theta);

    // Powers of (a / r) and the sines and cosines of m * longitude, both built once by recursion
    // rather than recomputed inside the double sum.
    double ratio_pow[WMM_MAX_DEGREE + 3];
    ratio_pow[0] = 1;
    for (int i = 1; i <= WMM_MAX_DEGREE + 2; i++)
        ratio_pow[i] = ratio_pow[i - 1] * (WMM_REFERENCE_RADIUS / r);

    double cos_mlon[WMM_MAX_DEGREE + 1], sin_mlon[WMM_MAX_DEGREE + 1];
    for (int m = 0; m <= WMM_MAX_DEGREE; m++) {
        cos_mlon[m] = std::cos(m * lon);
        sin_mlon[m] = std::sin(m * lon);
    }

    // Geocentric north, east and down components, in nT.
    double north = 0, east = 0, down = 0;
    for (const auto& c : WMM2025_COEFFICIENTS) {
        const double g = c.g + dt * c.g_dot;
        const double h = c.h + dt * c.h_dot;
        const double attenuation = ratio_pow[c.n + 2];
        const double in_phase = g * cos_mlon[c.m] + h * sin_mlon[c.m];

        north += attenuation * in_phase * legendre.dp[c.n][c.m];
        east += attenuation * c.m * (g * sin_mlon[c.m] - h * cos_mlon[c.m]) * legendre.p_over_sin[c.n][c.m];
        down -= attenuation * (c.n + 1) * in_phase * legendre.p[c.n][c.m];
    }

    // Rotate from the geocentric horizon to the geodetic one. East is common to both.
    const double psi = geocentric_lat - lat;
    const double geodetic_north = north * std::cos(psi) - down * std::sin(psi);
    const double geodetic_down = north * std::sin(psi) + down * std::cos(psi);

    // nT to Tesla, and north-east-down to the simulation ENU frame.
    return ChVector3d(east * 1e-9, geodetic_north * 1e-9, -geodetic_down * 1e-9);
}

}  // namespace sensor
}  // namespace chrono
