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
// =============================================================================

#ifndef CHMAGNETICFIELD_H
#define CHMAGNETICFIELD_H

#include "chrono/core/ChVector3.h"
#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_utils
/// @{

/// Source of the Earth magnetic field a magnetometer measures.
enum class ChMagneticFieldModel {
    /// The World Magnetic Model 2025, a degree-12 spherical harmonic expansion of the main field
    /// published by NOAA NCEI and the British Geological Survey. Valid from 2025.0 to 2030.0.
    /// Accurate to roughly 150 nT and 0.5 degrees of declination anywhere on Earth.
    WMM2025,

    /// A constant field vector supplied by the user, in the simulation ENU frame. Appropriate when
    /// the field has been measured at the site, or when a study needs a field that does not vary
    /// with position. This is what most robotics simulators offer.
    LOCAL
};

/// Epoch of the World Magnetic Model 2025 coefficients, in decimal years.
#define WMM2025_EPOCH 2025.0

/// End of the World Magnetic Model 2025 validity period, in decimal years.
#define WMM2025_VALID_UNTIL 2030.0

/// Evaluate the World Magnetic Model 2025 main field.
///
/// The returned vector is in the simulation ENU frame: +X east, +Y north, +Z up. In the northern
/// hemisphere the field points north and into the ground, so the Z component is negative there.
///
/// @param lat_deg Geodetic latitude in degrees, positive north
/// @param lon_deg Longitude in degrees, positive east
/// @param alt_m Height above the WGS-84 ellipsoid in metres
/// @param decimal_year Time at which to evaluate the model, in decimal years. Outside
/// [WMM2025_EPOCH, WMM2025_VALID_UNTIL] the secular variation is extrapolated and a warning is
/// issued once.
/// @return The magnetic flux density in Tesla, in the ENU frame
CH_SENSOR_API ChVector3d WMM2025Field(double lat_deg, double lon_deg, double alt_m, double decimal_year);

/// @} sensor_utils

}  // namespace sensor
}  // namespace chrono

#endif
