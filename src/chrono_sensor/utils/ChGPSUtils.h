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

#ifndef CHGPSUTILS_H
#define CHGPSUTILS_H

#include "chrono/core/ChVector3.h"
#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_utils
/// @{

/// Mean radius of the Earth, in metres.
///
/// Retained for callers that want a single representative radius. The coordinate conversions below
/// use the WGS-84 ellipsoid instead: a sphere of this radius misplaces a point by roughly 0.9 m per
/// kilometre travelled north and 2.7 m per kilometre travelled east at mid latitudes, which exceeds
/// the accuracy of any real receiver and makes simulated tracks impossible to overlay on real maps.
#define EARTH_RADIUS 6371000.0

/// WGS-84 semi-major axis, in metres.
#define WGS84_SEMI_MAJOR_AXIS 6378137.0

/// WGS-84 flattening.
#define WGS84_FLATTENING (1.0 / 298.257223563)

// -----------------------------------------------------------------------------
// Frame conventions
//
// SIMULATION frame: Cartesian metres with +X east, +Y north, +Z up, with the origin at the
// simulation reference point. This is the same thing as the local ENU frame, which is why the
// Cartesian/ENU conversions below are identities.
//
// GPS frame: a ChVector3d holding (LONGITUDE, LATITUDE, ALTITUDE) -- longitude FIRST, degrees for
// the first two components and metres for the third. This ordering applies to both the `coords` and
// the `ref` arguments of every function here, and to the gps_reference passed to ChGPSSensor and
// ChMagnetometerSensor. It is the opposite of the (lat, lon, alt) ordering used by most GPS
// libraries, by NMEA, and by chrono::synchrono::GPScoord, so it is worth checking at every call:
// a swapped pair is a valid coordinate somewhere else on Earth and produces no diagnostic.
//
//     ChVector3d gps_reference(-89.400, 43.070, 260.0);  // Madison, WI: longitude, latitude, alt
//
// ECEF frame: Earth-centred, Earth-fixed Cartesian metres, +X through (0 deg N, 0 deg E), +Z through
// the north pole.
// -----------------------------------------------------------------------------

/// Utility function for calculating GPS coordinates from Cartesian coordinates given the simulation's reference point
/// @param coords The simulation-frame position in metres; overwritten with (longitude, latitude, altitude)
/// @param ref The simulation's reference location as (longitude, latitude, altitude)
CH_SENSOR_API void Cartesian2GPS(ChVector3d& coords, const ChVector3d& ref);

/// Utility function for calculating Cartesian coordinates from GPS coordinates given the simulation's reference point
/// @param coords The position as (longitude, latitude, altitude); overwritten with simulation-frame metres
/// @param ref The simulation's reference location as (longitude, latitude, altitude)
CH_SENSOR_API void GPS2Cartesian(ChVector3d& coords, const ChVector3d& ref);

/// Utility function for calculating East-North-Up (ENU) coordinates from Cartesian coordinates given the simulation's
/// reference point. The simulation frame is already local ENU, so this leaves the coordinates unchanged; it exists so
/// that code which wants to be explicit about the frame can say so.
/// @param coords The simulation-frame position in metres, unchanged
/// @param ref The simulation's reference location as (longitude, latitude, altitude)
CH_SENSOR_API void Cartesian2ENU(ChVector3d& coords, const ChVector3d& ref);

/// Utility function for calculating Cartesian coordinates from ENU coordinates given the simulation's reference point.
/// The simulation frame is already local ENU, so this leaves the coordinates unchanged.
/// @param coords The ENU position in metres, unchanged
/// @param ref The simulation's reference location as (longitude, latitude, altitude)
CH_SENSOR_API void ENU2Cartesian(ChVector3d& coords, const ChVector3d& ref);

/// Utility function for calculating GPS coordinates from ENU coordinates given the simulation's reference point
/// @param coords The ENU position in metres; overwritten with (longitude, latitude, altitude)
/// @param ref The simulation's reference location as (longitude, latitude, altitude)
CH_SENSOR_API void ENU2GPS(ChVector3d& coords, const ChVector3d& ref);

/// Utility function for calculating ENU coordinates from GPS coordinates given the simulation's reference point
/// @param coords The position as (longitude, latitude, altitude); overwritten with ENU metres
/// @param ref The simulation's reference location as (longitude, latitude, altitude)
CH_SENSOR_API void GPS2ENU(ChVector3d& coords, const ChVector3d& ref);

/// Convert geodetic coordinates to Earth-centred, Earth-fixed Cartesian coordinates on the WGS-84 ellipsoid.
/// @param lat_rad Geodetic latitude in radians
/// @param lon_rad Longitude in radians
/// @param alt Height above the ellipsoid in metres
/// @return The ECEF position in metres
CH_SENSOR_API ChVector3d Geodetic2ECEF(double lat_rad, double lon_rad, double alt);

/// Convert Earth-centred, Earth-fixed Cartesian coordinates to geodetic coordinates on the WGS-84 ellipsoid.
/// @param ecef The ECEF position in metres
/// @param lat_rad Set to the geodetic latitude in radians
/// @param lon_rad Set to the longitude in radians
/// @param alt Set to the height above the ellipsoid in metres
CH_SENSOR_API void ECEF2Geodetic(const ChVector3d& ecef, double& lat_rad, double& lon_rad, double& alt);

/// Rotate a local East-North-Up displacement into an Earth-centred, Earth-fixed displacement.
/// @param enu The ENU displacement in metres
/// @param lat_rad Geodetic latitude of the tangent point in radians
/// @param lon_rad Longitude of the tangent point in radians
/// @return The corresponding ECEF displacement in metres
CH_SENSOR_API ChVector3d ENU2ECEFDelta(const ChVector3d& enu, double lat_rad, double lon_rad);

/// Rotate an Earth-centred, Earth-fixed displacement into a local East-North-Up displacement.
/// @param ecef The ECEF displacement in metres
/// @param lat_rad Geodetic latitude of the tangent point in radians
/// @param lon_rad Longitude of the tangent point in radians
/// @return The corresponding ENU displacement in metres
CH_SENSOR_API ChVector3d ECEF2ENUDelta(const ChVector3d& ecef, double lat_rad, double lon_rad);

/// @} sensor_utils

}  // namespace sensor
}  // namespace chrono
#endif
