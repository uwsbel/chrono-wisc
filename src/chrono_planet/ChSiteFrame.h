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
// Authors: bgwitt
// =============================================================================
//
// East-north-up tangent frame at a landing site, mapping planet longitude,
// latitude and elevation to local Cartesian meters.
//
// =============================================================================

#ifndef CH_SITE_FRAME_H
#define CH_SITE_FRAME_H

#include <cmath>

#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChConstants.h"

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChPlanetBody.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Local east-north-up frame tangent to a planet's reference sphere at a site origin.
/// Positions are equirectangular meters about the origin, so the frame is only meant for the
/// few kilometers a surface simulation covers; z is elevation above the origin's elevation.
class CH_PLANET_API ChSiteFrame {
  public:
    /// Construct a site frame on a sphere of the given radius (m), at the given origin (degrees, meters).
    ChSiteFrame(double radius, double origin_lon_deg, double origin_lat_deg, double origin_elev_m)
        : m_radius(radius), m_lon0(origin_lon_deg), m_lat0(origin_lat_deg), m_elev0(origin_elev_m) {}

    /// Construct a site frame on the body's reference sphere, at the given origin (degrees, meters).
    /// ChPlanetSurface::MakeSiteFrame builds one with the origin on the surface.
    ChSiteFrame(const ChPlanetBody& body, double origin_lon_deg, double origin_lat_deg, double origin_elev_m)
        : ChSiteFrame(body.GetRadius(), origin_lon_deg, origin_lat_deg, origin_elev_m) {}

    /// Convert a planet position (degrees, meters) to site coordinates.
    ChVector3d ToLocal(double lon_deg, double lat_deg, double elev_m) const {
        const double y = Deg2Rad(lat_deg - m_lat0) * m_radius;
        const double x = Deg2Rad(WrapDelta(lon_deg - m_lon0)) * m_radius * std::cos(Deg2Rad(lat_deg));
        return ChVector3d(x, y, elev_m - m_elev0);
    }

    /// Convert site x/y meters to longitude and latitude in degrees.
    void ToLonLat(double x, double y, double& lon_deg, double& lat_deg) const {
        lat_deg = m_lat0 + Rad2Deg(y / m_radius);
        lon_deg = m_lon0 + Rad2Deg(x / (m_radius * std::cos(Deg2Rad(lat_deg))));
    }

    double GetRadius() const { return m_radius; }           ///< reference sphere radius (m)
    double GetOriginLongitude() const { return m_lon0; }  ///< origin longitude (degrees)
    double GetOriginLatitude() const { return m_lat0; }   ///< origin latitude (degrees)
    double GetOriginElevation() const { return m_elev0; } ///< origin elevation (meters)

  private:
    static double Deg2Rad(double d) { return d * (CH_PI / 180.0); }
    static double Rad2Deg(double r) { return r * (180.0 / CH_PI); }
    static double WrapDelta(double d) {
        while (d > 180.0)
            d -= 360.0;
        while (d < -180.0)
            d += 360.0;
        return d;
    }

    double m_radius;
    double m_lon0;
    double m_lat0;
    double m_elev0;
};

/// A rectangle of site x/y coordinates (m), for example a CRM soil window or a hole cut out of the terrain.
struct CH_PLANET_API ChSiteRegion {
    double min_x = 0, min_y = 0, max_x = 0, max_y = 0;

    ChSiteRegion() = default;
    ChSiteRegion(double min_x_, double min_y_, double max_x_, double max_y_) : min_x(min_x_), min_y(min_y_), max_x(max_x_), max_y(max_y_) {}

    bool IsEmpty() const { return !(min_x < max_x && min_y < max_y); }
    bool Contains(double x, double y) const { return x >= min_x && x <= max_x && y >= min_y && y <= max_y; }
    /// The rectangle moved in by `d` (m) on every side (out for negative d).
    ChSiteRegion Inset(double d) const { return ChSiteRegion(min_x + d, min_y + d, max_x - d, max_y - d); }
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
