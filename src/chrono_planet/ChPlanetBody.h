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
// Description of a spherical planetary body: name, reference radius, surface
// gravity and the geographic coordinate system its DEMs are read in.
//
// =============================================================================

#ifndef CH_PLANET_BODY_H
#define CH_PLANET_BODY_H

#include <string>

#include "chrono/core/ChVector3.h"

#include "chrono_planet/ChApiPlanet.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// A planetary body modeled as a sphere.
/// Every elevation in the module is a height above the reference sphere of radius GetRadius(), and
/// every longitude and latitude is on that sphere. Bodies with a noticeable flattening (Earth, Mars)
/// are approximated by their mean radius.
///
/// The body carries no terrain. Build a ChPlanetSurface on it and add elevation data and relief
/// layers there. Ready-made bodies live under chrono_planet/planets (for example planets/moon).
class CH_PLANET_API ChPlanetBody {
  public:
    /// Construct a body with the given name and reference radius (meters).
    /// Throws std::invalid_argument if the radius is not positive.
    ChPlanetBody(const std::string& name, double radius, double surface_gravity = 0);

    const std::string& GetName() const { return m_name; }  ///< body name
    double GetRadius() const { return m_radius; }            ///< reference sphere radius (m)

    /// Set the surface gravitational acceleration (m/s^2, positive). Default: 0.
    /// The module does not apply it; demos and user code read it to configure a ChSystem.
    void SetSurfaceGravity(double g) { m_gravity = g; }
    double GetSurfaceGravity() const { return m_gravity; }  ///< surface gravity (m/s^2)

    /// Set the geographic coordinate system that projected DEMs are transformed into.
    /// Accepts anything OGRSpatialReference::SetFromUserInput does: WKT, a PROJ string, or an
    /// authority code such as "IAU_2015:30100" where the installed PROJ database provides one.
    /// If a projected system is given, its geographic base is used.
    /// Default (empty): a sphere of GetRadius() named after the body, which suits most planetary DEMs.
    void SetGeographicSRS(const std::string& srs) { m_srs = srs; }
    const std::string& GetGeographicSRS() const { return m_srs; }  ///< user SRS, empty for the default sphere

    /// Ground distance of one degree along a meridian (m).
    double GetMetersPerDegree() const;

    /// Planet-centered Cartesian position of a point at the given longitude, latitude (degrees)
    /// and height above the reference sphere (m). +z is the north pole, +x the prime meridian.
    ChVector3d ToCartesian(double lon_deg, double lat_deg, double height) const;

    /// Longitude, latitude (degrees) and height above the reference sphere (m) of a
    /// planet-centered Cartesian position.
    void ToGeographic(const ChVector3d& pos, double& lon_deg, double& lat_deg, double& height) const;

  private:
    std::string m_name;
    double m_radius;
    double m_gravity;
    std::string m_srs;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
