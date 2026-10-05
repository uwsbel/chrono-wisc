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

#include <stdexcept>

#include "chrono_planet/ChPlanetBody.h"
#include "chrono_planet/core/SphereMath.h"

namespace chrono {
namespace planet {

ChPlanetBody::ChPlanetBody(const std::string& name, double radius, double surface_gravity)
    : m_name(name), m_radius(radius), m_gravity(surface_gravity) {
    if (!(radius > 0))
        throw std::invalid_argument("ChPlanetBody: radius must be positive for body '" + name + "'");
}

double ChPlanetBody::GetMetersPerDegree() const {
    return util::metresPerDegLat(m_radius);
}

ChVector3d ChPlanetBody::ToCartesian(double lon_deg, double lat_deg, double height) const {
    const util::Vec3 p = util::pointOnSphere(lon_deg, lat_deg, m_radius + height);
    return ChVector3d(p.x, p.y, p.z);
}

void ChPlanetBody::ToGeographic(const ChVector3d& pos, double& lon_deg, double& lat_deg, double& height) const {
    const util::LonLat ll = util::lonLatOf(pos.x(), pos.y(), pos.z());
    lon_deg = ll.lon;
    lat_deg = ll.lat;
    height = util::elevationOf(pos.x(), pos.y(), pos.z(), m_radius);
}

}  // namespace planet
}  // namespace chrono
