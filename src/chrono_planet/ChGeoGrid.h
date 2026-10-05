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
// A square, regular longitude/latitude sampling grid.
//
// =============================================================================

#ifndef CH_GEO_GRID_H
#define CH_GEO_GRID_H

#include "chrono_planet/ChApiPlanet.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Square geographic sampling grid, row-major with rows running north.
/// Sample (i, j) sits at (lon0 + i * step_lon, lat0 + j * step_lat), for i and j in [0, n).
/// Steps must be positive. The member order is part of the interface, so the grid can be
/// brace-initialized and unpacked with structured bindings.
struct CH_PLANET_API ChGeoGrid {
    double lon0;      ///< longitude of sample (0, 0), degrees
    double lat0;      ///< latitude of sample (0, 0), degrees
    double step_lon;  ///< longitude step, degrees
    double step_lat;  ///< latitude step, degrees
    int n;            ///< samples per side
    /// Angular resolution the procedural relief is filtered to, in degrees. Features too small to be
    /// resolved at this spacing fade out. It may differ from the steps, for example to match a coarser
    /// level of detail.
    double spacing;
};

/// A longitude/latitude rectangle (degrees), for example the part of a surface that changed.
/// Longitudes are not wrapped; a region that crosses the dateline is not supported.
struct CH_PLANET_API ChGeoRegion {
    double min_lon = 1e300, min_lat = 1e300, max_lon = -1e300, max_lat = -1e300;
    /// Coarsest relief spacing (degrees) at which the change is visible; tiles with a coarser vertex
    /// spacing are unaffected. Infinite by default (visible at every level of detail).
    double max_spacing = 1e300;

    bool IsEmpty() const { return min_lon > max_lon || min_lat > max_lat; }

    /// Grow to include a point.
    void Include(double lon, double lat) {
        min_lon = lon < min_lon ? lon : min_lon;
        max_lon = lon > max_lon ? lon : max_lon;
        min_lat = lat < min_lat ? lat : min_lat;
        max_lat = lat > max_lat ? lat : max_lat;
    }

    /// Grow to include another region; the result is visible up to the larger of the two spacings.
    void Include(const ChGeoRegion& other) {
        if (other.IsEmpty())
            return;
        const bool was_empty = IsEmpty();
        Include(other.min_lon, other.min_lat);
        Include(other.max_lon, other.max_lat);
        max_spacing = was_empty ? other.max_spacing : (other.max_spacing > max_spacing ? other.max_spacing : max_spacing);
    }
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
