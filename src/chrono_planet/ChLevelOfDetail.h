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
// The level-of-detail schedule: what a zoom level means in degrees.
//
// =============================================================================

#ifndef CH_LEVEL_OF_DETAIL_H
#define CH_LEVEL_OF_DETAIL_H

#include "chrono_planet/ChApiPlanet.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// The level-of-detail schedule shared by the terrain model, physics and rendering.
/// Zoom 0 is a root tile of GetRootTileSize() degrees; each zoom halves the tile. A tile at zoom z is a
/// grid of GetDivisions(z) cells per side, so its vertices are GetVertexSpacing(z) degrees apart. The
/// surface filters its relief to that spacing, which is what makes a physics query at zoom z see the
/// same terrain as the renderer's mesh at zoom z.
class CH_PLANET_API ChLevelOfDetail {
  public:
    static constexpr int kMaxDivisions = 64;  ///< cells per tile side, cap

    /// Schedule with root tiles of the given side (degrees, in (0, 180]). Throws std::invalid_argument.
    explicit ChLevelOfDetail(double root_tile_deg = 16.0);

    double GetRootTileSize() const { return m_root_tile_deg; }  ///< root tile side (degrees)

    /// Cells per tile side at a zoom: 2^(zoom+1), capped at kMaxDivisions.
    static int GetDivisions(int zoom);

    /// Vertex spacing (degrees) of a tile mesh at a zoom.
    double GetVertexSpacing(int zoom) const { return GetVertexSpacing(zoom, m_root_tile_deg); }

    /// Vertex spacing (degrees) at a zoom for root tiles of the given side.
    static double GetVertexSpacing(int zoom, double root_tile_deg);

  private:
    double m_root_tile_deg;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
