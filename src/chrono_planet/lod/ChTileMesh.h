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
// Terrain tile meshes as the quadtree hands them to a rendering back end.
//
// =============================================================================

#ifndef CH_TILE_MESH_H
#define CH_TILE_MESH_H

#include <cstdint>
#include <memory>
#include <vector>

#include "chrono_planet/ChApiPlanet.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// One tile's renderable geometry: a vertex grid plus skirts, relative to the tile center.
/// Centers use planet-centered meters for Spherical, site meters for Cartesian. Triangles come from GetIndices(level),
/// counter-clockwise seen from outside (up for the grid, outward for the skirts).
struct CH_PLANET_API ChTileMesh {
    /// Vertex layout: position(3) slope(2) uv(2) parent position(3) parent slope(2). Positions are
    /// relative to (centerX, centerY, centerZ); slopes are the normal's east and north components over
    /// its up component.
    static constexpr int kFloatsPerVertex = 12;
    /// Triangle indices (grid plus skirts) shared by every tile of a level. Thread-safe.
    static const std::vector<unsigned int>& GetIndices(int level);

    static std::uint64_t nextId();
    std::uint64_t id = nextId();  ///< process-unique, in build order; a rebuilt tile gets a new id
    std::vector<float> vertexData;            ///< vertex count * kFloatsPerVertex, center-relative
    int level = 0;                            ///< quadtree level (zoom)
    double uvMin[2] = {0, 0}, uvSize[2] = {1, 1};              ///< tile extent in global UV
    double centerX = 0, centerY = 0, centerZ = 0, radius = 0;  ///< bounding sphere, skirts included (m)
    double maxElevation = 0;  ///< highest height: above reference sphere (Spherical) or site origin (Cartesian), m
    double minElevation = 0;  ///< lowest height in the same convention, m
    bool fromSplit = false;   ///< built when its parent split (not a root tile, nor a rebuild after a change)
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
