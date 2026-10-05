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
// Level-of-detail quadtree of terrain tiles streamed around a moving viewpoint.
//
// =============================================================================

#ifndef CH_PLANET_QUADTREE_STREAM_H
#define CH_PLANET_QUADTREE_STREAM_H

#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "chrono/core/ChVector2.h"
#include "chrono/core/ChVector3.h"

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/lod/ChTileMesh.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Terrain tiles for rendering: a ring of root tiles around the camera, each refined by its own
/// quadtree. Child meshes are built synchronously when a tile splits.
/// Tiles are built from a ChPlanetSurface, so they show exactly the terrain physics sees on the same
/// surface. Root tile size comes from the surface (ChPlanetSurface::SetRootTileSize). The tree at a
/// given simulation time depends only on the camera path, with splits complete before Update returns.
///
/// Filters that change at run time (for example a ChDeformationFilter fed by a soil model) report what
/// they changed; every Update polls the surface's filter chain and rebuilds the resident tiles fine
/// enough to show the change. Invalidate does the same for changes made outside the chain.
class CH_PLANET_API ChPlanetQuadtree {
  public:
    /// Build over a surface, keeping a ring of `view_range_tiles` root tiles each side of the camera.
    ChPlanetQuadtree(std::shared_ptr<const ChPlanetSurface> surface, int view_range_tiles);
    ~ChPlanetQuadtree();

    ChPlanetQuadtree(const ChPlanetQuadtree&) = delete;
    ChPlanetQuadtree& operator=(const ChPlanetQuadtree&) = delete;

    /// Simulation time of one LOD tick (s).
    static constexpr double kTickS = 1.0 / 60.0;

    /// Camera position (planet-centered, m) at simulation time t.
    using PoseFn = std::function<ChVector3d(double t)>;

    /// Advance the LOD to `sim_time`, sampling the camera path at each tick along the way.
    void Update(const PoseFn& pose_at, double sim_time);

    /// Advance the LOD to `sim_time` for a camera now at the given planet-centered position (m).
    /// Ticks since the previous call interpolate between the two positions.
    void Update(const ChVector3d& camera, double sim_time);

    /// How close the camera must come to a tile, in widths of that tile, before the tile splits into four
    /// finer ones (default: 1.0). Smaller values keep fewer, coarser tiles; each tile has up to 64 x 64
    /// cells, so at 1.0 a tile's vertices are about 1/64 radian apart as seen from where it splits.
    void SetSplitDistance(double tile_widths);
    double GetSplitDistance() const;

    /// Split every tile above the camera's horizon to at least this zoom (quadtree level below the root tiles),
    /// however far the camera is (default: 0). Root tiles have 2 x 2 cells and each zoom doubles that, up to
    /// 64 x 64, so from far away a minimum zoom keeps the planet from being drawn as a coarse polyhedron: at 3,
    /// with 16 degree root tiles, cells are 1/8 degree.
    void SetMinZoom(int zoom);
    int GetMinZoom() const;

    /// Never split tiles past this zoom (default: 17). With 16 degree root tiles, cells at zoom 17 are about 6 cm
    /// across and each zoom halves them; relief layers finer than that, such as regolith roughness, need a
    /// higher zoom to show up close. Past the zoom a camera gets near enough to split, it has no effect.
    void SetMaxZoom(int zoom);
    int GetMaxZoom() const;

    /// Horizon culling: tiles entirely below the camera's horizon are not refined. The horizon is taken on
    /// a sphere `margin` meters below both the camera and the tile's lowest point, so terrain up to that
    /// deep between them cannot wrongly hide a tile (default: 1000 m). A negative margin turns it off.
    void SetHorizonMargin(double margin);
    double GetHorizonMargin() const;

    /// Last tick stepped. A frame draws the tree this tick ended with.
    long long GetTick() const;

    /// Number of resident root tiles.
    int GetNumRootTiles() const;

    /// Every leaf tile mesh, in ChTileMesh::id order. A tile whose mesh is rebuilt reappears with a new
    /// id. Pointers stay valid until the next Update().
    std::vector<const ChTileMesh*> GetMeshes() const;

    /// Surface height at a point as a mesh at `zoom` carries it, or nullopt outside the resident root tiles.
    std::optional<double> GetElevation(double lon_deg, double lat_deg, int zoom = 0) const;

    /// Rebuild the resident tiles that touch a region and whose vertex spacing is below
    /// region.max_spacing. Returns the number of tiles rebuilt.
    int Invalidate(const ChGeoRegion& region);

    /// Tiles rebuilt so far because their surface changed.
    long long GetNumRebuiltTiles() const;

    /// A number that changes whenever any tile's mesh set or the root ring changes.
    unsigned long long GetMeshSetVersion() const;

    /// The surface the tiles are built from.
    std::shared_ptr<const ChPlanetSurface> GetSurface() const;

    /// Camera position at the last tick (planet-centered, m).
    ChVector3d GetCameraPosition() const;
    /// Camera longitude (x) and latitude (y) at the last tick, in degrees.
    ChVector2d GetCameraLonLat() const;
    /// Camera height above the reference sphere at the last tick (m).
    double GetCameraElevation() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
