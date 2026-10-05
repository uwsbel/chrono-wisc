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
// The quadtree terrain as an ordinary Chrono visual shape, for renderers that
// read visual models from the system (Chrono::Sensor).
//
// =============================================================================

#ifndef CH_PLANET_VISUAL_MESH_H
#define CH_PLANET_VISUAL_MESH_H

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/filters/ChDeformationFilter.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Quadtree terrain tiles around a viewpoint, as triangle-mesh visual shapes on a fixed body, one per tile.
/// Each tile's mesh is built once and never modified; when the tile set changes, the body's shapes are
/// replaced, which is how renderers that stage the system's visual models (Chrono::Sensor with Vulkan or
/// Metal ray tracing) notice the change, and tiles still in the set keep their mesh, so such a renderer
/// can reuse what it staged for them. A renderer that only reads meshes once when its scene is built
/// (the OptiX backend) will not see changes. Give it its own quadtree: its viewpoint drives that
/// quadtree's level of detail.
class CH_PLANET_API ChPlanetVisualMesh {
  public:
    /// How the tiles are placed in the system's frame.
    enum class Shape {
        /// In the site frame's equirectangular coordinates, exactly where physics on the same site has the
        /// ground. The frame is flat, so this only holds up for the few kilometers around the site it is meant for.
        SITE,
        /// At their true positions on the planet, in east-north-up axes at the site origin. Near the origin this
        /// matches SITE to within d^2 / 2R at a distance d (a millimeter at 60 m on the Moon, 0.3 m at 1 km), and
        /// farther out it keeps the planet round, for views from altitude or orbit.
        SPHERE
    };

    /// Add the terrain body to `sys`. `site` maps planet coordinates into the system's frame.
    ChPlanetVisualMesh(ChSystem* sys, std::shared_ptr<ChPlanetQuadtree> world, const ChSiteFrame& site);
    ~ChPlanetVisualMesh();

    /// Material of the terrain shape (default: gray, Lambertian-like).
    void SetMaterial(std::shared_ptr<ChVisualMaterial> material);
    std::shared_ptr<ChVisualMaterial> GetMaterial() const { return m_material; }

    /// Draw ground that a deformation filter has lowered by more than `min_depth` (m) with another material,
    /// such as a darker one for regolith compacted by wheels. The lowering is sampled at the tile's vertices and
    /// faces are split along the `min_depth` contour, so the boundary follows the rut outline, not the tile grid.
    /// Applies to tiles built after the call, so set it before the first update. Pass a null filter to turn it off.
    void SetCompaction(std::shared_ptr<ChDeformationFilter> filter, std::shared_ptr<ChVisualMaterial> material, double min_depth = 0.01);

    /// Ground lowered past a depth (m), and the material it is drawn with there.
    struct CompactionLevel {
        double min_depth;
        std::shared_ptr<ChVisualMaterial> material;
    };

    /// As SetCompaction, with several levels: ground lowered past each level's depth takes its material, so ruts can
    /// darken by steps with depth rather than all at once. Faces are split along every level's contour.
    void SetCompaction(std::shared_ptr<ChDeformationFilter> filter, std::vector<CompactionLevel> levels);

    /// Leave the terrain out of a rectangle of site x/y, cutting the faces that cross its edges, where other ground
    /// takes over, such as a work site modeled on its own. The tiles are rebuilt on the next update.
    void AddHole(const ChSiteRegion& hole);
    /// Remove every hole. The tiles are rebuilt on the next update.
    void ClearHoles();
    const std::vector<ChSiteRegion>& GetHoles() const { return m_holes; }

    /// Set how the tiles are placed (default: SITE). Applies to tiles built after the call, so set it before
    /// the first update.
    void SetShape(Shape shape) { m_shape = shape; }
    Shape GetShape() const { return m_shape; }

    /// Ease tiles in over this much simulation time (s, default: 0.5) when the quadtree splits their parent:
    /// a new tile starts in its parent's shape and blends to its own, so the terrain does not pop as the camera
    /// comes closer (geomorphing). Tiles rebuilt because the surface changed, such as for soil deformation, show
    /// at once. 0 turns it off.
    void SetMorphTime(double time) { m_morph_time = time; }
    double GetMorphTime() const { return m_morph_time; }

    /// Leave out tiles farther than this from the viewpoint (m, default: 5000). This bounds the triangles a
    /// renderer holds; set it to the farthest range a sensor sees.
    void SetMaxDistance(double distance) { m_max_distance = distance; }
    double GetMaxDistance() const { return m_max_distance; }

    /// Advance the level of detail to `time` for a viewpoint in the system's frame (site coordinates, or the
    /// east-north-up axes at the site origin for SPHERE), and replace the tile shapes if the tile set changed.
    /// Returns true when they were replaced.
    bool Update(const ChVector3d& viewpoint, double time);

    /// Append one tile to a mesh in site coordinates: vertices with normals and UVs, and its triangles
    /// (grid and skirts) counter-clockwise seen from outside. `morph` blends the vertices from the parent tile's
    /// shape (0) to the tile's own (1).
    static void AppendTile(const ChTileMesh& tile, const ChSiteFrame& site, ChTriangleMeshConnected& mesh, double morph = 1.0);

    std::shared_ptr<ChBody> GetBody() const { return m_body; }
    std::shared_ptr<ChPlanetQuadtree> GetWorld() const { return m_world; }
    size_t GetNumTiles() const { return m_num_tiles; }
    size_t GetNumTriangles() const { return m_num_triangles; }
    unsigned int GetNumRebuilds() const { return m_num_rebuilds; }

  private:
    void Rebuild(const ChVector3d& viewpoint_planet, double time);

    /// Split the faces of a tile's mesh along the compaction-depth contour and mark those whose ground has been
    /// lowered past it with material 1. Returns false, leaving the mesh untouched, if no ground is.
    bool MarkCompaction(const ChTileMesh& tile, ChTriangleMeshConnected& mesh) const;

    /// Move the vertices and normals of a tile's mesh, built by AppendTile with the same morph, to their SPHERE
    /// placement.
    void PlaceOnSphere(const ChTileMesh& tile, ChTriangleMeshConnected& mesh, double morph) const;

    /// Planet-centered position of a point in the system's frame.
    ChVector3d ToPlanet(const ChVector3d& p) const;

    std::shared_ptr<ChPlanetQuadtree> m_world;
    ChSiteFrame m_site;
    std::shared_ptr<ChBody> m_body;
    std::shared_ptr<ChVisualMaterial> m_material;
    Shape m_shape = Shape::SITE;
    ChVector3d m_origin;                        ///< site origin, planet-centered (m)
    ChVector3d m_east, m_north, m_up;           ///< east-north-up axes at the site origin
    std::shared_ptr<ChDeformationFilter> m_compaction;              ///< source of the ground lowering, if any
    std::vector<CompactionLevel> m_compaction_levels;              ///< materials of lowered ground, by depth
    std::vector<ChSiteRegion> m_holes;                              ///< rectangles of site x/y left out
    std::unordered_map<std::uint64_t, std::shared_ptr<ChVisualShapeTriangleMesh>> m_tiles;  ///< by ChTileMesh::id
    double m_max_distance;
    double m_morph_time = 0.5;
    std::unordered_map<std::uint64_t, double> m_morphs;  ///< start time of each tile easing in, by ChTileMesh::id
    std::uint64_t m_last_id = 0;                          ///< newest tile id drawn so far
    unsigned long long m_mesh_version;
    size_t m_num_tiles;
    size_t m_num_triangles;
    unsigned int m_num_rebuilds;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
