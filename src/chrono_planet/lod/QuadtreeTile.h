#ifndef CH_PLANET_QUADTREE_TILE_H
#define CH_PLANET_QUADTREE_TILE_H

#include "chrono_planet/ChApiPlanet.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <unordered_map>
#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/lod/Quadtree.h"
#include "chrono_planet/lod/TileMeshBuilder.h"
#include "chrono_planet/lod/TileMetadata.h"

namespace chrono {
namespace planet {

class ChPlanetSurface;

// How eagerly the tree refines, shared by every root tile of a quadtree.
struct LodParams {
    double splitDistanceM0;   // split distance of a root tile (m); level L splits at this / 2^L
    double horizonMarginM;    // assumed terrain depth below the camera and the tile for horizon tests (m), < 0: off
    double sphereRadiusM;     // body radius (m)
    int minLevel = 0;         // tiles above the horizon split to at least this level
    int maxLevel = 17;        // tiles never split past this level
};

// Camera distance below which a tile at level splits.
inline double lodSplitDistance(int level, const LodParams& p) {
    if (level < 0) {
        return p.sphereRadiusM;
    }
    return p.splitDistanceM0 / static_cast<double>(1 << std::min(level, 30));
}

// True when the whole tile is certainly below the camera's horizon: past the tangent points on a sphere
// horizonMarginM below both the camera and the tile's lowest point.
inline bool belowHorizon(double cameraRadiusM, double tileMinRadiusM, double tileMaxRadiusM, double distanceM,
                         const LodParams& p) {
    if (p.horizonMarginM < 0) {
        return false;
    }
    const double r0 = std::min(cameraRadiusM, tileMinRadiusM) - p.horizonMarginM;
    if (r0 <= 0) {
        return false;
    }
    const double reach = std::sqrt(std::max(0.0, cameraRadiusM * cameraRadiusM - r0 * r0)) +
                         std::sqrt(std::max(0.0, tileMaxRadiusM * tileMaxRadiusM - r0 * r0));
    return distanceM > reach;
}

// One root tile with its LOD tree, each node's mesh, synchronous child mesh builds.
template <typename CoordSystem>
class QuadtreeTile {
public:
    using Boundary = typename CoordSystem::Boundary;
    using Position = typename CoordSystem::Position;
    using Node = QuadTree<TileMetadata, CoordSystem>;

    // Builds the root mesh synchronously from the surface.
    QuadtreeTile(Boundary b, std::shared_ptr<const ChPlanetSurface> surface);
    ~QuadtreeTile();
    QuadtreeTile(const QuadtreeTile&) = delete;
    QuadtreeTile& operator=(const QuadtreeTile&) = delete;

    // One LOD step: split (building children synchronously) or merge at the camera position in meters.
    void updateLOD(const util::Vec3& cameraM, const LodParams& lod);

    // Every node's mesh, divided nodes included.
    const std::unordered_map<const Node*, Mesh>& getMeshes() const { return meshes_; }

    // Changes whenever a mesh is added, regenerated or removed.
    unsigned meshSetVersion() const { return meshSetVersion_; }

    // Rebuilds every node touching the rectangle with spacing below the threshold.
    // Bounds and spacing use degrees for Spherical and site meters for Cartesian. Returns the number of nodes rebuilt.
    int invalidate(double minLon, double minLat, double maxLon, double maxLat, double maxSpacingDeg);

    // Ground height at pos as a mesh at zoomLevel would carry it.
    [[nodiscard]] double getElevation(Position pos, int zoomLevel) const;

private:
    void updateLODRec(Node* node, const util::Vec3& cameraM, const LodParams& lod);
    // Camera distance to the node's elevation shell, and whether the node is below the camera's horizon.
    double distanceTo(const Node* node, const util::Vec3& cameraM, bool& hidden, const LodParams& lod) const;
    int invalidateRec(Node* node, double minLon, double minLat, double maxLon, double maxLat, double maxSpacingDeg);
    void buildMesh(Node* node);                                   // ensures the node has a mesh and metadata
    void rebuildMesh(Node* node);                                 // rebuilds after a change, reusing the node's height cache
    void storeMesh(Node* node, Mesh m);                           // installs a mesh and its metadata
    Mesh buildTimed(const Boundary& bounds, int level) const;     // generateMesh, timed for the bench
    Mesh generateMesh(const Boundary& bounds, int level) const;

    std::shared_ptr<const ChPlanetSurface> surface_;
    double radiusM_;   // body radius, for the LOD distances
    std::unordered_map<const Node*, Mesh> meshes_;
    // Static-stage heights of nodes rebuilt for changes, so later changes re-run only the dynamic filters.
    std::unordered_map<const Node*, TileHeightCache> caches_;
    unsigned meshSetVersion_ = 0;
    // The root node. Declared last so its onDestroy callbacks still find the maps above alive.
    std::unique_ptr<Node> tree_;
};

}  // namespace planet
}  // namespace chrono

#endif   // CH_PLANET_QUADTREE_TILE_H
