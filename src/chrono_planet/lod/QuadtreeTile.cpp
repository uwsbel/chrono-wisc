#include "chrono_planet/lod/QuadtreeTile.h"

#include <cmath>
#include <type_traits>
#include "chrono_planet/lod/CartesianCoordinates.h"

#include "chrono_planet/core/BenchProfiler.h"
#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/lod/SphericalCoordinates.h"

namespace chrono {
namespace planet {

namespace {

// Deepest level the LOD builds.

// Children merge only once the camera has retreated this far past the split distance.
constexpr double kMergeHysteresis = 1.5;

// Tiles wholly past this latitude are never refined: toward the pole they only grow thinner.
constexpr double kPolarCapLatDeg = 89.9;

// True for a tile that lies wholly inside the excluded polar cap. A tile that only reaches into it still splits, so
// the ground outside the cap is refined: a root tile that touches the pole would otherwise hold everything within its
// own height of the pole, 16 degrees, at its coarse level.
template <typename Boundary>
bool inPolarCap(const Boundary& boundary) {
    if constexpr (std::is_same_v<Boundary, Cartesian::Boundary>)
        return false;
    else
        return std::abs(boundary.centerLatDeg) - boundary.halfHeightDeg >= kPolarCapLatDeg;
}

}   // namespace

template <typename CoordSystem>
QuadtreeTile<CoordSystem>::QuadtreeTile(Boundary b, std::shared_ptr<const ChPlanetSurface> surface)
    : surface_(std::move(surface)), radiusM_(surface_->GetBody().GetRadius()), tree_(std::make_unique<Node>(b)) {
    tree_->onInit = [this](Node* node) { buildMesh(node); };
    tree_->onDestroy = [this](Node* node) {
        caches_.erase(node);
        if (meshes_.erase(node)) {
            meshSetVersion_++;
        }
    };
    tree_->onSplit = [this](Node* parent) {
        parent->getType()->morphFromParent = true;
        for (Node* c : parent->children()) {
            if (c) {
                c->getType()->morphFromParent = true;
            }
        }
        meshSetVersion_++;
    };
    tree_->onMerge = [this](Node*) { meshSetVersion_++; };
    buildMesh(tree_.get());
}

template <typename CoordSystem>
QuadtreeTile<CoordSystem>::~QuadtreeTile() = default;

template <typename CoordSystem>
void QuadtreeTile<CoordSystem>::updateLOD(const util::Vec3& cameraM, const LodParams& lod) {
    updateLODRec(tree_.get(), cameraM, lod);
}

template <typename CoordSystem>
double QuadtreeTile<CoordSystem>::distanceTo(const Node* node, const util::Vec3& cameraM, bool& hidden, const LodParams& lod) const {
    const TileMetadata& meta = *node->getType();
    const double distance = CoordinateTraits<CoordSystem>::distanceToBounds(node->getBoundary(), cameraM, meta.minElevation, meta.maxElevation, radiusM_);
    if constexpr (std::is_same_v<CoordSystem, Cartesian>)
        hidden = false;
    else
        hidden = belowHorizon(util::length(cameraM), radiusM_ + meta.minElevation, radiusM_ + meta.maxElevation, distance, lod);
    return distance;
}

template <typename CoordSystem>
void QuadtreeTile<CoordSystem>::updateLODRec(Node* node, const util::Vec3& cameraM, const LodParams& lod) {
    const Boundary boundary = node->getBoundary();
    if (inPolarCap(boundary)) {
        return;
    }
    bool hidden = false;
    const double distance = distanceTo(node, cameraM, hidden, lod);
    const int level = node->getLevel();
    const double splitDist = lodSplitDistance(level, lod);
    const double mergeDist = splitDist * kMergeHysteresis;
    const bool canSplit = level < lod.maxLevel && !hidden;
    const bool forced = level < lod.minLevel;  // with canSplit, not below the horizon

    if (canSplit && (distance < splitDist || forced)) {
        if (!node->isDivided()) {
            BENCH_SCOPE("split");
            node->subdivide();   // onInit builds each child mesh before the split becomes visible
            bench::bump(bench::ctr().splitsDone);
        }
        for (Node* c : node->children()) {
            if (c) {
                updateLODRec(c, cameraM, lod);
            }
        }
    } else if ((distance > mergeDist && !forced) || hidden) {
        if (node->isDivided()) {
            BENCH_SCOPE("merge");
            node->getType()->morphFromParent = false;   // back at full detail, no split ease
            node->merge();
            bench::bump(bench::ctr().merges);
        }
    }
}

template <typename CoordSystem>
void QuadtreeTile<CoordSystem>::buildMesh(Node* node) {
    Mesh m = buildTimed(node->getBoundary(), node->getLevel());
    m.fromSplit = node->getLevel() > 0;  // only a split creates nodes below the root
    storeMesh(node, std::move(m));
}

template <typename CoordSystem>
void QuadtreeTile<CoordSystem>::rebuildMesh(Node* node) {
    BENCH_SCOPE("rebuild");
    storeMesh(node, TileMeshBuilder<CoordSystem>(surface_).build(node->getBoundary(), node->getLevel(), &caches_[node]));
}

template <typename CoordSystem>
void QuadtreeTile<CoordSystem>::storeMesh(Node* node, Mesh m) {
    TileMetadata& meta = *node->getType();
    meta.maxElevation = m.maxElevation;
    meta.minElevation = m.minElevation;
    meshes_[node] = std::move(m);
    meshSetVersion_++;
}

template <typename CoordSystem>
Mesh QuadtreeTile<CoordSystem>::buildTimed(const Boundary& bounds, int level) const {
    BENCH_SCOPE("build_sync");
    const auto t0 = bench::Clock::now();
    Mesh m = generateMesh(bounds, level);
    bench::bump(bench::ctr().syncBuildNs,
                std::chrono::duration_cast<std::chrono::nanoseconds>(bench::Clock::now() - t0).count());
    return m;
}

template <typename CoordSystem>
Mesh QuadtreeTile<CoordSystem>::generateMesh(const Boundary& bounds, int level) const {
    return TileMeshBuilder<CoordSystem>(surface_).build(bounds, level);
}

template <typename CoordSystem>
int QuadtreeTile<CoordSystem>::invalidate(double minLon, double minLat, double maxLon, double maxLat, double maxSpacingDeg) {
    return invalidateRec(tree_.get(), minLon, minLat, maxLon, maxLat, maxSpacingDeg);
}

template <typename CoordSystem>
int QuadtreeTile<CoordSystem>::invalidateRec(Node* node, double minLon, double minLat, double maxLon, double maxLat,
                                              double maxSpacingDeg) {
    const Boundary b = node->getBoundary();
    double halfWidth;
    if constexpr (std::is_same_v<CoordSystem, Cartesian>) {
        if (b.centerY + b.halfHeightM < minLat || b.centerY - b.halfHeightM > maxLat ||
            b.centerX + b.halfWidthM < minLon || b.centerX - b.halfWidthM > maxLon)
            return 0;
        halfWidth = b.halfWidthM;
    } else {
        if (b.centerLatDeg + b.halfHeightDeg < minLat || b.centerLatDeg - b.halfHeightDeg > maxLat) {
            return 0;
        }
        bool overlaps = false;
        for (double shift : {0.0, 360.0, -360.0}) {
            const double lon0 = b.centerLonDeg - b.halfWidthDeg + shift, lon1 = b.centerLonDeg + b.halfWidthDeg + shift;
            overlaps = overlaps || !(lon1 < minLon || lon0 > maxLon);
        }
        if (!overlaps) {
            return 0;
        }
        halfWidth = b.halfWidthDeg;
    }
    int rebuilt = 0;
    const int level = node->getLevel();
    if (2.0 * halfWidth / MeshTopology::divisions(level) < maxSpacingDeg) {
        rebuildMesh(node);
        ++rebuilt;
    }
    if (node->isDivided()) {
        for (Node* c : node->children()) {
            if (c) {
                rebuilt += invalidateRec(c, minLon, minLat, maxLon, maxLat, maxSpacingDeg);
            }
        }
    }
    return rebuilt;
}

template <typename CoordSystem>
double QuadtreeTile<CoordSystem>::getElevation(Position pos, int zoomLevel) const {
    if constexpr (std::is_same_v<CoordSystem, Cartesian>) {
        const auto& site = tree_->getBoundary().site;
        double lon, lat;
        site.ToLonLat(pos.x, pos.y, lon, lat);
        const double width = std::ldexp(2 * tree_->getBoundary().halfWidthM, -zoomLevel);
        const double spacing = width / MeshTopology::divisions(zoomLevel) / util::metresPerDegLon(site.GetRadius(), lat);
        return surface_->GetElevationAtZoom(lon, lat, zoomLevel, spacing) - site.GetOriginElevation();
    } else {
        return surface_->GetElevationAtZoom(pos.lon, pos.lat, zoomLevel);
    }
}

template class QuadtreeTile<Spherical>;
template class QuadtreeTile<Cartesian>;

}  // namespace planet
}  // namespace chrono
