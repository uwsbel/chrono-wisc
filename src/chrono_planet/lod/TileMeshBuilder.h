#ifndef CH_PLANET_TILEMESHBUILDER_H
#define CH_PLANET_TILEMESHBUILDER_H

#include "chrono_planet/ChApiPlanet.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/lod/CoordinateSystems.h"
#include "chrono_planet/lod/TileMetadata.h"

namespace chrono {
namespace planet {

class ChPlanetSurface;

// The tile mesh is public, see ChTileMesh.h.
using Mesh = ChTileMesh;

// Grid dimensions and the shared index buffer per level.
struct CH_PLANET_API MeshTopology {
    static constexpr int kMaxDivisions = 64;      // cells per side cap, == ChLevelOfDetail::kMaxDivisions
    static constexpr int kMaxDistinctLevel = 5;   // divisions() saturates here, deeper levels share its topology

    static int divisions(int level);   // cells per side, capped at kMaxDivisions
    static int gridSide(int level) { return divisions(level) + 1; }
    static double vertexSpacingDeg(int level, double rootDeg);    // degrees between grid vertices
    static size_t vertexCount(int level);                         // grid plus the four skirt rows
    static const std::vector<unsigned int>& indices(int level);   // grid + skirt triangles, built once, thread-safe
};

// The static-stage heights of a tile's two grids (see ChPlanetSurface::GetStaticElevationGrid), kept so a
// rebuild after a dynamic filter changes re-runs only the dynamic filters.
struct TileHeightCache {
    std::vector<double> fine, parent;
};

// Builds one tile's Mesh from the planet surface. Instantiated for Spherical and Cartesian.
template <typename CoordSystem>
class TileMeshBuilder {
public:
    using Boundary = typename CoordSystem::Boundary;

    explicit TileMeshBuilder(const std::shared_ptr<const ChPlanetSurface>& surface)
        : surface_(surface) {}

    // Builds an independent mesh. No per-build state is retained; repeated calls may
    // use different bounds and levels. Allocates mesh IDs and records profiling.
    // With a cache, its empty grids are filled and its filled ones reused.
    [[nodiscard]] Mesh build(const Boundary& bounds, int level, TileHeightCache* cache = nullptr) const;

    // Skirt depth in meters at level. CH_PLANET_NOSKIRT=1 disables skirts.
    static double skirtDepthForLevel(int level);

private:
    struct FineGrid {
        int side = 0;
        std::vector<double> positionsM, slopes;   // packed xyz positions and east/north slopes
        std::vector<float> uv;                    // packed tile-local UV pairs
    };
    struct ParentLattice {
        int side = 0, divisions = 0, fineCellsPerCell = 1;
        std::vector<double> positionsM, slopes;
    };
    struct GeometrySummary {
        util::Vec3 centerM;
        double minElevationM, maxElevationM;
    };

    [[nodiscard]] static FineGrid sampleFineGrid(const Boundary& bounds, int level, int divisions, const ChPlanetSurface& surface, std::vector<double>* cache);
    [[nodiscard]] static std::optional<ParentLattice> sampleParentLattice(const Boundary& bounds, int level, int divisions, const ChPlanetSurface& surface, std::vector<double>* cache);
    [[nodiscard]] static GeometrySummary summarizeGeometry(const std::vector<double>& positionsM, double sphereRadiusM);
    [[nodiscard]] static double boundingRadiusM(const std::vector<double>& positionsM, const util::Vec3& centerM, double skirtDepthM);
    [[nodiscard]] static std::vector<float> packVertices(const FineGrid& fine, const std::optional<ParentLattice>& parent,
                                                         const util::Vec3& centerM, double skirtDepthM);

    // The normal's east/north components divided by its up component.
    struct Slope2 {
        double east, north;
    };
    struct VertexSample {
        util::Vec3 positionRelativeM;
        Slope2 slope;
        std::array<float, 2> tileUV;
        util::Vec3 parentPositionRelativeM;
        Slope2 parentSlope;
    };

    [[nodiscard]] static std::vector<double> smoothNormals(const std::vector<double>& positionsM, int side);
    [[nodiscard]] static Slope2 slopeAt(const util::Vec3& positionM, const util::Vec3& normal);
    [[nodiscard]] static std::vector<double> slopesFor(const std::vector<double>& positionsM, int side);
    // Bilinear parent samples at a fine-grid vertex; absent parent uses fine samples.
    [[nodiscard]] static util::Vec3 parentPositionAt(const FineGrid& fine, const std::optional<ParentLattice>& parent, int row, int column);
    [[nodiscard]] static Slope2 parentSlopeAt(const FineGrid& fine, const std::optional<ParentLattice>& parent, int row, int column);
    [[nodiscard]] static double parentComponentAt(const ParentLattice& parent, const std::vector<double>& samples, int componentsPerSample,
                                                  int row, int column, int component);
    // Overwrites one vertex in the explicitly supplied packed output buffer.
    static void writeVertex(std::vector<float>& vertices, size_t vertexIndex, const VertexSample& sample);

    // Shared read-only surface; every build owns its intermediate data.
    std::shared_ptr<const ChPlanetSurface> surface_;
};

}  // namespace planet
}  // namespace chrono

#endif   // CH_PLANET_TILEMESHBUILDER_H
