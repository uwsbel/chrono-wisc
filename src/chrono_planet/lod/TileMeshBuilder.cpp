#include "chrono_planet/lod/TileMeshBuilder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>

#include "chrono_planet/core/BenchProfiler.h"
#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/core/Parallel.h"
#include "chrono_planet/ChLevelOfDetail.h"
#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/core/SphereMath.h"
#include "chrono_planet/lod/SphericalCoordinates.h"
#include "chrono_planet/lod/CartesianCoordinates.h"

namespace chrono {
namespace planet {

using namespace util;

int MeshTopology::divisions(int level) { return ChLevelOfDetail::GetDivisions(level); }

double MeshTopology::vertexSpacingDeg(int level, double rootDeg) {
    return ChLevelOfDetail::GetVertexSpacing(level, rootDeg);
}

size_t MeshTopology::vertexCount(int level) {
    const size_t n = gridSide(level);
    return n * n + 4 * n;
}

namespace {

// Grid triangles followed by one skirt strip per border, for a grid of n vertices per side.
std::vector<unsigned int> buildIndices(int n) {
    const int divisions = n - 1;
    std::vector<unsigned int> out;
    out.reserve(static_cast<size_t>(divisions) * divisions * 6 + 4 * (n - 1) * 6);
    for (int j = 0; j < divisions; ++j) {
        for (int i = 0; i < divisions; ++i) {
            // j grows north and i east; counter-clockwise seen from above, so the front face is up.
            const unsigned int tl = j * n + i, tr = tl + 1, bl = (j + 1) * n + i, br = bl + 1;
            out.insert(out.end(), {tl, tr, bl, tr, br, bl});
        }
    }
    // Skirts, one strip per border, hanging off the skirt rows after the grid.
    unsigned int base = static_cast<unsigned int>(n) * n;
    // Walls face outward: south and east strips as listed, north and west ones reversed.
    auto strip = [&](bool reversed, auto&& topIndex) {
        for (int k = 0; k + 1 < n; ++k) {
            const unsigned int t0 = topIndex(k), t1 = topIndex(k + 1);
            const unsigned int b0 = base + k, b1 = base + k + 1;
            if (reversed)
                out.insert(out.end(), {t0, t1, b0, t1, b1, b0});
            else
                out.insert(out.end(), {t0, b0, t1, t1, b0, b1});
        }
        base += n;
    };
    strip(false, [n](int k) { return static_cast<unsigned int>(k); });                 // south row
    strip(true, [n](int k) { return static_cast<unsigned int>((n - 1) * n + k); });    // north row
    strip(true, [n](int k) { return static_cast<unsigned int>(k * n); });              // west column
    strip(false, [n](int k) { return static_cast<unsigned int>(k * n + (n - 1)); });   // east column
    return out;
}

}   // namespace

const std::vector<unsigned int>& MeshTopology::indices(int level) {
    // One table per distinct topology, all built on first use so any thread may read them afterwards.
    static const auto tables = [] {
        std::array<std::vector<unsigned int>, kMaxDistinctLevel + 1> t;
        for (int l = 0; l <= kMaxDistinctLevel; ++l) {
            t[l] = buildIndices(gridSide(l));
        }
        return t;
    }();
    return tables[std::clamp(level, 0, kMaxDistinctLevel)];
}

const std::vector<unsigned int>& ChTileMesh::GetIndices(int level) {
    return MeshTopology::indices(level);
}

template <typename C>
Mesh TileMeshBuilder<C>::build(const Boundary& bounds, int level, TileHeightCache* cache) const {
    BENCH_SCOPE("generate");
    const int divisions = MeshTopology::divisions(level);
    const auto fine = sampleFineGrid(bounds, level, divisions, *surface_, cache ? &cache->fine : nullptr);
    const auto parent = sampleParentLattice(bounds, level, divisions, *surface_, cache ? &cache->parent : nullptr);
    const auto geometry = summarizeGeometry(fine.positionsM, surface_->GetBody().GetRadius());
    const double skirtDepthM = skirtDepthForLevel(level);
    Mesh mesh;
    mesh.level = level;
    mesh.centerX = geometry.centerM.x;
    mesh.centerY = geometry.centerM.y;
    mesh.centerZ = geometry.centerM.z;
    mesh.minElevation = geometry.minElevationM;
    mesh.maxElevation = geometry.maxElevationM;
    mesh.radius = boundingRadiusM(fine.positionsM, geometry.centerM, skirtDepthM);
    CoordinateTraits<C>::textureBounds(bounds, mesh.uvMin, mesh.uvSize);
    mesh.vertexData = packVertices(fine, parent, geometry.centerM, skirtDepthM);
    return mesh;
}

template <typename C>
double TileMeshBuilder<C>::skirtDepthForLevel(int level) {
    // Walls stay deeper than the sag a coarser neighbor's T-junctions open.
    static constexpr double depthByLevel[] = {2000.0, 1000.0, 500.0, 250.0, 150.0, 80.0, 70.0, 50.0, 30.0, 20.0};
    constexpr int count = sizeof(depthByLevel) / sizeof(depthByLevel[0]);
    static const bool noSkirt = std::getenv("CH_PLANET_NOSKIRT") != nullptr;
    if (noSkirt) {
        return 0.0;
    }
    return depthByLevel[std::clamp(level, 0, count - 1)];
}

template <typename C>
typename TileMeshBuilder<C>::FineGrid TileMeshBuilder<C>::sampleFineGrid(
    const Boundary& bounds, int level, int divisions, const ChPlanetSurface& surface, std::vector<double>* cache) {
    FineGrid grid;
    grid.side = divisions + 1;
    {
        BENCH_SCOPE("grid");
        grid.positionsM = CoordinateTraits<C>::cartesianGrid(bounds, divisions, surface, level, 0.0, cache);
        grid.uv.resize(static_cast<size_t>(grid.side) * grid.side * 2);
        for (int row = 0; row < grid.side; ++row) {
            for (int column = 0; column < grid.side; ++column) {
                const size_t vertex = static_cast<size_t>(row) * grid.side + column;
                grid.uv[vertex * 2] = static_cast<float>(static_cast<double>(column) / divisions);
                grid.uv[vertex * 2 + 1] = static_cast<float>(static_cast<double>(row) / divisions);
            }
        }
    }
    {
        BENCH_SCOPE("slopes");
        grid.slopes = slopesFor(grid.positionsM, grid.side);
    }
    return grid;
}

// The parent's lattice, one ring of cells wider than this tile, so every fine vertex has a parent cell.
template <typename C>
std::optional<typename TileMeshBuilder<C>::ParentLattice> TileMeshBuilder<C>::sampleParentLattice(
    const Boundary& bounds, int level, int divisions, const ChPlanetSurface& surface, std::vector<double>* cache) {
    BENCH_SCOPE("lattice");
    const int parentDivisions = level > 0 ? MeshTopology::divisions(level - 1) / 2 : 0;   // parent cells across this tile
    if (parentDivisions < 1) {
        return std::nullopt;
    }
    ParentLattice parent;
    parent.divisions = parentDivisions;
    parent.fineCellsPerCell = divisions / parentDivisions;   // fine cells per parent cell
    const Boundary ring = CoordinateTraits<C>::expanded(bounds, 1.0 + 2.0 / parent.divisions);
    parent.side = parent.divisions + 3;   // coarse grid side incl. ring
    parent.positionsM = CoordinateTraits<C>::cartesianGrid(ring, parent.divisions + 2, surface, level - 1, 0.0, cache);
    parent.slopes = slopesFor(parent.positionsM, parent.side);
    return parent;
}

template <typename C>
typename TileMeshBuilder<C>::GeometrySummary TileMeshBuilder<C>::summarizeGeometry(const std::vector<double>& positionsM, double sphereRadiusM) {
    const size_t gridCount = positionsM.size() / 3;
    double cx = 0, cy = 0, cz = 0, maxElev = -1e30, minElev = 1e30;
    for (size_t v = 0; v < gridCount; ++v) {
        cx += positionsM[v * 3];
        cy += positionsM[v * 3 + 1];
        cz += positionsM[v * 3 + 2];
        const double e = CoordinateTraits<C>::elevationOf(positionsM[v * 3], positionsM[v * 3 + 1], positionsM[v * 3 + 2], sphereRadiusM);
        maxElev = std::max(maxElev, e);
        minElev = std::min(minElev, e);
    }
    return {{cx / double(gridCount), cy / double(gridCount), cz / double(gridCount)}, minElev, maxElev};
}

template <typename C>
std::vector<float> TileMeshBuilder<C>::packVertices(const FineGrid& fine, const std::optional<ParentLattice>& parent,
                                                    const Vec3& centerM, double skirtDepthM) {
    const size_t gridCount = static_cast<size_t>(fine.side) * fine.side;
    std::vector<float> vertices((gridCount + 4 * fine.side) * Mesh::kFloatsPerVertex);
    {
        BENCH_SCOPE("vertices");
        for (int j = 0; j < fine.side; ++j) {
            for (int i = 0; i < fine.side; ++i) {
                const size_t v = static_cast<size_t>(j) * fine.side + i;
                const Vec3 positionM = Vec3::from(&fine.positionsM[v * 3]);
                writeVertex(vertices, v, {positionM - centerM, {fine.slopes[v * 2], fine.slopes[v * 2 + 1]}, {fine.uv[v * 2], fine.uv[v * 2 + 1]}, parentPositionAt(fine, parent, j, i) - centerM, parentSlopeAt(fine, parent, j, i)});
            }
        }
    }
    {
        BENCH_SCOPE("skirts");
        size_t dst = gridCount;
        const int n = fine.side;
        auto skirt = [&](auto&& gridIndexAt) {
            for (int k = 0; k < n; ++k) {
                const size_t src = gridIndexAt(k);
                const Vec3 skirtPositionM = (CoordinateTraits<C>::skirtVertex(Vec3::from(&fine.positionsM[src * 3]), skirtDepthM) - centerM);
                const Slope2 slope{fine.slopes[src * 2], fine.slopes[src * 2 + 1]};
                writeVertex(vertices, dst++, {skirtPositionM, slope, {fine.uv[src * 2], fine.uv[src * 2 + 1]}, skirtPositionM, slope});
            }
        };
        skirt([n](int k) { return static_cast<size_t>(k); });                 // south row
        skirt([n](int k) { return static_cast<size_t>((n - 1) * n + k); });   // north row
        skirt([n](int k) { return static_cast<size_t>(k * n); });             // west column
        skirt([n](int k) { return static_cast<size_t>(k * n + (n - 1)); });   // east column
    }
    return vertices;
}

template <typename C>
double TileMeshBuilder<C>::boundingRadiusM(const std::vector<double>& positionsM, const Vec3& centerM, double skirtDepthM) {
    double radiusSquaredM2 = 0.0;
    for (size_t vertex = 0; vertex < positionsM.size() / 3; ++vertex) {
        const double dx = positionsM[vertex * 3] - centerM.x;
        const double dy = positionsM[vertex * 3 + 1] - centerM.y;
        const double dz = positionsM[vertex * 3 + 2] - centerM.z;
        radiusSquaredM2 = std::max(radiusSquaredM2, dx * dx + dy * dy + dz * dz);
    }
    return std::sqrt(radiusSquaredM2) + skirtDepthM;
}

// Area-weighted face normals accumulated on a (side x side) grid.
template <typename C>
std::vector<double> TileMeshBuilder<C>::smoothNormals(const std::vector<double>& positionsM, int side) {
    std::vector<double> out;
    out.assign(static_cast<size_t>(side) * side * 3, 0.0);
    for (int j = 0; j + 1 < side; ++j) {
        for (int i = 0; i + 1 < side; ++i) {
            const size_t tl = static_cast<size_t>(j) * side + i, tr = tl + 1, bl = tl + side, br = bl + 1;
            const size_t tris[2][3] = {{tl, bl, tr}, {tr, bl, br}};
            for (const auto& t : tris) {
                const Vec3 a = Vec3::from(&positionsM[t[0] * 3]);
                // j grows north, i grows east.
                const Vec3 faceNormal = cross(Vec3::from(&positionsM[t[2] * 3]) - a, Vec3::from(&positionsM[t[1] * 3]) - a);
                for (size_t k : t) {
                    (Vec3::from(&out[k * 3]) + faceNormal).store(&out[k * 3]);
                }
            }
        }
    }
    for (size_t v = 0; v < out.size() / 3; ++v) {
        normalize(Vec3::from(&out[v * 3])).store(&out[v * 3]);
    }
    return out;
}

// A normal expressed as east/north rise over its own up component.
template <typename C>
typename TileMeshBuilder<C>::Slope2 TileMeshBuilder<C>::slopeAt(const Vec3& positionM, const Vec3& normal) {
    const EnuFrame frame = CoordinateTraits<C>::frameAt(positionM);
    const double upComponent = std::max(dot(normal, frame.up), 0.05);
    return {dot(normal, frame.east) / upComponent, dot(normal, frame.north) / upComponent};
}

// Per-vertex slope of the smoothed grid normal.
template <typename C>
std::vector<double> TileMeshBuilder<C>::slopesFor(const std::vector<double>& positionsM, int side) {
    std::vector<double> outSlope2;
    const auto normals = smoothNormals(positionsM, side);
    const size_t count = static_cast<size_t>(side) * side;
    outSlope2.resize(count * 2);
    for (size_t v = 0; v < count; ++v) {
        const Slope2 slope = slopeAt(Vec3::from(&positionsM[v * 3]), Vec3::from(&normals[v * 3]));
        outSlope2[v * 2] = slope.east;
        outSlope2[v * 2 + 1] = slope.north;
    }
    return outSlope2;
}

template <typename C>
Vec3 TileMeshBuilder<C>::parentPositionAt(const FineGrid& fine, const std::optional<ParentLattice>& parent, int row, int column) {
    if (!parent) {
        return Vec3::from(&fine.positionsM[(static_cast<size_t>(row) * fine.side + column) * 3]);
    }
    return {parentComponentAt(*parent, parent->positionsM, 3, row, column, 0),
            parentComponentAt(*parent, parent->positionsM, 3, row, column, 1),
            parentComponentAt(*parent, parent->positionsM, 3, row, column, 2)};
}

template <typename C>
typename TileMeshBuilder<C>::Slope2 TileMeshBuilder<C>::parentSlopeAt(const FineGrid& fine, const std::optional<ParentLattice>& parent, int row, int column) {
    if (!parent) {
        const size_t offset = (static_cast<size_t>(row) * fine.side + column) * 2;
        return {fine.slopes[offset], fine.slopes[offset + 1]};
    }
    return {parentComponentAt(*parent, parent->slopes, 2, row, column, 0),
            parentComponentAt(*parent, parent->slopes, 2, row, column, 1)};
}

// Packed parent samples include one extra cell around the tile.
template <typename C>
double TileMeshBuilder<C>::parentComponentAt(const ParentLattice& parent, const std::vector<double>& samples, int componentsPerSample,
                                             int row, int column, int component) {
    const int southRow = row / parent.fineCellsPerCell,
              westColumn = column / parent.fineCellsPerCell;   // parent cell
    const int northRow = std::min(southRow + 1, parent.divisions),
              eastColumn = std::min(westColumn + 1, parent.divisions);
    const double rowFraction = double(row - southRow * parent.fineCellsPerCell) / parent.fineCellsPerCell,
                 columnFraction = double(column - westColumn * parent.fineCellsPerCell) / parent.fineCellsPerCell;
    auto at = [&](int r, int c) {
        return samples[(static_cast<size_t>(r + 1) * parent.side + (c + 1)) * componentsPerSample + component];
    };
    const double southValue = at(southRow, westColumn) * (1.0 - columnFraction) + at(southRow, eastColumn) * columnFraction;
    const double northValue = at(northRow, westColumn) * (1.0 - columnFraction) + at(northRow, eastColumn) * columnFraction;
    return southValue * (1.0 - rowFraction) + northValue * rowFraction;
}

template <typename C>
void TileMeshBuilder<C>::writeVertex(std::vector<float>& vertices, size_t vertexIndex, const VertexSample& sample) {
    float* output = &vertices[vertexIndex * Mesh::kFloatsPerVertex];
    auto writePosition = [&output](const Vec3& positionM) {
        *output++ = static_cast<float>(positionM.x);
        *output++ = static_cast<float>(positionM.y);
        *output++ = static_cast<float>(positionM.z);
    };
    auto writeSlope = [&output](Slope2 slope) {
        *output++ = static_cast<float>(slope.east);
        *output++ = static_cast<float>(slope.north);
    };
    writePosition(sample.positionRelativeM);
    writeSlope(sample.slope);
    *output++ = sample.tileUV[0];
    *output++ = sample.tileUV[1];
    writePosition(sample.parentPositionRelativeM);
    writeSlope(sample.parentSlope);
}

template class TileMeshBuilder<Spherical>;

template class TileMeshBuilder<Cartesian>;

}  // namespace planet
}  // namespace chrono
