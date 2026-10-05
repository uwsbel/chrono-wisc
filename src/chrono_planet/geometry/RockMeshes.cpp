#include "chrono_planet/geometry/RockMeshes.h"
#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/core/FieldHash.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <vector>

namespace chrono {
namespace planet {

namespace {

using field::hash01;
using util::Vec3;

inline Vec3 operator*(Vec3 v, double scale) { return {v.x * scale, v.y * scale, v.z * scale}; }
inline double magnitude(Vec3 v) { return std::sqrt(dot(v, v)); }

// Collapsed faces keep the generator's upward fallback normal.
Vec3 normalizedOrUp(Vec3 v) {
    const double size = magnitude(v);
    return size > 1e-12 ? v * (1.0 / size) : Vec3{0, 0, 1};
}

// 3D value noise on a unit lattice, trilinear over hashed corners with smoothstep weights.
inline double lattice(int x, int y, int z, int seed) {
    // hash01 is 2D, so fold z into the first axis with a large odd multiplier.
    return hash01(x * 73856093 ^ z * 19349663, y, seed);
}
double valueNoise3(Vec3 p, int seed) {
    const int ix = field::fastFloor(p.x), iy = field::fastFloor(p.y), iz = field::fastFloor(p.z);
    const double fx = p.x - ix, fy = p.y - iy, fz = p.z - iz;
    const double u = util::smoothstep01(fx), v = util::smoothstep01(fy),
                 w = util::smoothstep01(fz);
    double acc = 0.0;
    for (int dz = 0; dz < 2; ++dz) {
        for (int dy = 0; dy < 2; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
                const double wx = dx ? u : 1.0 - u;
                const double wy = dy ? v : 1.0 - v;
                const double wz = dz ? w : 1.0 - w;
                acc += wx * wy * wz * lattice(ix + dx, iy + dy, iz + dz, seed);
            }
        }
    }
    return acc * 2.0 - 1.0;   // signed, unit range
}

// Icosphere. Subdivision only appends vertices, so every LOD's vertices are a prefix of the finest.
struct Triangle {
    int a, b, c;
};

constexpr int kMaxSubdivision = RockMesh::kLods;
constexpr int kSupportSubdivision = 2;

struct IcosphereLevel {
    std::vector<Triangle> faces;
    int vertexCount = 0;
};

struct Icosphere {
    std::vector<Vec3> directions;   // appended in subdivision order
    std::array<IcosphereLevel, kMaxSubdivision + 1> levels;
};

std::vector<Triangle> appendSubdivision(const std::vector<Triangle>& faces, std::vector<Vec3>& directions) {
    std::map<std::pair<int, int>, int> midpoints;
    auto midpoint = [&](int a, int b) {
        const std::pair<int, int> edge = std::minmax(a, b);
        const auto found = midpoints.find(edge);
        if (found != midpoints.end()) {
            return found->second;
        }
        const int index = static_cast<int>(directions.size());
        directions.push_back(normalizedOrUp(directions[a] + directions[b]));
        midpoints.emplace(edge, index);
        return index;
    };

    std::vector<Triangle> next;
    next.reserve(faces.size() * 4);
    for (const Triangle& face : faces) {
        const int ab = midpoint(face.a, face.b);
        const int bc = midpoint(face.b, face.c);
        const int ca = midpoint(face.c, face.a);
        next.push_back({face.a, ab, ca});
        next.push_back({face.b, bc, ab});
        next.push_back({face.c, ca, bc});
        next.push_back({ab, bc, ca});
    }
    return next;
}

const Icosphere& icosphere() {
    static const Icosphere sphere = []() {
        Icosphere result;
        const double t = (1.0 + std::sqrt(5.0)) / 2.0;
        const Vec3 corners[] = {
            {-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t}, {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
        for (const Vec3& corner : corners) {
            result.directions.push_back(normalizedOrUp(corner));
        }
        result.levels[0] = {
            {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}},
            static_cast<int>(result.directions.size())};
        for (int level = 1; level <= kMaxSubdivision; ++level) {
            auto& next = result.levels[level];
            next.faces = appendSubdivision(result.levels[level - 1].faces, result.directions);
            next.vertexCount = static_cast<int>(result.directions.size());
        }
        return result;
    }();
    return sphere;
}

// A rock is one of three kinds, by its seed. Rounded: a lumpy ball with a few fracture faces, as a weathered boulder.
// Block: many deep cuts and little lumpiness, so it is all faces and edges, as freshly broken rock. Slab: a block a
// fifth as thick as it is long, cut about its rim and flat above and below, as a plate lifted from layered bedrock.
// Blocks and slabs are most of the library: a field of rounded ones alone reads as pebbles, not as broken rock.
enum class RockKind { Rounded, Block, Slab };
RockKind kindOf(int seed) {
    static constexpr RockKind kinds[RockMeshes::kCount] = {RockKind::Block, RockKind::Slab,  RockKind::Rounded, RockKind::Block,
                                                           RockKind::Slab,  RockKind::Rounded, RockKind::Block, RockKind::Slab,
                                                           RockKind::Rounded, RockKind::Block, RockKind::Slab,  RockKind::Slab};
    return kinds[((seed % RockMeshes::kCount) + RockMeshes::kCount) % RockMeshes::kCount];
}

// Slicing planes per rounded rock, and the normal crease angle that keeps slice faces as hard edges.
constexpr int kCutsMin = 3, kCutsMax = 6;
constexpr int kBlockCutsMin = 9, kBlockCutsMax = 14;
constexpr int kSlabCutsMin = 6, kSlabCutsMax = 9;
constexpr double kCreaseDeg = 32.0;

struct SlicePlane {
    Vec3 normal;
    double reachFraction;
};

std::vector<SlicePlane> slicePlanes(int seed) {
    const RockKind kind = kindOf(seed);
    const int low = kind == RockKind::Block ? kBlockCutsMin : kind == RockKind::Slab ? kSlabCutsMin : kCutsMin;
    const int high = kind == RockKind::Block ? kBlockCutsMax : kind == RockKind::Slab ? kSlabCutsMax : kCutsMax;
    // The fraction keeps the top count reachable but never exceeded.
    const int count = low + static_cast<int>(hash01(seed, 3, 7003) * (high - low + 0.99f));
    std::vector<SlicePlane> planes;
    planes.reserve(count + 2);
    if (kind == RockKind::Slab) {
        // Its two faces: flat, and not quite parallel
        planes.push_back({normalizedOrUp({0.10 * (hash01(seed, 40, 7040) - 0.5), 0.10 * (hash01(seed, 41, 7041) - 0.5), 1.0}), 0.70});
        planes.push_back({normalizedOrUp({0.16 * (hash01(seed, 42, 7042) - 0.5), 0.16 * (hash01(seed, 43, 7043) - 0.5), -1.0}), 0.70});
    }
    for (int i = 0; i < count; ++i) {
        // A slab is cut about its rim only: its cuts' normals lie near its plane
        const double z = (kind == RockKind::Slab ? 0.25 : 1.0) * (2.0 * hash01(seed, 10 + i, 7010) - 1.0);
        const double phi = util::kTwoPi * hash01(seed, 20 + i, 7020);
        const double radius = std::sqrt(std::max(0.0, 1.0 - z * z));
        // Deep cuts leave visible fracture faces rather than shallow caps. A block's and a slab's are deeper yet
        const double reach = kind == RockKind::Rounded ? 0.52 + 0.32 * hash01(seed, 30 + i, 7030) : 0.42 + 0.36 * hash01(seed, 30 + i, 7030);
        planes.push_back({{radius * std::cos(phi), radius * std::sin(phi), z}, reach});
    }
    return planes;
}

void applySlice(std::vector<Vec3>& positions, const SlicePlane& plane) {
    double reach = 0.0;
    for (const Vec3& position : positions) {
        reach = std::max(reach, dot(position, plane.normal));
    }
    const double offset = reach * plane.reachFraction;
    for (Vec3& position : positions) {
        const double projection = dot(position, plane.normal);
        if (projection > offset) {
            position = position - plane.normal * (projection - offset);
        }
    }
}

void scaleToUnitRadius(std::vector<Vec3>& positions) {
    double maxRadius = 0.0;
    for (const Vec3& position : positions) {
        maxRadius = std::max(maxRadius, magnitude(position));
    }
    if (maxRadius > 1e-9) {
        for (Vec3& position : positions) {
            position = position * (1.0 / maxRadius);
        }
    }
}

// Shape the finest lattice once. Coarser LODs use prefixes of these positions.
std::vector<Vec3> displacedPositions(int seed, const std::vector<Vec3>& directions) {
    // Unequal axes give the block a preferred resting orientation.
    const RockKind kind = kindOf(seed);
    const Vec3 axes{1.0, (kind == RockKind::Rounded ? 0.70 : 0.58) + (kind == RockKind::Rounded ? 0.22 : 0.34) * hash01(seed, 1, 7001),
                    kind == RockKind::Slab ? 0.16 + 0.12 * hash01(seed, 2, 7002) : (kind == RockKind::Block ? 0.45 : 0.52) + (kind == RockKind::Block ? 0.30 : 0.22) * hash01(seed, 2, 7002)};
    // A broken rock's faces are flat: it has a third of a rounded one's lumps
    const double lumps = kind == RockKind::Rounded ? 1.0 : 0.35;
    std::vector<Vec3> positions;
    positions.reserve(directions.size());
    for (const Vec3& direction : directions) {
        double radius = 1.0;
        radius += lumps * 0.20 * valueNoise3(direction * 1.6, seed * 3 + 11);
        radius += lumps * 0.12 * valueNoise3(direction * 3.4, seed * 3 + 12);
        radius += lumps * 0.07 * valueNoise3(direction * 7.0, seed * 3 + 13);
        radius = std::max(0.55, radius);
        positions.push_back({direction.x * radius * axes.x,
                             direction.y * radius * axes.y,
                             direction.z * radius * axes.z});
    }

    // Project caps onto fracture planes without changing topology.
    for (const SlicePlane& plane : slicePlanes(seed)) {
        applySlice(positions, plane);
    }
    // Keep ChRockInstance::radiusM equal to the maximum distance from the origin.
    scaleToUnitRadius(positions);
    return positions;
}

// Correct any inward slivers left by slicing before computing corner normals.
std::vector<Vec3> orientFacesAndComputeNormals(const std::vector<Vec3>& positions,
                                               std::vector<Triangle>& faces) {
    std::vector<Vec3> normals;
    normals.reserve(faces.size());
    for (Triangle& face : faces) {
        const Vec3& a = positions[face.a];
        const Vec3& b = positions[face.b];
        const Vec3& c = positions[face.c];
        Vec3 normal = normalizedOrUp(cross(b - a, c - a));
        const Vec3 centroid = (a + b + c) * (1.0 / 3.0);
        if (dot(normal, centroid) < 0.0) {
            std::swap(face.b, face.c);
            normal = normal * -1.0;
        }
        normals.push_back(normal);
    }
    return normals;
}

Vec3 cornerNormal(const std::vector<Vec3>& faceNormals, const std::vector<int>& neighbors,
                  const Vec3& faceNormal, double creaseCosine) {
    Vec3 normal{};
    for (int neighbor : neighbors) {
        if (dot(faceNormals[neighbor], faceNormal) >= creaseCosine) {
            normal = normal + faceNormals[neighbor];
        }
    }
    normal = normalizedOrUp(normal);
    return dot(normal, faceNormal) <= 0.0 ? faceNormal : normal;
}

void appendVertex(RockMesh& mesh, const Vec3& position, const Vec3& normal) {
    mesh.indices.push_back(static_cast<std::uint32_t>(mesh.vertices.size() / RockMesh::kVertexStride));
    mesh.vertices.insert(mesh.vertices.end(),
                         {static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(position.z),
                          static_cast<float>(normal.x), static_cast<float>(normal.y), static_cast<float>(normal.z)});
}

RockMesh::Lod appendLod(const std::vector<Vec3>& positions, const IcosphereLevel& level, RockMesh& mesh) {
    std::vector<Triangle> faces = level.faces;
    const std::vector<Vec3> faceNormals = orientFacesAndComputeNormals(positions, faces);
    std::vector<std::vector<int>> adjacentFaces(static_cast<size_t>(level.vertexCount));
    for (size_t i = 0; i < faces.size(); ++i) {
        for (int vertex : {faces[i].a, faces[i].b, faces[i].c}) {
            adjacentFaces[vertex].push_back(static_cast<int>(i));
        }
    }
    const double creaseCosine = std::cos(util::deg2rad(kCreaseDeg));
    const auto firstIndex = static_cast<std::uint32_t>(mesh.indices.size());
    for (size_t i = 0; i < faces.size(); ++i) {
        for (int vertex : {faces[i].a, faces[i].b, faces[i].c}) {
            // Average only neighbors within the crease angle to preserve slice edges.
            const Vec3 normal = cornerNormal(faceNormals, adjacentFaces[vertex], faceNormals[i], creaseCosine);
            appendVertex(mesh, positions[vertex], normal);
        }
    }
    return {firstIndex, static_cast<std::uint32_t>(mesh.indices.size()) - firstIndex};
}

RockMesh buildRock(int seed) {
    const Icosphere& sphere = icosphere();
    const std::vector<Vec3> positions = displacedPositions(seed, sphere.directions);
    RockMesh mesh;
    size_t indexCount = 0;
    for (int lod = 0; lod < RockMesh::kLods; ++lod) {
        indexCount += sphere.levels[kMaxSubdivision - lod].faces.size() * 3;
    }
    mesh.indices.reserve(indexCount);
    mesh.vertices.reserve(indexCount * RockMesh::kVertexStride);
    // LOD 0 is finest. Each subsequent LOD drops one subdivision level.
    for (int lod = 0; lod < RockMesh::kLods; ++lod) {
        mesh.lod[lod] = appendLod(positions, sphere.levels[kMaxSubdivision - lod], mesh);
    }

    const int supportCount = sphere.levels[kSupportSubdivision].vertexCount;
    mesh.support.reserve(static_cast<size_t>(supportCount) * 3);
    for (int i = 0; i < supportCount; ++i) {
        mesh.support.insert(mesh.support.end(),
                            {static_cast<float>(positions[i].x), static_cast<float>(positions[i].y), static_cast<float>(positions[i].z)});
    }
    return mesh;
}

const std::array<RockMesh, RockMeshes::kCount>& library() {
    // Built once per process, bit-identical everywhere.
    static const std::array<RockMesh, RockMeshes::kCount> lib = []() {
        std::array<RockMesh, RockMeshes::kCount> meshes;
        for (int i = 0; i < RockMeshes::kCount; ++i) {
            meshes[i] = buildRock(i);
        }
        return meshes;
    }();
    return lib;
}

}   // namespace

const RockMesh& RockMeshes::get(std::uint16_t meshId) {
    return library()[meshId % kCount];
}

RockMeshes::VerticalExtents RockMeshes::extentsZ(std::uint16_t meshId, const std::array<double, 4>& q) {
    const RockMesh& mesh = get(meshId);
    double lo = 1e30, hi = -1e30;
    // Only the z row of the rotation matters.
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double r20 = 2.0 * (x * z - w * y);
    const double r21 = 2.0 * (y * z + w * x);
    const double r22 = 1.0 - 2.0 * (x * x + y * y);
    for (size_t i = 0; i + 2 < mesh.support.size(); i += 3) {
        const double pz = r20 * mesh.support[i] + r21 * mesh.support[i + 1] + r22 * mesh.support[i + 2];
        lo = std::min(lo, pz);
        hi = std::max(hi, pz);
    }
    return {-lo, hi - lo};
}

}  // namespace planet
}  // namespace chrono
