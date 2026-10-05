#include "chrono_planet/geometry/RockSdf.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <thread>

#include "chrono_planet/geometry/RockMeshes.h"

namespace chrono {
namespace planet {

namespace {

struct Vec3f {
    float x = 0, y = 0, z = 0;
};
inline Vec3f operator+(Vec3f a, Vec3f b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3f operator-(Vec3f a, Vec3f b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3f operator*(Vec3f a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(Vec3f a, Vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3f cross(Vec3f a, Vec3f b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length(Vec3f a) { return std::sqrt(dot(a, a)); }

struct Triangle {
    Vec3f a, b, c;
    Vec3f centroid;
    float reach;   // bounding radius about the centroid
};

// Squared distance from p to the triangle (Ericson 2005, 5.1.5).
float distanceSquared(const Triangle& t, Vec3f p) {
    const Vec3f ab = t.b - t.a, ac = t.c - t.a, ap = p - t.a;
    const float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.f && d2 <= 0.f) {
        return dot(ap, ap);
    }
    const Vec3f bp = p - t.b;
    const float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.f && d4 <= d3) {
        return dot(bp, bp);
    }
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
        const float v = d1 / (d1 - d3);
        const Vec3f q = t.a + ab * v;
        return dot(p - q, p - q);
    }
    const Vec3f cp = p - t.c;
    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.f && d5 <= d6) {
        return dot(cp, cp);
    }
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
        const float w = d2 / (d2 - d6);
        const Vec3f q = t.a + ac * w;
        return dot(p - q, p - q);
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        const Vec3f q = t.b + (t.c - t.b) * w;
        return dot(p - q, p - q);
    }
    const float denom = 1.f / (va + vb + vc);
    const float v = vb * denom, w = vc * denom;
    const Vec3f q = t.a + ab * v + ac * w;
    return dot(p - q, p - q);
}

// Solid angle the triangle subtends at p (Van Oosterom and Strackee 1983), for the inside test.
float solidAngle(const Triangle& t, Vec3f p) {
    const Vec3f a = t.a - p, b = t.b - p, c = t.c - p;
    const float la = length(a), lb = length(b), lc = length(c);
    const float num = dot(a, cross(b, c));
    const float den = la * lb * lc + dot(a, b) * lc + dot(a, c) * lb + dot(b, c) * la;
    return 2.f * std::atan2(num, den);
}

// The LOD-1 surface, within a voxel of the drawn LOD 0.
std::vector<Triangle> meshTriangles(std::uint16_t meshId) {
    const RockMesh& mesh = RockMeshes::get(meshId);
    const RockMesh::Lod& lod = mesh.lod[1];
    std::vector<Triangle> triangles;
    triangles.reserve(lod.count / 3);
    auto position = [&](std::uint32_t index) {
        const float* vertex = &mesh.vertices[static_cast<size_t>(index) * RockMesh::kVertexStride];
        return Vec3f{vertex[0], vertex[1], vertex[2]};
    };
    for (std::uint32_t i = lod.first; i + 2 < lod.first + lod.count; i += 3) {
        Triangle triangle;
        triangle.a = position(mesh.indices[i]);
        triangle.b = position(mesh.indices[i + 1]);
        triangle.c = position(mesh.indices[i + 2]);
        triangle.centroid = (triangle.a + triangle.b + triangle.c) * (1.f / 3.f);
        triangle.reach = std::max({length(triangle.a - triangle.centroid),
                                   length(triangle.b - triangle.centroid),
                                   length(triangle.c - triangle.centroid)});
        triangles.push_back(triangle);
    }
    return triangles;
}

// Half the full sphere, the winding threshold between inside and out.
constexpr float kHalfSphereSr = 6.2831853f;

float signedDistance(const std::vector<Triangle>& triangles, Vec3f point) {
    float nearestSquared = 1e30f;
    float windingAngle = 0.f;
    for (const Triangle& triangle : triangles) {
        // The centroid sphere prunes distance checks. Every triangle still counts toward the winding.
        const float lowerBound = length(point - triangle.centroid) - triangle.reach;
        if (lowerBound * lowerBound < nearestSquared) {
            nearestSquared = std::min(nearestSquared, distanceSquared(triangle, point));
        }
        windingAngle += solidAngle(triangle, point);
    }
    const float distance = std::sqrt(nearestSquared);
    // Winding number is one inside and zero outside, so past a half is in.
    return windingAngle > kHalfSphereSr ? -distance : distance;
}

RockSdfGrid bake(std::uint16_t meshId) {
    const std::vector<Triangle> triangles = meshTriangles(meshId);
    constexpr int resolution = RockSdfGrid::kN;
    constexpr float extent = RockSdfGrid::kExtent;
    const float voxelSpacing = 2.f * extent / static_cast<float>(resolution - 1);
    RockSdfGrid grid;
    grid.d.resize(static_cast<size_t>(resolution) * resolution * resolution);
    for (int z = 0; z < resolution; ++z) {
        for (int y = 0; y < resolution; ++y) {
            for (int x = 0; x < resolution; ++x) {
                const Vec3f point{-extent + x * voxelSpacing,
                                  -extent + y * voxelSpacing,
                                  -extent + z * voxelSpacing};
                const size_t index = (static_cast<size_t>(z) * resolution + y) * resolution + x;
                grid.d[index] = signedDistance(triangles, point);
            }
        }
    }
    return grid;
}

const std::array<RockSdfGrid, RockMeshes::kCount>& library() {
    // Baked once per process, every mesh on its own thread.
    static const std::array<RockSdfGrid, RockMeshes::kCount> lib = []() {
        std::array<RockSdfGrid, RockMeshes::kCount> grids;
        std::vector<std::thread> workers;
        workers.reserve(RockMeshes::kCount);
        for (int i = 0; i < RockMeshes::kCount; ++i) {
            workers.emplace_back([&grids, i]() { grids[i] = bake(static_cast<std::uint16_t>(i)); });
        }
        for (std::thread& worker : workers) {
            worker.join();
        }
        return grids;
    }();
    return lib;
}

}   // namespace

const RockSdfGrid& RockSdf::get(std::uint16_t meshId) {
    return library()[meshId % RockMeshes::kCount];
}

}  // namespace planet
}  // namespace chrono
