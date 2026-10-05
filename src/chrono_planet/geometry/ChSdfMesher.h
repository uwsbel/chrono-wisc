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
// Triangle meshes of the zero level of signed distances sampled on a grid.
//
// =============================================================================

#ifndef CH_SDF_MESHER_H
#define CH_SDF_MESHER_H

#include <array>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "chrono/core/ChVector3.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_planet/ChApiPlanet.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Signed distances (negative inside) on a regular grid of nx x ny x nz points, x fastest, point (i, j, k) at
/// origin + spacing * (i, j, k).
struct CH_PLANET_API ChSdfGrid {
    ChVector3d origin;
    double spacing = 0;
    int nx = 0, ny = 0, nz = 0;
    std::vector<float> values;

    /// A grid over a box, `spacing` apart, every value `fill`.
    void Resize(const ChVector3d& lo, const ChVector3d& hi, double spacing, float fill);
    size_t Index(int i, int j, int k) const { return size_t(i) + size_t(nx) * (size_t(j) + size_t(ny) * size_t(k)); }
};

/// Append the zero level of a grid of signed distances to a mesh, with surface nets: a vertex in each cell the
/// level crosses, at the mean of its crossings of the cell's edges, and a quad across each crossed edge. Triangles
/// are counter-clockwise seen from outside, vertex normals follow the distances' gradient. Returns the number of
/// triangles added.
CH_PLANET_API size_t MeshSdfGrid(const ChSdfGrid& grid, ChTriangleMeshConnected& mesh);

/// As above, with only the quads across edges that start at grid points from `first` to `last` (inclusive, in
/// each axis): grids that overlap by a point, meshed with disjoint ranges, give meshes that meet without gaps.
CH_PLANET_API size_t MeshSdfGrid(const ChSdfGrid& grid, ChTriangleMeshConnected& mesh, const ChVector3i& first, const ChVector3i& last);

/// Signed distances splatted into a sparse lattice: bricks of 8^3 points exist only where something was splatted,
/// each meshed on its own (surface nets, as MeshSdfGrid), and meshes of neighboring bricks meet without gaps. Points
/// sit at whole multiples of the spacing. Spheres are merged by a smooth union, so a crowd of them reads as one mass.
///
/// Splatting everything again each frame and calling Remesh rebuilds the meshes only of the bricks whose samples
/// changed, and of the neighbors they share cells with; the others keep their meshes.
class CH_PLANET_API ChSparseSdfGrid {
  public:
    static constexpr int kBrick = 8;
    using Key = std::int64_t;

    /// A lattice of the given spacing; `far` is the distance of points nothing was splatted near.
    ChSparseSdfGrid(double spacing, float far);

    double GetSpacing() const { return m_h; }

    /// Start a new frame of splats: every brick is emptied (their meshes are kept for Remesh to compare).
    void Begin();

    /// Merge a sphere into the field, as the smooth union of width `blend` (m) with what is there.
    void SplatSphere(const ChVector3d& center, double radius, double blend);

    /// Merge a solid given by its signed distance over a box (lattice points outside the box are left alone) into the
    /// field, as a plain union.
    void SplatFunction(const ChVector3d& lo, const ChVector3d& hi, const std::function<float(const ChVector3d& p)>& distance);

    /// Mesh the bricks whose samples changed since the last Remesh, drop those left empty. Returns the number of
    /// bricks meshed.
    size_t Remesh();

    /// Append the meshes of all bricks to a mesh.
    void AppendMeshes(ChTriangleMeshConnected& mesh) const;

    size_t GetNumBricks() const { return m_bricks.size(); }

  private:
    using Samples = std::array<float, kBrick * kBrick * kBrick>;
    struct Brick {
        Samples now;     // this frame's splats
        Samples before;  // the samples its mesh was made from
        bool used = false;  // splatted this frame
        ChTriangleMeshConnected mesh;
    };

    static Key MakeKey(int bi, int bj, int bk);
    float Sample(int i, int j, int k) const;
    void MeshBrick(int bi, int bj, int bk, ChTriangleMeshConnected& mesh) const;

    double m_h;
    float m_far;
    std::unordered_map<Key, Brick> m_bricks;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
