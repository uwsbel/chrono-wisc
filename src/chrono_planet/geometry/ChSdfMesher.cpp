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

#include "chrono_planet/geometry/ChSdfMesher.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_set>

namespace chrono {
namespace planet {

void ChSdfGrid::Resize(const ChVector3d& lo, const ChVector3d& hi, double h, float fill) {
    origin = lo;
    spacing = h;
    nx = std::max(2, static_cast<int>(std::ceil((hi.x() - lo.x()) / h)) + 1);
    ny = std::max(2, static_cast<int>(std::ceil((hi.y() - lo.y()) / h)) + 1);
    nz = std::max(2, static_cast<int>(std::ceil((hi.z() - lo.z()) / h)) + 1);
    values.assign(size_t(nx) * ny * nz, fill);
}

size_t MeshSdfGrid(const ChSdfGrid& g, ChTriangleMeshConnected& mesh) {
    return MeshSdfGrid(g, mesh, ChVector3i(0, 0, 0), ChVector3i(g.nx - 1, g.ny - 1, g.nz - 1));
}

size_t MeshSdfGrid(const ChSdfGrid& g, ChTriangleMeshConnected& mesh, const ChVector3i& first, const ChVector3i& last) {
    const int nx = g.nx, ny = g.ny, nz = g.nz;
    if (nx < 2 || ny < 2 || nz < 2)
        return 0;
    const int cx = nx - 1, cy = ny - 1, cz = nz - 1;
    auto d = [&](int i, int j, int k) { return g.values[g.Index(i, j, k)]; };
    // Trilinear distance at fractional grid coordinates, for normals
    auto sample = [&](double a, double b, double c) {
        a = std::clamp(a, 0.0, nx - 1.0001), b = std::clamp(b, 0.0, ny - 1.0001), c = std::clamp(c, 0.0, nz - 1.0001);
        const int i = static_cast<int>(a), j = static_cast<int>(b), k = static_cast<int>(c);
        const double fa = a - i, fb = b - j, fc = c - k;
        double v = 0;
        for (int n = 0; n < 8; ++n) {
            const int di = n & 1, dj = (n >> 1) & 1, dk = (n >> 2) & 1;
            v += (di ? fa : 1 - fa) * (dj ? fb : 1 - fb) * (dk ? fc : 1 - fc) * d(i + di, j + dj, k + dk);
        }
        return v;
    };

    auto& vertices = mesh.GetCoordsVertices();
    auto& normals = mesh.GetCoordsNormals();
    auto& faces = mesh.GetIndicesVertices();
    auto& face_normals = mesh.GetIndicesNormals();
    const size_t first_face = faces.size();

    std::vector<int> vertex(size_t(cx) * cy * cz, -2);
    auto cell_vertex = [&](int a, int b, int c) -> int {
        if (a < 0 || b < 0 || c < 0 || a >= cx || b >= cy || c >= cz)
            return -1;
        int& slot = vertex[size_t(a) + size_t(cx) * (size_t(b) + size_t(cy) * size_t(c))];
        if (slot != -2)
            return slot;
        slot = -1;
        float corner[8];
        bool in = false, out = false;
        for (int n = 0; n < 8; ++n) {
            corner[n] = d(a + (n & 1), b + ((n >> 1) & 1), c + ((n >> 2) & 1));
            (corner[n] < 0 ? in : out) = true;
        }
        if (!in || !out)
            return slot;
        static const int edges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
        ChVector3d sum(0, 0, 0);
        int count = 0;
        for (const auto& e : edges) {
            const float d0 = corner[e[0]], d1 = corner[e[1]];
            if ((d0 < 0) == (d1 < 0))
                continue;
            const double t = d0 / (d0 - d1);
            const ChVector3d p0(e[0] & 1, (e[0] >> 1) & 1, (e[0] >> 2) & 1);
            const ChVector3d p1(e[1] & 1, (e[1] >> 1) & 1, (e[1] >> 2) & 1);
            sum += p0 + (p1 - p0) * t;
            ++count;
        }
        const ChVector3d f = sum / count;
        const double sa = a + f.x(), sb = b + f.y(), sc = c + f.z();
        ChVector3d grad(sample(sa + 0.5, sb, sc) - sample(sa - 0.5, sb, sc), sample(sa, sb + 0.5, sc) - sample(sa, sb - 0.5, sc),
                        sample(sa, sb, sc + 0.5) - sample(sa, sb, sc - 0.5));
        const double len = grad.Length();
        slot = static_cast<int>(vertices.size());
        vertices.push_back(g.origin + ChVector3d(sa, sb, sc) * g.spacing);
        normals.push_back(len > 1e-12 ? grad / len : ChVector3d(0, 0, 1));
        return slot;
    };
    auto add = [&](int a, int b, int c) {
        faces.push_back(ChVector3i(a, b, c));
        face_normals.push_back(ChVector3i(a, b, c));
    };

    for (int k = std::max(0, first.z()); k <= std::min(nz - 1, last.z()); ++k)
        for (int j = std::max(0, first.y()); j <= std::min(ny - 1, last.y()); ++j)
            for (int i = std::max(0, first.x()); i <= std::min(nx - 1, last.x()); ++i) {
                const int p[3] = {i, j, k};
                const bool inside = d(i, j, k) < 0;
                for (int axis = 0; axis < 3; ++axis) {
                    int q[3] = {i, j, k};
                    if (++q[axis] >= (axis == 0 ? nx : axis == 1 ? ny : nz))
                        continue;
                    if ((d(q[0], q[1], q[2]) < 0) == inside)
                        continue;
                    const int u = (axis + 1) % 3, v = (axis + 2) % 3;
                    int ids[4];
                    bool ok = true;
                    for (int n = 0; n < 4 && ok; ++n) {
                        int cell[3] = {p[0], p[1], p[2]};
                        if (n == 1 || n == 2)
                            --cell[u];
                        if (n == 2 || n == 3)
                            --cell[v];
                        ids[n] = cell_vertex(cell[0], cell[1], cell[2]);
                        ok = ids[n] >= 0;
                    }
                    if (!ok)
                        continue;
                    if (!inside)
                        std::swap(ids[1], ids[3]);
                    if ((vertices[ids[0]] - vertices[ids[2]]).Length2() <= (vertices[ids[1]] - vertices[ids[3]]).Length2()) {
                        add(ids[0], ids[1], ids[2]);
                        add(ids[0], ids[2], ids[3]);
                    } else {
                        add(ids[0], ids[1], ids[3]);
                        add(ids[1], ids[2], ids[3]);
                    }
                }
            }
    return faces.size() - first_face;
}

// -----------------------------------------------------------------------------

namespace {
constexpr int B = ChSparseSdfGrid::kBrick;
constexpr int kBias = 1 << 20;  // brick coordinates are stored offset to keep them positive

int FloorDiv(int a, int b) {
    return a >= 0 ? a / b : -((-a + b - 1) / b);
}

size_t Local(int li, int lj, int lk) {
    return size_t(li) + size_t(B) * (size_t(lj) + size_t(B) * size_t(lk));
}

float SmoothMin(float a, float b, float k) {
    if (!(k > 0))
        return std::min(a, b);
    const float h = std::max(k - std::abs(a - b), 0.0f) / k;
    return std::min(a, b) - h * h * k * 0.25f;
}
}  // namespace

ChSparseSdfGrid::ChSparseSdfGrid(double spacing, float far) : m_h(spacing), m_far(far) {}

ChSparseSdfGrid::Key ChSparseSdfGrid::MakeKey(int bi, int bj, int bk) {
    return (Key(bi + kBias) << 42) | (Key(bj + kBias) << 21) | Key(bk + kBias);
}

void ChSparseSdfGrid::Begin() {
    for (auto& entry : m_bricks) {
        entry.second.now.fill(m_far);
        entry.second.used = false;
    }
}

float ChSparseSdfGrid::Sample(int i, int j, int k) const {
    const int bi = FloorDiv(i, B), bj = FloorDiv(j, B), bk = FloorDiv(k, B);
    const auto it = m_bricks.find(MakeKey(bi, bj, bk));
    if (it == m_bricks.end() || !it->second.used)
        return m_far;
    return it->second.now[Local(i - bi * B, j - bj * B, k - bk * B)];
}

void ChSparseSdfGrid::SplatSphere(const ChVector3d& center, double radius, double blend) {
    const double reach = radius + blend + m_h;
    const int i0 = static_cast<int>(std::floor((center.x() - reach) / m_h)), i1 = static_cast<int>(std::ceil((center.x() + reach) / m_h));
    const int j0 = static_cast<int>(std::floor((center.y() - reach) / m_h)), j1 = static_cast<int>(std::ceil((center.y() + reach) / m_h));
    const int k0 = static_cast<int>(std::floor((center.z() - reach) / m_h)), k1 = static_cast<int>(std::ceil((center.z() + reach) / m_h));
    const float r = static_cast<float>(radius), k = static_cast<float>(blend);
    for (int bk = FloorDiv(k0, B); bk <= FloorDiv(k1, B); ++bk)
        for (int bj = FloorDiv(j0, B); bj <= FloorDiv(j1, B); ++bj)
            for (int bi = FloorDiv(i0, B); bi <= FloorDiv(i1, B); ++bi) {
                auto [it, created] = m_bricks.try_emplace(MakeKey(bi, bj, bk));
                Brick& brick = it->second;
                if (created) {
                    brick.now.fill(m_far);
                    brick.before.fill(m_far);  // nothing was drawn here
                }
                brick.used = true;
                for (int kk = std::max(k0, bk * B); kk <= std::min(k1, bk * B + B - 1); ++kk)
                    for (int jj = std::max(j0, bj * B); jj <= std::min(j1, bj * B + B - 1); ++jj)
                        for (int ii = std::max(i0, bi * B); ii <= std::min(i1, bi * B + B - 1); ++ii) {
                            const ChVector3d p(ii * m_h, jj * m_h, kk * m_h);
                            float& v = brick.now[Local(ii - bi * B, jj - bj * B, kk - bk * B)];
                            v = SmoothMin(v, static_cast<float>((p - center).Length()) - r, k);
                        }
            }
}

void ChSparseSdfGrid::SplatFunction(const ChVector3d& lo, const ChVector3d& hi, const std::function<float(const ChVector3d& p)>& distance) {
    const int i0 = static_cast<int>(std::floor(lo.x() / m_h)), i1 = static_cast<int>(std::ceil(hi.x() / m_h));
    const int j0 = static_cast<int>(std::floor(lo.y() / m_h)), j1 = static_cast<int>(std::ceil(hi.y() / m_h));
    const int k0 = static_cast<int>(std::floor(lo.z() / m_h)), k1 = static_cast<int>(std::ceil(hi.z() / m_h));
    for (int bk = FloorDiv(k0, B); bk <= FloorDiv(k1, B); ++bk)
        for (int bj = FloorDiv(j0, B); bj <= FloorDiv(j1, B); ++bj)
            for (int bi = FloorDiv(i0, B); bi <= FloorDiv(i1, B); ++bi) {
                auto [it, created] = m_bricks.try_emplace(MakeKey(bi, bj, bk));
                Brick& brick = it->second;
                if (created) {
                    brick.now.fill(m_far);
                    brick.before.fill(m_far);
                }
                brick.used = true;
                for (int kk = std::max(k0, bk * B); kk <= std::min(k1, bk * B + B - 1); ++kk)
                    for (int jj = std::max(j0, bj * B); jj <= std::min(j1, bj * B + B - 1); ++jj)
                        for (int ii = std::max(i0, bi * B); ii <= std::min(i1, bi * B + B - 1); ++ii) {
                            float& v = brick.now[Local(ii - bi * B, jj - bj * B, kk - bk * B)];
                            v = std::min(v, std::clamp(distance(ChVector3d(ii * m_h, jj * m_h, kk * m_h)), -m_far, m_far));
                        }
            }
}

void ChSparseSdfGrid::MeshBrick(int bi, int bj, int bk, ChTriangleMeshConnected& mesh) const {
    // The brick's cells [8b - 1, 8b + 7] read samples [8b - 1, 8b + 8], some in its neighbors: copy them into a small
    // dense grid and mesh the quads across the edges that start at the brick's own points, 1 to 8 in that grid
    ChSdfGrid grid;
    grid.origin = ChVector3d((bi * B - 1) * m_h, (bj * B - 1) * m_h, (bk * B - 1) * m_h);
    grid.spacing = m_h;
    grid.nx = grid.ny = grid.nz = B + 2;
    grid.values.resize(size_t(B + 2) * (B + 2) * (B + 2));
    for (int c = 0; c < B + 2; ++c)
        for (int b = 0; b < B + 2; ++b)
            for (int a = 0; a < B + 2; ++a)
                grid.values[grid.Index(a, b, c)] = Sample(bi * B - 1 + a, bj * B - 1 + b, bk * B - 1 + c);
    MeshSdfGrid(grid, mesh, ChVector3i(1, 1, 1), ChVector3i(B, B, B));
}

size_t ChSparseSdfGrid::Remesh() {
    auto coords = [](Key key, int& bi, int& bj, int& bk) {
        bi = static_cast<int>((key >> 42) & ((1 << 21) - 1)) - kBias;
        bj = static_cast<int>((key >> 21) & ((1 << 21) - 1)) - kBias;
        bk = static_cast<int>(key & ((1 << 21) - 1)) - kBias;
    };
    // Bricks whose samples changed, and the neighbors whose cells read them; bricks nothing was splatted into go
    std::unordered_set<Key> dirty;
    std::vector<Key> empty;
    for (auto& [key, brick] : m_bricks) {
        if (!brick.used)
            empty.push_back(key);
        if (std::memcmp(brick.now.data(), brick.before.data(), sizeof(Samples)) == 0)
            continue;
        int bi, bj, bk;
        coords(key, bi, bj, bk);
        for (int dk = -1; dk <= 1; ++dk)
            for (int dj = -1; dj <= 1; ++dj)
                for (int di = -1; di <= 1; ++di)
                    dirty.insert(MakeKey(bi + di, bj + dj, bk + dk));
    }
    for (Key key : empty)
        m_bricks.erase(key);
    size_t meshed = 0;
    for (Key key : dirty) {
        auto it = m_bricks.find(key);
        if (it == m_bricks.end())
            continue;
        int bi, bj, bk;
        coords(key, bi, bj, bk);
        it->second.mesh.Clear();
        MeshBrick(bi, bj, bk, it->second.mesh);
        ++meshed;
    }
    for (auto& entry : m_bricks)
        entry.second.before = entry.second.now;
    return meshed;
}

void ChSparseSdfGrid::AppendMeshes(ChTriangleMeshConnected& mesh) const {
    for (const auto& entry : m_bricks) {
        const auto& m = entry.second.mesh;
        const int base = static_cast<int>(mesh.GetCoordsVertices().size());
        mesh.GetCoordsVertices().insert(mesh.GetCoordsVertices().end(), m.GetCoordsVertices().begin(), m.GetCoordsVertices().end());
        mesh.GetCoordsNormals().insert(mesh.GetCoordsNormals().end(), m.GetCoordsNormals().begin(), m.GetCoordsNormals().end());
        for (const auto& f : m.GetIndicesVertices()) {
            mesh.GetIndicesVertices().push_back(ChVector3i(f[0] + base, f[1] + base, f[2] + base));
            mesh.GetIndicesNormals().push_back(ChVector3i(f[0] + base, f[1] + base, f[2] + base));
        }
    }
}

}  // namespace planet
}  // namespace chrono
