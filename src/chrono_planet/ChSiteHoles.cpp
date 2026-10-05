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

#include "chrono_planet/ChSiteHoles.h"

#include <algorithm>
#include <array>

namespace chrono {
namespace planet {

namespace {

// A point of a face, by its barycentric weights on the face's corners, and its x/y
struct Corner {
    std::array<double, 3> w;
    double x, y;
};
using Polygon = std::vector<Corner>;

// Keep the part of a convex polygon where a * x + b * y <= c
Polygon Clip(const Polygon& in, double a, double b, double c) {
    Polygon out;
    const size_t n = in.size();
    for (size_t i = 0; i < n; ++i) {
        const Corner& p = in[i];
        const Corner& q = in[(i + 1) % n];
        const double dp = a * p.x + b * p.y - c;
        const double dq = a * q.x + b * q.y - c;
        if (dp <= 0)
            out.push_back(p);
        if ((dp < 0 && dq > 0) || (dp > 0 && dq < 0)) {
            const double t = dp / (dp - dq);
            Corner r;
            for (int k = 0; k < 3; ++k)
                r.w[k] = p.w[k] + t * (q.w[k] - p.w[k]);
            r.x = p.x + t * (q.x - p.x);
            r.y = p.y + t * (q.y - p.y);
            out.push_back(r);
        }
    }
    return out;
}

double Area2(const Polygon& p) {
    double a = 0;
    for (size_t i = 0; i < p.size(); ++i) {
        const Corner& u = p[i];
        const Corner& v = p[(i + 1) % p.size()];
        a += u.x * v.y - v.x * u.y;
    }
    return std::abs(a);
}

// The parts of a polygon outside a rectangle, as convex pieces: left of it, right of it, and below and above it
// between its sides
void Outside(const Polygon& in, const ChSiteRegion& r, std::vector<Polygon>& out) {
    Polygon piece = Clip(in, 1, 0, r.min_x);
    if (piece.size() >= 3)
        out.push_back(piece);
    piece = Clip(in, -1, 0, -r.max_x);
    if (piece.size() >= 3)
        out.push_back(piece);
    const Polygon middle = Clip(Clip(in, -1, 0, -r.min_x), 1, 0, r.max_x);
    if (middle.size() < 3)
        return;
    piece = Clip(middle, 0, 1, r.min_y);
    if (piece.size() >= 3)
        out.push_back(piece);
    piece = Clip(middle, 0, -1, -r.max_y);
    if (piece.size() >= 3)
        out.push_back(piece);
}

}  // namespace

size_t CutSiteHoles(ChTriangleMeshConnected& mesh, const std::vector<ChSiteRegion>& holes) {
    std::vector<ChSiteRegion> active;
    for (const auto& h : holes)
        if (!h.IsEmpty())
            active.push_back(h);
    if (active.empty())
        return 0;

    auto& vertices = mesh.GetCoordsVertices();
    auto& normals = mesh.GetCoordsNormals();
    auto& uvs = mesh.GetCoordsUV();
    auto& colors = mesh.GetCoordsColors();
    auto& faces = mesh.GetIndicesVertices();
    auto& face_normals = mesh.GetIndicesNormals();
    auto& face_uvs = mesh.GetIndicesUV();
    auto& face_colors = mesh.GetIndicesColors();
    auto& face_materials = mesh.GetIndicesMaterials();
    const size_t nf = faces.size();
    const bool has_normals = face_normals.size() == nf;
    const bool has_uvs = face_uvs.size() == nf;
    const bool has_face_colors = face_colors.size() == nf;
    const bool has_vertex_colors = !has_face_colors && colors.size() == vertices.size() && !colors.empty();
    const bool has_materials = face_materials.size() == nf;

    std::vector<ChVector3i> new_faces, new_normals, new_uvs, new_colors;
    std::vector<int> new_materials;
    new_faces.reserve(nf);
    size_t cut = 0;

    for (size_t f = 0; f < nf; ++f) {
        // Copies, as new vertices are appended below
        const ChVector3i face = faces[f];
        const ChVector3d v[3] = {vertices[face[0]], vertices[face[1]], vertices[face[2]]};
        const double x0 = std::min({v[0].x(), v[1].x(), v[2].x()}), x1 = std::max({v[0].x(), v[1].x(), v[2].x()});
        const double y0 = std::min({v[0].y(), v[1].y(), v[2].y()}), y1 = std::max({v[0].y(), v[1].y(), v[2].y()});

        // Faces clear of every hole are kept whole
        std::vector<Polygon> pieces;
        bool touched = false;
        for (const auto& h : active)
            touched = touched || !(x1 <= h.min_x || x0 >= h.max_x || y1 <= h.min_y || y0 >= h.max_y);
        auto keep = [&](size_t g) {
            new_faces.push_back(faces[g]);
            if (has_normals)
                new_normals.push_back(face_normals[g]);
            if (has_uvs)
                new_uvs.push_back(face_uvs[g]);
            if (has_face_colors)
                new_colors.push_back(face_colors[g]);
            if (has_materials)
                new_materials.push_back(face_materials[g]);
        };
        if (!touched) {
            keep(f);
            continue;
        }

        // Cut the face by each hole in turn
        pieces.push_back({{{{1, 0, 0}}, v[0].x(), v[0].y()}, {{{0, 1, 0}}, v[1].x(), v[1].y()}, {{{0, 0, 1}}, v[2].x(), v[2].y()}});
        for (const auto& h : active) {
            std::vector<Polygon> next;
            for (const auto& p : pieces)
                Outside(p, h, next);
            pieces.swap(next);
        }
        ++cut;

        // Each piece's corners as indices into the vertex, normal and UV arrays: the face's own corners keep
        // theirs, others get new entries interpolated across the face. A fan of faces covers the piece.
        struct Index {
            int vertex, normal, uv;
        };
        for (const auto& p : pieces) {
            if (Area2(p) < 1e-14)
                continue;
            std::vector<Index> ids;
            for (const auto& c : p) {
                int own = -1;
                for (int k = 0; k < 3; ++k)
                    if (c.w[k] > 1 - 1e-12)
                        own = k;
                if (own >= 0) {
                    ids.push_back({face[own], has_normals ? face_normals[f][own] : 0, has_uvs ? face_uvs[f][own] : 0});
                    continue;
                }
                Index id{static_cast<int>(vertices.size()), 0, 0};
                vertices.push_back(v[0] * c.w[0] + v[1] * c.w[1] + v[2] * c.w[2]);
                if (has_normals) {
                    const ChVector3i& fn = face_normals[f];
                    const ChVector3d nrm = normals[fn[0]] * c.w[0] + normals[fn[1]] * c.w[1] + normals[fn[2]] * c.w[2];
                    const double len = nrm.Length();
                    id.normal = static_cast<int>(normals.size());
                    normals.push_back(len > 1e-12 ? nrm / len : ChVector3d(0, 0, 1));
                }
                if (has_uvs) {
                    const ChVector3i& fu = face_uvs[f];
                    id.uv = static_cast<int>(uvs.size());
                    uvs.push_back(uvs[fu[0]] * c.w[0] + uvs[fu[1]] * c.w[1] + uvs[fu[2]] * c.w[2]);
                }
                if (has_vertex_colors) {
                    const ChColor c0 = colors[face[0]], c1 = colors[face[1]], c2 = colors[face[2]];
                    colors.push_back(ChColor(float(c0.R * c.w[0] + c1.R * c.w[1] + c2.R * c.w[2]), float(c0.G * c.w[0] + c1.G * c.w[1] + c2.G * c.w[2]),
                                             float(c0.B * c.w[0] + c1.B * c.w[1] + c2.B * c.w[2])));
                }
                ids.push_back(id);
            }
            for (size_t t = 1; t + 1 < ids.size(); ++t) {
                const Index& a = ids[0];
                const Index& b = ids[t];
                const Index& c = ids[t + 1];
                new_faces.push_back(ChVector3i(a.vertex, b.vertex, c.vertex));
                if (has_normals)
                    new_normals.push_back(ChVector3i(a.normal, b.normal, c.normal));
                if (has_uvs)
                    new_uvs.push_back(ChVector3i(a.uv, b.uv, c.uv));
                if (has_face_colors)
                    new_colors.push_back(face_colors[f]);
                if (has_materials)
                    new_materials.push_back(face_materials[f]);
            }
        }
    }
    if (cut == 0)
        return 0;

    faces.swap(new_faces);
    if (has_normals)
        face_normals.swap(new_normals);
    if (has_uvs)
        face_uvs.swap(new_uvs);
    if (has_face_colors)
        face_colors.swap(new_colors);
    if (has_materials)
        face_materials.swap(new_materials);
    return cut;
}

}  // namespace planet
}  // namespace chrono
