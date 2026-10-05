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

#include "chrono_planet/ChPlanetVisualMesh.h"
#include "chrono_planet/ChSiteHoles.h"

#include <algorithm>
#include <unordered_map>

#include "chrono_planet/core/SphereMath.h"

namespace chrono {
namespace planet {

namespace {

// Position (center-relative) and slopes of a tile vertex, blended from its parent's (morph 0) to its own (1).
struct MorphedVertex {
    double x, y, z, slope_east, slope_north;
    MorphedVertex(const float* v, double morph)
        : x(v[7] + (v[0] - v[7]) * morph),
          y(v[8] + (v[1] - v[8]) * morph),
          z(v[9] + (v[2] - v[9]) * morph),
          slope_east(v[10] + (v[3] - v[10]) * morph),
          slope_north(v[11] + (v[4] - v[11]) * morph) {}
};

double Ease(double s) {
    s = std::clamp(s, 0.0, 1.0);
    return s * s * (3.0 - 2.0 * s);
}

}  // namespace

ChPlanetVisualMesh::ChPlanetVisualMesh(ChSystem* sys, std::shared_ptr<ChPlanetQuadtree> world, const ChSiteFrame& site)
    : m_world(std::move(world)), m_site(site), m_max_distance(5000.0), m_mesh_version(~0ull), m_num_tiles(0), m_num_triangles(0), m_num_rebuilds(0) {
    const util::Vec3 origin = util::pointOnSphere(site.GetOriginLongitude(), site.GetOriginLatitude(), site.GetRadius() + site.GetOriginElevation());
    const util::EnuFrame enu = util::enuAt(site.GetOriginLongitude(), site.GetOriginLatitude());
    m_origin = ChVector3d(origin.x, origin.y, origin.z);
    m_east = ChVector3d(enu.east.x, enu.east.y, enu.east.z);
    m_north = ChVector3d(enu.north.x, enu.north.y, enu.north.z);
    m_up = ChVector3d(enu.up.x, enu.up.y, enu.up.z);

    m_material = chrono_types::make_shared<ChVisualMaterial>();
    m_material->SetDiffuseColor(ChColor(0.42f, 0.41f, 0.39f));
    m_material->SetRoughness(0.95f);
    m_material->SetMetallic(0.0f);

    m_body = chrono_types::make_shared<ChBody>();
    m_body->SetFixed(true);
    m_body->EnableCollision(false);
    m_body->AddVisualModel(chrono_types::make_shared<ChVisualModel>());
    sys->AddBody(m_body);
}

ChPlanetVisualMesh::~ChPlanetVisualMesh() {}

void ChPlanetVisualMesh::SetMaterial(std::shared_ptr<ChVisualMaterial> material) {
    m_material = std::move(material);
    for (auto& tile : m_tiles) {
        auto& materials = tile.second->GetMaterials();
        if (materials.empty())
            materials.push_back(m_material);
        else
            materials[0] = m_material;
    }
}

void ChPlanetVisualMesh::SetCompaction(std::shared_ptr<ChDeformationFilter> filter, std::shared_ptr<ChVisualMaterial> material, double min_depth) {
    std::vector<CompactionLevel> levels;
    if (material)
        levels.push_back({min_depth, std::move(material)});
    SetCompaction(std::move(filter), std::move(levels));
}

void ChPlanetVisualMesh::SetCompaction(std::shared_ptr<ChDeformationFilter> filter, std::vector<CompactionLevel> levels) {
    m_compaction = std::move(filter);
    std::sort(levels.begin(), levels.end(), [](const CompactionLevel& a, const CompactionLevel& b) { return a.min_depth < b.min_depth; });
    m_compaction_levels = std::move(levels);
}

bool ChPlanetVisualMesh::MarkCompaction(const ChTileMesh& tile, ChTriangleMeshConnected& mesh) const {
    if (!m_compaction || m_compaction_levels.empty())
        return false;

    // How far each vertex has been lowered, at this tile's resolution, as the terrain drawn for it has it.
    const double spacing = m_world->GetSurface()->GetSampleSpacingAtZoom(tile.level);
    const size_t n = tile.vertexData.size() / ChTileMesh::kFloatsPerVertex;
    std::vector<double> depth(n);
    for (size_t i = 0; i < n; ++i) {
        const float* v = &tile.vertexData[i * ChTileMesh::kFloatsPerVertex];
        const util::LonLat ll = util::lonLatOf(tile.centerX + v[0], tile.centerY + v[1], tile.centerZ + v[2]);
        depth[i] = -m_compaction->GetDelta(ll.lon, ll.lat, spacing);
    }

    // A rut wall drops within a vertex or two, and the soil nodes sink unevenly along it, so a contour through the
    // raw depths zigzags with the grid. Smooth them over each vertex's neighbors first; this only moves the
    // boundary between the materials, not the ground.
    const auto& tile_faces = mesh.GetIndicesVertices();
    std::vector<double> sum(n), count(n);
    for (int pass = 0; pass < 2; ++pass) {
        std::fill(sum.begin(), sum.end(), 0.0);
        std::fill(count.begin(), count.end(), 0.0);
        for (const auto& face : tile_faces) {
            for (int e = 0; e < 3; ++e) {
                const int a = face[e], b = face[(e + 1) % 3];
                sum[a] += depth[b], count[a] += 1;
                sum[b] += depth[a], count[b] += 1;
            }
        }
        for (size_t i = 0; i < n; ++i)
            if (count[i] > 0)
                depth[i] = 0.5 * depth[i] + 0.5 * sum[i] / count[i];
    }

    // Split the faces each compaction contour crosses along it, so the boundaries between the materials follow the
    // rut outline, interpolated linearly along each edge, rather than the tile's grid. The tile's vertices come first
    // in the mesh, and each face indexes positions, normals and UVs alike. Each face then takes the material of the
    // deepest level it lies beyond.
    auto& vertices = mesh.GetCoordsVertices();
    auto& normals = mesh.GetCoordsNormals();
    auto& uvs = mesh.GetCoordsUV();
    std::vector<ChVector3i> faces = mesh.GetIndicesVertices();
    bool any = false;
    for (const auto& level : m_compaction_levels) {
        const double contour = level.min_depth;
        std::vector<ChVector3i> split;
        split.reserve(faces.size());
        std::unordered_map<std::uint64_t, int> crossings;  // contour point on each split edge, by its end vertices
        auto crossing = [&](int a, int b) {
            const std::uint64_t key = (std::uint64_t(std::min(a, b)) << 32) | std::uint64_t(std::max(a, b));
            auto it = crossings.find(key);
            if (it != crossings.end())
                return it->second;
            // The two ends lie on either side of the contour, so the depths differ.
            const double t = (depth[a] - contour) / (depth[a] - depth[b]);
            const int i = static_cast<int>(vertices.size());
            vertices.push_back(vertices[a] + (vertices[b] - vertices[a]) * t);
            normals.push_back((normals[a] + (normals[b] - normals[a]) * t).GetNormalized());
            uvs.push_back(uvs[a] + (uvs[b] - uvs[a]) * t);
            depth.push_back(contour);
            crossings.emplace(key, i);
            return i;
        };
        for (const auto& face : faces) {
            const int v[3] = {face.x(), face.y(), face.z()};
            const bool in[3] = {depth[v[0]] > contour, depth[v[1]] > contour, depth[v[2]] > contour};
            any |= in[0] || in[1] || in[2];
            if (in[0] == in[1] && in[1] == in[2]) {
                split.push_back(face);
                continue;
            }
            // One vertex is alone on its side of the contour. Starting from it, in the face's order, cut off its
            // corner and fill the rest with two triangles, keeping the face's orientation.
            const int k = (in[0] != in[1] && in[0] != in[2]) ? 0 : (in[1] != in[0] && in[1] != in[2]) ? 1 : 2;
            const int a = v[k], b = v[(k + 1) % 3], c = v[(k + 2) % 3];
            const int ab = crossing(a, b), ac = crossing(a, c);
            split.push_back(ChVector3i(a, ab, ac));
            split.push_back(ChVector3i(ab, b, c));
            split.push_back(ChVector3i(ab, c, ac));
        }
        faces = std::move(split);
    }
    if (!any)
        return false;
    std::vector<int> face_materials(faces.size(), 0);
    for (size_t f = 0; f < faces.size(); ++f) {
        const double d = (depth[faces[f].x()] + depth[faces[f].y()] + depth[faces[f].z()]) / 3;
        for (size_t l = 0; l < m_compaction_levels.size(); ++l)
            if (d > m_compaction_levels[l].min_depth)
                face_materials[f] = static_cast<int>(l) + 1;
    }
    mesh.GetIndicesVertices() = faces;
    mesh.GetIndicesNormals() = faces;
    mesh.GetIndicesUV() = std::move(faces);
    mesh.GetIndicesMaterials() = std::move(face_materials);
    return true;
}

ChVector3d ChPlanetVisualMesh::ToPlanet(const ChVector3d& p) const {
    if (m_shape == Shape::SPHERE)
        return m_origin + p.x() * m_east + p.y() * m_north + p.z() * m_up;
    double lon, lat;
    m_site.ToLonLat(p.x(), p.y(), lon, lat);
    const util::Vec3 q = util::pointOnSphere(lon, lat, m_site.GetRadius() + m_site.GetOriginElevation() + p.z());
    return ChVector3d(q.x, q.y, q.z);
}

void ChPlanetVisualMesh::PlaceOnSphere(const ChTileMesh& tile, ChTriangleMeshConnected& mesh, double morph) const {
    auto& vertices = mesh.GetCoordsVertices();
    auto& normals = mesh.GetCoordsNormals();
    const size_t n = tile.vertexData.size() / ChTileMesh::kFloatsPerVertex;
    for (size_t i = 0; i < n; ++i) {
        const MorphedVertex v(&tile.vertexData[i * ChTileMesh::kFloatsPerVertex], morph);
        const ChVector3d p = ChVector3d(tile.centerX + v.x, tile.centerY + v.y, tile.centerZ + v.z) - m_origin;
        vertices[i] = ChVector3d(Vdot(p, m_east), Vdot(p, m_north), Vdot(p, m_up));
        // AppendTile's normal is in the east-north-up axes at the vertex; turn it into those at the origin.
        const util::LonLat ll = util::lonLatOf(tile.centerX + v.x, tile.centerY + v.y, tile.centerZ + v.z);
        const util::EnuFrame enu = util::enuAt(ll.lon, ll.lat);
        const ChVector3d& nl = normals[i];
        const ChVector3d np(enu.east.x * nl.x() + enu.north.x * nl.y() + enu.up.x * nl.z(), enu.east.y * nl.x() + enu.north.y * nl.y() + enu.up.y * nl.z(),
                            enu.east.z * nl.x() + enu.north.z * nl.y() + enu.up.z * nl.z());
        normals[i] = ChVector3d(Vdot(np, m_east), Vdot(np, m_north), Vdot(np, m_up));
    }
}

void ChPlanetVisualMesh::AddHole(const ChSiteRegion& hole) {
    m_holes.push_back(hole);
    m_tiles.clear();
    m_mesh_version = ~0ull;
}

void ChPlanetVisualMesh::ClearHoles() {
    m_holes.clear();
    m_tiles.clear();
    m_mesh_version = ~0ull;
}

bool ChPlanetVisualMesh::Update(const ChVector3d& viewpoint, double time) {
    // The viewpoint on the planet, for the LOD
    const ChVector3d viewpoint_planet = ToPlanet(viewpoint);
    m_world->Update(viewpoint_planet, time);

    // Tiles easing in change shape every update until they are done.
    const auto version = m_world->GetMeshSetVersion();
    if (version == m_mesh_version && m_morphs.empty())
        return false;
    m_mesh_version = version;
    Rebuild(viewpoint_planet, time);
    return true;
}

void ChPlanetVisualMesh::Rebuild(const ChVector3d& viewpoint_planet, double time) {
    // A tile keeps its mesh id until rebuilt, so only tiles new since the last rebuild, and those easing in,
    // get a new shape; the rest keep theirs, and tiles no longer in the quadtree are dropped. A tile eases in
    // if a split built it after every tile drawn so far: not a parent coming back after a merge, which was
    // drawn before, nor any tile of the first update.
    std::unordered_map<std::uint64_t, std::shared_ptr<ChVisualShapeTriangleMesh>> tiles;
    auto model = m_body->GetVisualModel();
    model->Clear();
    m_num_triangles = 0;
    std::uint64_t last_id = m_last_id;
    for (const ChTileMesh* tile : m_world->GetMeshes()) {
        const ChVector3d center(tile->centerX, tile->centerY, tile->centerZ);
        if ((center - viewpoint_planet).Length() - tile->radius > m_max_distance)
            continue;
        last_id = std::max(last_id, tile->id);

        auto morphing = m_morphs.find(tile->id);
        if (morphing == m_morphs.end() && m_morph_time > 0 && m_num_rebuilds > 0 && tile->fromSplit && tile->id > m_last_id)
            morphing = m_morphs.emplace(tile->id, time).first;
        double morph = 1;
        if (morphing != m_morphs.end()) {
            morph = Ease((time - morphing->second) / m_morph_time);
            if (morph >= 1)
                m_morphs.erase(morphing);  // this last shape is final
        }

        auto cached = m_tiles.find(tile->id);
        std::shared_ptr<ChVisualShapeTriangleMesh> shape;
        if (cached != m_tiles.end() && morphing == m_morphs.end()) {
            shape = cached->second;
        } else {
            auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
            AppendTile(*tile, m_site, *mesh, morph);
            if (m_shape == Shape::SPHERE)
                PlaceOnSphere(*tile, *mesh, morph);
            // A tile with no compacted ground keeps a single material, like any other.
            const bool compacted = MarkCompaction(*tile, *mesh);
            CutSiteHoles(*mesh, m_holes);
            shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            shape->SetMesh(mesh);
            shape->SetMutable(false);  // never edited: a changed tile comes back as a new tile
            shape->AddMaterial(m_material);
            if (compacted)
                for (const auto& level : m_compaction_levels)
                    shape->AddMaterial(level.material);
        }
        // A tile wholly in a hole keeps its (empty) shape cached but is not drawn
        if (shape->GetMesh()->GetNumTriangles() > 0)
            model->AddShape(shape);
        m_num_triangles += shape->GetMesh()->GetNumTriangles();
        tiles.emplace(tile->id, std::move(shape));
    }
    m_tiles = std::move(tiles);
    m_num_tiles = m_tiles.size();
    m_last_id = last_id;
    for (auto it = m_morphs.begin(); it != m_morphs.end();)
        it = m_tiles.count(it->first) ? std::next(it) : m_morphs.erase(it);
    ++m_num_rebuilds;
}

void ChPlanetVisualMesh::AppendTile(const ChTileMesh& tile, const ChSiteFrame& site, ChTriangleMeshConnected& mesh, double morph) {
    const size_t n = tile.vertexData.size() / ChTileMesh::kFloatsPerVertex;
    auto& vertices = mesh.GetCoordsVertices();
    auto& normals = mesh.GetCoordsNormals();
    auto& uvs = mesh.GetCoordsUV();
    const int base = static_cast<int>(vertices.size());
    vertices.reserve(vertices.size() + n);
    normals.reserve(normals.size() + n);
    uvs.reserve(uvs.size() + n);

    for (size_t i = 0; i < n; ++i) {
        const float* data = &tile.vertexData[i * ChTileMesh::kFloatsPerVertex];
        const MorphedVertex v(data, morph);
        // Center-relative planet position, re-projected through the site frame so it matches the physics.
        const double px = tile.centerX + v.x;
        const double py = tile.centerY + v.y;
        const double pz = tile.centerZ + v.z;
        const util::LonLat ll = util::lonLatOf(px, py, pz);
        vertices.push_back(site.ToLocal(ll.lon, ll.lat, util::elevationOf(px, py, pz, site.GetRadius())));
        // Slopes are the normal's east and north components over its up component.
        normals.push_back(ChVector3d(v.slope_east, v.slope_north, 1.0).GetNormalized());
        uvs.push_back(ChVector2d(data[5], data[6]));
    }

    const auto& idx = ChTileMesh::GetIndices(tile.level);
    auto& faces = mesh.GetIndicesVertices();
    auto& face_normals = mesh.GetIndicesNormals();
    auto& face_uvs = mesh.GetIndicesUV();
    const size_t ntri = idx.size() / 3;
    faces.reserve(faces.size() + ntri);
    face_normals.reserve(face_normals.size() + ntri);
    face_uvs.reserve(face_uvs.size() + ntri);
    for (size_t t = 0; t < ntri; ++t) {
        const ChVector3i tri(base + static_cast<int>(idx[3 * t]), base + static_cast<int>(idx[3 * t + 1]), base + static_cast<int>(idx[3 * t + 2]));
        faces.push_back(tri);
        face_normals.push_back(tri);
        face_uvs.push_back(tri);
    }
}

}  // namespace planet
}  // namespace chrono
