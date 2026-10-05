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

#include <cmath>
#include <iostream>
#include <vector>

#include "chrono/collision/ChCollisionShapeTriangleMesh.h"

#include "chrono_planet/ChSiteHoles.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"

#include "chrono_vehicle/terrain/PlanetTerrain.h"

#include "chrono/utils/ChConstants.h"

namespace chrono {
namespace vehicle {

PlanetTerrain::PlanetTerrain(ChSystem* system,
                             std::shared_ptr<const planet::ChPlanetSurface> surface,
                             const planet::ChSiteFrame& site)
    : m_system(system),
      m_surface(std::move(surface)),
      m_site(site),
      m_size(30.0),
      m_resolution(0.25),
      m_margin(4.0),
      m_friction(0.8f),
      m_soup(false),
      m_center(0, 0) {}

PlanetTerrain::~PlanetTerrain() {}

void PlanetTerrain::SetPatchSize(double size) {
    m_size = size;
}

void PlanetTerrain::SetPatchResolution(double resolution) {
    m_resolution = resolution;
}

void PlanetTerrain::SetRebuildMargin(double margin) {
    m_margin = margin;
}

void PlanetTerrain::Initialize(const ChVector2d& center) {
    const double max_margin = 0.25 * m_size;
    if (m_margin > max_margin) {
        std::cerr << "PlanetTerrain: rebuild margin " << m_margin << " m too large for a " << m_size
                  << " m patch; clamping to " << max_margin << " m" << std::endl;
        m_margin = max_margin;
    }
    if (!m_material) {
        m_material = ChContactMaterial::DefaultMaterial(m_system->GetContactMethod());
        m_material->SetFriction(m_friction);
    }
    RebuildPatch(center);
}

bool PlanetTerrain::UpdatePatch(const ChVector3d& loc) {
    const double half = m_size / 2.0 - m_margin;
    if (std::abs(loc.x() - m_center.x()) < half && std::abs(loc.y() - m_center.y()) < half)
        return false;
    RebuildPatch(ChVector2d(loc.x(), loc.y()));
    return true;
}

void PlanetTerrain::RebuildPatch(const ChVector2d& center) {
    m_center = center;

    const int n = static_cast<int>(std::round(m_size / m_resolution)) + 1;
    double center_lon, center_lat;
    m_site.ToLonLat(center.x(), center.y(), center_lon, center_lat);
    const double deg_per_m_lat = 180.0 / (CH_PI * m_site.GetRadius());
    const double deg_per_m_lon = deg_per_m_lat / std::cos(center_lat * CH_PI / 180.0);
    const double step_lat = m_resolution * deg_per_m_lat;
    const double step_lon = m_resolution * deg_per_m_lon;
    const double lon0 = center_lon - (n - 1) / 2.0 * step_lon;
    const double lat0 = center_lat - (n - 1) / 2.0 * step_lat;

    std::vector<double> h;
    m_surface->GetElevationGrid(lon0, lat0, step_lon, step_lat, n, h);

    std::vector<ChVector3d> verts(static_cast<size_t>(n) * n);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const size_t v = static_cast<size_t>(j) * n + i;
            verts[v] = m_site.ToLocal(lon0 + i * step_lon, lat0 + j * step_lat, h[v]);
        }

    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    mesh->GetCoordsVertices() = verts;
    auto& tris = mesh->GetIndicesVertices();
    tris.reserve(static_cast<size_t>(n - 1) * (n - 1) * 2);
    for (int j = 0; j < n - 1; ++j)
        for (int i = 0; i < n - 1; ++i) {
            const int a = j * n + i, b = a + 1, c = a + n, d = c + 1;
            tris.emplace_back(a, b, d);
            tris.emplace_back(a, d, c);
        }
    planet::CutSiteHoles(*mesh, m_holes);

    std::shared_ptr<ChTriangleMesh> coll_mesh = mesh;
    if (m_soup) {
        auto soup = chrono_types::make_shared<ChTriangleMeshSoup>();
        const auto& v = mesh->GetCoordsVertices();
        soup->GetTriangles().reserve(mesh->GetIndicesVertices().size());
        for (const auto& f : mesh->GetIndicesVertices())
            soup->AddTriangle(v[f[0]], v[f[1]], v[f[2]]);
        coll_mesh = soup;
    }

    auto body = chrono_types::make_shared<ChBody>();
    body->SetFixed(true);
    auto shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(m_material, coll_mesh, true, false, 0.005);
    body->AddCollisionShape(shape);
    body->EnableCollision(true);
    for (int family : m_disallowed_families)
        body->GetCollisionModel()->DisallowCollisionsWith(family);
    m_system->Add(body);
    // Collision models are bound only at the system's first step, so a body added later must be bound here.
    if (auto* coll = m_system->GetCollisionSystem().get())
        coll->BindItem(body);

    if (m_ground)
        m_system->Remove(m_ground);
    m_ground = body;
}

void PlanetTerrain::AddHole(const planet::ChSiteRegion& hole, std::function<double(double x, double y)> height) {
    m_holes.push_back(hole);
    m_hole_heights.push_back(std::move(height));
}

const std::function<double(double, double)>* PlanetTerrain::HoleHeight(double x, double y) const {
    for (size_t h = 0; h < m_holes.size(); ++h)
        if (m_hole_heights[h] && m_holes[h].Contains(x, y))
            return &m_hole_heights[h];
    return nullptr;
}

double PlanetTerrain::SurfaceHeight(double x, double y) const {
    if (const auto* height = HoleHeight(x, y))
        return (*height)(x, y);
    double lon, lat;
    m_site.ToLonLat(x, y, lon, lat);
    return m_surface->GetElevation(lon, lat) - m_site.GetOriginElevation();
}

double PlanetTerrain::GetHeight(const ChVector3d& loc) const {
    if (m_height_fun)
        return (*m_height_fun)(loc);
    return SurfaceHeight(loc.x(), loc.y());
}

ChVector3d PlanetTerrain::GetPoint(const ChVector3d& loc) const {
    if (m_point_fun)
        return (*m_point_fun)(loc);
    return ChVector3d(loc.x(), loc.y(), SurfaceHeight(loc.x(), loc.y()));
}

ChVector3d PlanetTerrain::GetNormal(const ChVector3d& loc) const {
    if (m_normal_fun)
        return (*m_normal_fun)(loc);
    // Central differences at the patch resolution, the slope the collision mesh actually presents.
    const double d = m_resolution;
    const double dzdx = (SurfaceHeight(loc.x() + d, loc.y()) - SurfaceHeight(loc.x() - d, loc.y())) / (2 * d);
    const double dzdy = (SurfaceHeight(loc.x(), loc.y() + d) - SurfaceHeight(loc.x(), loc.y() - d)) / (2 * d);
    ChVector3d normal(-dzdx, -dzdy, 1.0);
    normal.Normalize();
    return normal;
}

float PlanetTerrain::GetCoefficientFriction(const ChVector3d& loc) const {
    if (m_friction_fun)
        return (*m_friction_fun)(loc);
    return m_friction;
}

}  // end namespace vehicle
}  // end namespace chrono
