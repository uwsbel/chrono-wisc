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
// Demo helper: boulders from the surface's procedural rock layer, placed
// as fixed rigid bodies in a site frame. Each rock is a pure function of its
// longitude and latitude, so the bodies created here stand exactly where the
// quadtree renderer draws them and sit on the plinth the ChPlanetSurface
// already carries. Rocks are added as a tracked position moves; since the
// bodies are fixed, nothing is ever removed.
//
// =============================================================================

#ifndef DEMO_PLANET_BOULDER_FIELD_H
#define DEMO_PLANET_BOULDER_FIELD_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <unordered_set>
#include <vector>

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/collision/ChCollisionShapeConvexHull.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/procedural/ChRockLayer.h"
#include "chrono_planet/geometry/RockMeshes.h"

/// Procedural boulders as fixed collision bodies around a moving position.
class PlanetBoulderField {
  public:
    /// Construct over a surface in a site frame. Rocks smaller than min_radius (m) are skipped.
    PlanetBoulderField(chrono::ChSystem* sys,
                       std::shared_ptr<const chrono::planet::ChPlanetSurface> surface,
                       const chrono::planet::ChSiteFrame& site,
                       std::shared_ptr<chrono::ChContactMaterial> material,
                       double min_radius)
        : m_sys(sys),
          m_surface(std::move(surface)),
          m_site(site),
          m_rocks(m_surface->FindFilter<chrono::planet::ChRockLayer>()),
          m_material(std::move(material)),
          m_min_radius(min_radius) {
        m_color = chrono::ChColor(0.36f, 0.34f, 0.32f);
    }

    /// Set the color of rocks created after this call (default: a dark regolith grey).
    void SetColor(const chrono::ChColor& color) { m_color = color; }

    /// Set a visual material for rocks created after this call, in place of the color (for example a Hapke
    /// material for Chrono::Sensor cameras). Null goes back to the color.
    void SetVisualMaterial(std::shared_ptr<chrono::ChVisualMaterial> material) { m_visual_material = std::move(material); }

    /// Set the collision family of the rocks and the family they ignore (defaults: kRockFamily, kTerrainFamily).
    /// Applies to rocks created after this call. The caller assigns terrain_family to the ground body.
    void SetCollisionFamilies(int rock_family, int terrain_family) {
        m_family = rock_family;
        m_ignore_family = terrain_family;
    }

    static constexpr int kRockFamily = 2;     ///< default collision family of the rocks
    static constexpr int kTerrainFamily = 1;  ///< default family the rocks do not collide with

    /// Add every rock within radius meters of center (site x/y) that is not already in the system.
    /// New bodies are bound to vis if given, so they appear in an already-initialized visual system.
    /// Returns the number of rocks added.
    int Update(const chrono::ChVector3d& center, double radius, chrono::ChVisualSystem* vis = nullptr) {
        double lon, lat;
        m_site.ToLonLat(center.x(), center.y(), lon, lat);

        // Bounding lon/lat rectangle of the disc, padded by the largest rock so nothing on the rim is missed.
        if (!m_rocks)
            return 0;  // the surface has no rock layer
        const double reach = radius + m_rocks->GetMaxRadius();
        const double dlat = reach / m_surface->GetBody().GetMetersPerDegree();
        const double dlon = dlat / std::max(std::cos(lat * chrono::CH_PI / 180.0), 0.05);
        const auto rocks = m_rocks->Query(lon - dlon, lat - dlat, lon + dlon, lat + dlat, m_min_radius);

        int added = 0;
        for (const auto& rock : rocks) {
            if (m_ids.count(rock.id))
                continue;
            const chrono::ChVector3d p = m_site.ToLocal(rock.lonDeg, rock.latDeg, 0.0);
            if ((chrono::ChVector2d(p.x(), p.y()) - chrono::ChVector2d(center.x(), center.y())).Length() > radius)
                continue;
            auto body = MakeBoulder(rock);
            m_sys->AddBody(body);
            if (vis)
                vis->BindItem(body);
            m_ids.insert(rock.id);
            m_bodies.push_back(body);
            m_largest = std::max(m_largest, static_cast<double>(rock.radiusM));
            ++added;
        }
        return added;
    }

    size_t GetNumRocks() const { return m_bodies.size(); }  ///< rocks in the system
    double GetLargestRadius() const { return m_largest; }   ///< largest rock so far (m)
    const std::vector<std::shared_ptr<chrono::ChBody>>& GetBodies() const { return m_bodies; }  ///< rock bodies

  private:
    /// A fixed body carrying the rock's finest mesh for drawing and its coarsest mesh as a convex hull for contact.
    std::shared_ptr<chrono::ChBody> MakeBoulder(const chrono::planet::ChRockInstance& rock) const {
        const chrono::planet::RockMesh& mesh = chrono::planet::RockMeshes::get(rock.meshId);
        const double r = rock.radiusM;

        // Orientation in the local ENU frame, which is the site frame. the rock layer returns (x, y, z, w).
        const auto q = chrono::planet::ChRockLayer::GetOrientation(rock);
        const chrono::ChQuaterniond rot(q[3], q[0], q[1], q[2]);

        // Rest the rock on the surface (which includes its plinth) and bury it by its bury fraction.
        const auto ext = chrono::planet::RockMeshes::extentsZ(rock.meshId, q);
        const double ground = m_surface->GetElevation(rock.lonDeg, rock.latDeg) - m_site.GetOriginElevation();
        const double rise = chrono::planet::ChRockLayer::GetCenterRise(rock, ext.bottomExtent * r, ext.totalHeight * r);
        chrono::ChVector3d pos = m_site.ToLocal(rock.lonDeg, rock.latDeg, 0.0);
        pos.z() = ground + rise;

        auto body = chrono_types::make_shared<chrono::ChBody>();
        body->SetName("boulder_" + std::to_string(rock.id));
        body->SetPos(pos);
        body->SetRot(rot);
        body->SetFixed(true);

        // Visual: the finest LOD, scaled to the rock's size.
        auto trimesh = chrono_types::make_shared<chrono::ChTriangleMeshConnected>();
        const size_t nv = mesh.vertices.size() / chrono::planet::RockMesh::kVertexStride;
        auto& verts = trimesh->GetCoordsVertices();
        auto& norms = trimesh->GetCoordsNormals();
        verts.reserve(nv);
        norms.reserve(nv);
        for (size_t i = 0; i < nv; ++i) {
            const float* v = &mesh.vertices[i * chrono::planet::RockMesh::kVertexStride];
            verts.emplace_back(v[0] * r, v[1] * r, v[2] * r);
            norms.emplace_back(v[3], v[4], v[5]);
        }
        const chrono::planet::RockMesh::Lod& fine = mesh.lod[0];
        auto& faces = trimesh->GetIndicesVertices();
        auto& face_normals = trimesh->GetIndicesNormals();
        for (std::uint32_t t = fine.first; t + 2 < fine.first + fine.count; t += 3) {
            const chrono::ChVector3i tri(static_cast<int>(mesh.indices[t]), static_cast<int>(mesh.indices[t + 1]),
                                         static_cast<int>(mesh.indices[t + 2]));
            faces.push_back(tri);
            face_normals.push_back(tri);
        }
        auto shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
        shape->SetMesh(trimesh, false);
        shape->SetMutable(false);
        if (m_visual_material)
            shape->AddMaterial(m_visual_material);
        else
            shape->SetColor(m_color);
        body->AddVisualShape(shape);

        // Collision: the convex hull of the coarsest LOD's vertices, plenty for a rounded boulder.
        const chrono::planet::RockMesh::Lod& coarse = mesh.lod[chrono::planet::RockMesh::kLods - 1];
        std::unordered_set<std::uint32_t> used;
        std::vector<chrono::ChVector3d> hull;
        for (std::uint32_t t = coarse.first; t < coarse.first + coarse.count; ++t)
            if (used.insert(mesh.indices[t]).second)
                hull.push_back(verts[mesh.indices[t]]);
        auto coll = chrono_types::make_shared<chrono::ChCollisionShapeConvexHull>(m_material, hull);
        body->AddCollisionShape(coll);
        body->EnableCollision(true);
        body->GetCollisionModel()->SetFamily(m_family);
        body->GetCollisionModel()->DisallowCollisionsWith(m_family);  // fixed rocks never touch each other
        body->GetCollisionModel()->DisallowCollisionsWith(m_ignore_family);

        return body;
    }

    chrono::ChSystem* m_sys;
    std::shared_ptr<const chrono::planet::ChPlanetSurface> m_surface;
    chrono::planet::ChSiteFrame m_site;
    std::shared_ptr<chrono::planet::ChRockLayer> m_rocks;  ///< the surface's rock layer, if any
    std::shared_ptr<chrono::ChContactMaterial> m_material;
    double m_min_radius;
    chrono::ChColor m_color;
    std::shared_ptr<chrono::ChVisualMaterial> m_visual_material;
    int m_family = kRockFamily;
    int m_ignore_family = kTerrainFamily;

    std::unordered_set<std::uint64_t> m_ids;
    std::vector<std::shared_ptr<chrono::ChBody>> m_bodies;
    double m_largest = 0.0;
};

#endif
