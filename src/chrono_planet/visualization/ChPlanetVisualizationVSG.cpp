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

#include <algorithm>
#include <unordered_set>

#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_vsg/utils/ChShapeBuilderVSG.h"

#include "chrono_planet/ChPlanetVisualMesh.h"
#include "chrono_planet/ChSiteHoles.h"
#include "chrono_planet/core/SphereMath.h"
#include "chrono_planet/visualization/ChPlanetVisualizationVSG.h"

namespace chrono {
namespace planet {

// The plugin's panel: a wireframe toggle and what the quadtree holds.
class ChPlanetVisualizationVSG::GuiVSG : public vsg3d::ChGuiComponentVSG {
  public:
    explicit GuiVSG(ChPlanetVisualizationVSG* plugin) : m_plugin(plugin) {}

    virtual void render(vsg::CommandBuffer& cb) override {
        // Top right, below the logo, clear of the visual system's own panel at the top left. Placed once the
        // display has a size (it is zero on the first frame), then left where the user moves it.
        const ImVec2 display = ImGui::GetIO().DisplaySize;
        if (!m_placed && display.x > 0) {
            ImGui::SetNextWindowPos(ImVec2(display.x - 10.0f, 100.0f), ImGuiCond_Always, ImVec2(1.0f, 0.0f));
            m_placed = true;
        }
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin("Planet terrain");
        bool wireframe = m_plugin->GetWireframe();
        if (ImGui::Checkbox("Wireframe", &wireframe))
            m_plugin->SetWireframe(wireframe);
        if (m_plugin->HasDeformationColoring()) {
            bool coloring = m_plugin->IsDeformationColoringEnabled();
            if (ImGui::Checkbox("Color by deformation", &coloring))
                m_plugin->EnableDeformationColoring(coloring);
        }
        ImGui::Text("Tiles in scene: %zu", m_plugin->GetNumTiles());
        ImGui::Text("Tiles rebuilt for changes: %lld", m_plugin->GetWorld()->GetNumRebuiltTiles());
        ImGui::End();
    }

  private:
    ChPlanetVisualizationVSG* m_plugin;
    bool m_placed = false;
};

ChPlanetVisualizationVSG::ChPlanetVisualizationVSG(std::shared_ptr<ChPlanetQuadtree> world, const ChSiteFrame& site)
    : m_world(std::move(world)),
      m_site(site),
      m_wireframe(false),
      m_wireframe_requested(false),
      m_visible(true),
      m_show_gui(true),
      m_color_range(0.05),
      m_coloring(false),
      m_coloring_requested(false),
      m_mesh_version(~0ull) {
    m_material = chrono_types::make_shared<ChVisualMaterial>();
    m_material->SetDiffuseColor(ChColor(0.42f, 0.41f, 0.39f));
    m_material->SetRoughness(0.95f);
    m_material->SetMetallic(0.0f);
}

ChPlanetVisualizationVSG::~ChPlanetVisualizationVSG() {}

void ChPlanetVisualizationVSG::OnAttach() {
    m_vsys->SetCameraVertical(CameraVerticalDir::Z);
    m_gui = chrono_types::make_shared<GuiVSG>(this);
    m_gui->SetVisibility(m_show_gui);
    m_vsys->AddGuiComponent(m_gui);
}

void ChPlanetVisualizationVSG::SetDeformationColoring(std::shared_ptr<ChDeformationFilter> filter,
                                                      double range,
                                                      ChColormap::Type colormap) {
    m_deformation = std::move(filter);
    m_color_range = range > 0 ? range : 0.05;
    m_colormap = m_deformation ? std::make_unique<ChColormap>(colormap) : nullptr;
    m_coloring_requested = m_deformation != nullptr;
}

void ChPlanetVisualizationVSG::SetShowGui(bool val) {
    m_show_gui = val;
    if (m_gui)
        m_gui->SetVisibility(val);
}

void ChPlanetVisualizationVSG::OnBindAssets() {
    m_terrain_scene = vsg::Switch::create();
    m_vsys->GetVSGScene()->addChild(m_terrain_scene);
}

void ChPlanetVisualizationVSG::OnRender() {
    // The camera in site coordinates, mapped onto the sphere for the LOD.
    const ChVector3d cam = m_vsys->GetCameraPosition();
    double lon, lat;
    m_site.ToLonLat(cam.x(), cam.y(), lon, lat);
    const double radius = m_site.GetRadius() + m_site.GetOriginElevation() + cam.z();
    const util::Vec3 p = util::pointOnSphere(lon, lat, radius);
    m_world->Update(ChVector3d(p.x, p.y, p.z), m_vsys->GetSimulationTime());

    SyncTiles();
}

void ChPlanetVisualizationVSG::SetVisible(bool val) {
    m_visible = val;
    if (m_terrain_scene)
        m_terrain_scene->setAllChildren(val);
}

void ChPlanetVisualizationVSG::SyncTiles() {
    // A wireframe or coloring toggle rebuilds every tile in the scene with the new setting.
    const bool coloring = m_coloring_requested && m_deformation;
    if (m_wireframe_requested != m_wireframe || coloring != m_coloring || m_holes_changed) {
        m_holes_changed = false;
        m_wireframe = m_wireframe_requested;
        m_coloring = coloring;
        m_terrain_scene->children.clear();
        m_tiles.clear();
        m_mesh_version = ~0ull;
    }

    const auto version = m_world->GetMeshSetVersion();
    if (version == m_mesh_version)
        return;
    m_mesh_version = version;

    // A rebuilt tile comes back with a new id, so tiles are matched by mesh id alone.
    const auto live = m_world->GetMeshes();
    std::unordered_set<std::uint64_t> live_ids;
    for (const ChTileMesh* mesh : live)
        live_ids.insert(mesh->id);

    // Drop tiles whose mesh is gone.
    auto& children = m_terrain_scene->children;
    for (auto it = m_tiles.begin(); it != m_tiles.end();) {
        if (live_ids.count(it->first)) {
            ++it;
            continue;
        }
        const auto node = it->second;
        children.erase(std::remove_if(children.begin(), children.end(),
                                      [&](const vsg::Switch::Child& c) { return c.node == node; }),
                       children.end());
        it = m_tiles.erase(it);
    }

    // Add tiles the quadtree holds that the scene does not.
    for (const ChTileMesh* mesh : live) {
        if (m_tiles.count(mesh->id))
            continue;
        auto node = BuildTile(*mesh);
        m_terrain_scene->addChild(m_visible, node);
        m_tiles[mesh->id] = node;
    }
}

vsg::ref_ptr<vsg::Node> ChPlanetVisualizationVSG::BuildTile(const ChTileMesh& mesh) const {
    const size_t n = mesh.vertexData.size() / ChTileMesh::kFloatsPerVertex;

    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    ChPlanetVisualMesh::AppendTile(mesh, m_site, *trimesh);

    auto transform = vsg::MatrixTransform::create();
    if (!m_coloring) {
        CutSiteHoles(*trimesh, m_holes);
        if (trimesh->GetIndicesVertices().empty())
            return vsg::Group::create();  // a tile wholly in a hole
        return m_vsys->GetVSGShapeBuilder()->CreateTrimeshPbrMatShape(trimesh, transform, {m_material}, true, m_wireframe);
    }

    // Vertex colors by deformation depth, at this tile's relief spacing so coarse tiles stay plain.
    const double spacing = m_world->GetSurface()->GetSampleSpacingAtZoom(mesh.level);
    const ChColor base = m_material->GetDiffuseColor();
    auto& colors = trimesh->GetCoordsColors();
    colors.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const float* v = &mesh.vertexData[i * ChTileMesh::kFloatsPerVertex];
        const util::LonLat ll = util::lonLatOf(mesh.centerX + v[0], mesh.centerY + v[1], mesh.centerZ + v[2]);
        const double depth = -m_deformation->Apply(ll.lon, ll.lat, spacing, 0.0);
        colors.push_back(depth > 1e-3 ? m_colormap->Get(depth, 0.0, m_color_range) : base);
    }
    CutSiteHoles(*trimesh, m_holes);
    if (trimesh->GetIndicesVertices().empty())
        return vsg::Group::create();
    return m_vsys->GetVSGShapeBuilder()->CreateTrimeshColAvgShape(trimesh, transform, base, true, m_wireframe);
}

}  // namespace planet
}  // namespace chrono
