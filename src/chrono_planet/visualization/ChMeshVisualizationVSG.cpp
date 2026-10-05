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

#include "chrono_planet/visualization/ChMeshVisualizationVSG.h"

#include <stdexcept>

namespace chrono {
namespace planet {

ChMeshVisualizationVSG::ChMeshVisualizationVSG(std::shared_ptr<ChVisualMaterial> material) : m_material(std::move(material)) {
    if (!m_material)
        throw std::invalid_argument("ChMeshVisualizationVSG: null material");
}

ChMeshVisualizationVSG::~ChMeshVisualizationVSG() {}

void ChMeshVisualizationVSG::SetMesh(std::shared_ptr<const ChTriangleMeshConnected> mesh) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_mesh = std::move(mesh);
    m_changed = true;
}

void ChMeshVisualizationVSG::OnBindAssets() {
    m_scene = vsg::Switch::create();
    m_vsys->GetVSGScene()->addChild(m_scene);
    m_draws = std::make_unique<ChMeshDrawsVSG>(m_vsys, m_scene);
}

void ChMeshVisualizationVSG::SetVisible(bool val) {
    m_visible = val;
    if (m_draws)
        m_draws->SetVisible(val);
    if (m_scene)
        m_scene->setAllChildren(val);
}

void ChMeshVisualizationVSG::OnRender() {
    std::shared_ptr<const ChTriangleMeshConnected> mesh;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_draws || !m_changed)
            return;
        m_changed = false;
        mesh = m_mesh;
    }
    m_draws->Remove(m_current);
    m_current.clear();
    if (mesh && mesh->GetNumTriangles() > 0)
        m_current = m_draws->Add(*mesh, {m_material});
}

}  // namespace planet
}  // namespace chrono
