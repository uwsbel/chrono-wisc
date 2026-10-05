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

#include "chrono_planet/visualization/ChMeshDrawsVSG.h"

#include <algorithm>
#include <stdexcept>

#include "chrono_vsg/utils/ChDataUtilsVSG.h"
#include "chrono_vsg/utils/ChShapeBuilderVSG.h"

namespace chrono {
namespace planet {

ChMeshDrawsVSG::ChMeshDrawsVSG(vsg3d::ChVisualSystemVSG* vsys, vsg::ref_ptr<vsg::Switch> scene) : m_vsys(vsys), m_scene(std::move(scene)) {
    if (!m_vsys || !m_scene)
        throw std::invalid_argument("ChMeshDrawsVSG: null visual system or scene");
}

vsg::ref_ptr<vsg::StateGroup> ChMeshDrawsVSG::StateFor(const std::shared_ptr<ChVisualMaterial>& material) {
    auto found = m_states.find(material.get());
    if (found != m_states.end())
        return found->second.second;

    // The visual system's own trimesh builder makes the state: build a one-triangle mesh with it, keep the state
    // group it makes and drop the triangle. The group's children are then the meshes drawn with that material.
    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    mesh->GetCoordsVertices() = {ChVector3d(0, 0, 0), ChVector3d(1, 0, 0), ChVector3d(0, 1, 0)};
    mesh->GetIndicesVertices() = {ChVector3i(0, 1, 2)};
    auto transform = vsg::MatrixTransform::create();
    auto graph = m_vsys->GetVSGShapeBuilder()->CreateTrimeshPbrMatShape(mesh, transform, {material}, true, m_wireframe);
    vsg::ref_ptr<vsg::StateGroup> state;
    for (auto& child : transform->children) {
        state = child.cast<vsg::StateGroup>();
        if (state)
            break;
    }
    if (!state)
        throw std::runtime_error("ChMeshDrawsVSG: unexpected layout of a VSG trimesh");
    state->children.clear();
    m_scene->addChild(m_visible, graph);
    m_states[material.get()] = {material, state};
    return state;
}

ChMeshDrawsVSG::Draws ChMeshDrawsVSG::Add(const ChTriangleMeshConnected& mesh, const std::vector<std::shared_ptr<ChVisualMaterial>>& materials) {
    Draws draws;
    const auto& vertices = mesh.GetCoordsVertices();
    const auto& normals = mesh.GetCoordsNormals();
    const auto& faces = mesh.GetIndicesVertices();
    const auto& face_normals = mesh.GetIndicesNormals();
    const auto& face_materials = mesh.GetIndicesMaterials();
    const bool smooth = face_normals.size() == faces.size();
    const bool by_face = face_materials.size() == faces.size();
    auto builder = m_vsys->GetVSGShapeBuilder();

    // One draw per material, its vertices laid out as the trimesh builder lays them out: positions, normals,
    // texture coordinates and colors, a vertex per face corner
    for (size_t m = 0; m < materials.size(); ++m) {
        std::vector<size_t> selected;
        for (size_t f = 0; f < faces.size(); ++f)
            if (by_face ? face_materials[f] == static_cast<int>(m) : m == 0)
                selected.push_back(f);
        if (selected.empty())
            continue;
        const size_t n = 3 * selected.size();
        auto positions = vsg::vec3Array::create(n);
        auto vertex_normals = vsg::vec3Array::create(n);
        auto texcoords = vsg::vec2Array::create(n);
        auto colors = vsg::vec4Array::create(n, vsg::vec4{1.0f, 1.0f, 1.0f, 1.0f});
        auto indices = vsg::uintArray::create(n);
        size_t k = 0;
        for (size_t f : selected) {
            const ChVector3i& face = faces[f];
            const ChVector3d flat = Vcross(vertices[face[1]] - vertices[face[0]], vertices[face[2]] - vertices[face[0]]).GetNormalized();
            for (int c = 0; c < 3; ++c, ++k) {
                positions->set(k, vsg::vec3CH(vertices[face[c]]));
                vertex_normals->set(k, vsg::vec3CH(smooth ? normals[face_normals[f][c]] : flat));
                texcoords->set(k, vsg::vec2(0.0f, 1.0f));
                indices->set(k, static_cast<unsigned int>(k));
            }
        }
        auto draw = vsg::VertexIndexDraw::create();
        draw->assignArrays(vsg::DataList{positions, vertex_normals, texcoords, colors});
        draw->assignIndices(indices);
        draw->indexCount = static_cast<uint32_t>(n);
        draw->instanceCount = 1;
        builder->CompileNode(draw);
        auto state = StateFor(materials[m]);
        state->addChild(draw);
        draws.push_back({state, draw});
    }
    return draws;
}

void ChMeshDrawsVSG::Remove(const Draws& draws) {
    for (const auto& [state, draw] : draws) {
        auto& children = state->children;
        children.erase(std::remove(children.begin(), children.end(), draw), children.end());
    }
}

void ChMeshDrawsVSG::Clear() {
    m_scene->children.clear();
    m_states.clear();
}

}  // namespace planet
}  // namespace chrono
