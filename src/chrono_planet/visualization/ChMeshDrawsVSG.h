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
// Triangle meshes added to and removed from a VSG scene as they change, for
// plugins drawing geometry that is rebuilt while the simulation runs.
//
// =============================================================================

#ifndef CH_MESH_DRAWS_VSG_H
#define CH_MESH_DRAWS_VSG_H

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_planet/ChApiPlanet.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Draws triangle meshes that come and go under a VSG switch node. The VSG visual system binds a body's shapes
/// once and builds pipeline state for every shape it binds, too slowly for geometry rebuilt every frame; here every
/// mesh of a material shares one set of pipeline state, built on first use, so adding a mesh costs only the upload
/// of its vertices. Use from a plugin's OnRender: state built while the visual system is initialized is not
/// compiled.
class CH_PLANET_API ChMeshDrawsVSG {
  public:
    /// The draws of one mesh, one per material its faces use.
    using Draws = std::vector<std::pair<vsg::ref_ptr<vsg::StateGroup>, vsg::ref_ptr<vsg::Node>>>;

    ChMeshDrawsVSG(vsg3d::ChVisualSystemVSG* vsys, vsg::ref_ptr<vsg::Switch> scene);

    /// Draw a mesh in the system's frame, its faces with the materials their material indices pick (all with the
    /// first if it has none). Returns its draws, for Remove.
    Draws Add(const ChTriangleMeshConnected& mesh, const std::vector<std::shared_ptr<ChVisualMaterial>>& materials);

    /// Stop drawing a mesh.
    void Remove(const Draws& draws);

    /// Remove every mesh, and the state built for them.
    void Clear();

    /// Build state for wireframe drawing from now on (default: false); Clear to apply it to meshes drawn so far.
    void SetWireframe(bool val) { m_wireframe = val; }

    /// Show or hide what is drawn with state built from now on.
    void SetVisible(bool val) { m_visible = val; }

  private:
    vsg::ref_ptr<vsg::StateGroup> StateFor(const std::shared_ptr<ChVisualMaterial>& material);

    vsg3d::ChVisualSystemVSG* m_vsys;
    vsg::ref_ptr<vsg::Switch> m_scene;
    std::unordered_map<const ChVisualMaterial*, std::pair<std::shared_ptr<ChVisualMaterial>, vsg::ref_ptr<vsg::StateGroup>>> m_states;
    bool m_wireframe = false;
    bool m_visible = true;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
