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
//
// VSG plugin drawing a triangle mesh that is replaced as the simulation runs.
//
// =============================================================================

#ifndef CH_MESH_VISUALIZATION_VSG_H
#define CH_MESH_VISUALIZATION_VSG_H

#include <memory>
#include <mutex>

#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/visualization/ChMeshDrawsVSG.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// VSG plugin that draws a triangle mesh, in world coordinates, replaced whenever SetMesh gives a new one, such as the
/// loose soil of vehicle::PlanetCRMWindow.
class CH_PLANET_API ChMeshVisualizationVSG : public vsg3d::ChVisualSystemVSGPlugin {
  public:
    explicit ChMeshVisualizationVSG(std::shared_ptr<ChVisualMaterial> material);
    ~ChMeshVisualizationVSG();

    /// The mesh to draw from the next frame on (null or empty draws nothing). The mesh must not change afterwards.
    void SetMesh(std::shared_ptr<const ChTriangleMeshConnected> mesh);

    /// Show or hide the mesh.
    void SetVisible(bool val);

    virtual void OnBindAssets() override;
    virtual void OnRender() override;

  private:
    std::shared_ptr<ChVisualMaterial> m_material;
    std::mutex m_mutex;
    std::shared_ptr<const ChTriangleMeshConnected> m_mesh;
    bool m_changed = false;
    vsg::ref_ptr<vsg::Switch> m_scene;
    std::unique_ptr<ChMeshDrawsVSG> m_draws;
    ChMeshDrawsVSG::Draws m_current;
    bool m_visible = true;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
