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
// VSG plugin drawing the quadtree terrain around the camera in a site frame.
//
// =============================================================================

#ifndef CH_PLANET_VISUALIZATION_VSG_H
#define CH_PLANET_VISUALIZATION_VSG_H

#include <cstdint>
#include <memory>
#include <vector>
#include <unordered_map>

#include "chrono/assets/ChColormap.h"
#include "chrono/assets/ChVisualMaterial.h"

#include "chrono_vsg/ChGuiComponentVSG.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/filters/ChDeformationFilter.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// VSG plugin that mirrors the quadtree's resident tiles into the scene.
/// Each frame the level of detail is stepped at the camera position on simulation time, and every
/// tile mesh the world holds is re-projected through the site frame, so the drawn ground is the
/// surface that vehicle::PlanetTerrain and vehicle::PlanetSCMTerrain put under the wheels. The site frame
/// must be on the same body as the quadtree's surface. Tiles use the visual system's
/// PBR pipeline with one material.
class CH_PLANET_API ChPlanetVisualizationVSG : public vsg3d::ChVisualSystemVSGPlugin {
  public:
    /// Construct the plugin over a quadtree, drawn in the given site frame.
    ChPlanetVisualizationVSG(std::shared_ptr<ChPlanetQuadtree> world, const ChSiteFrame& site);
    ~ChPlanetVisualizationVSG();

    /// Set the material every tile is drawn with (default: a matte neutral grey).
    void SetMaterial(std::shared_ptr<ChVisualMaterial> material) { m_material = material; }

    /// Draw tiles as wireframe (default: false). Can be toggled at any time; the tiles in the scene are
    /// rebuilt on the next frame.
    void SetWireframe(bool val) { m_wireframe_requested = val; }
    bool GetWireframe() const { return m_wireframe_requested; }

    /// Color the terrain by the depth of a deformation filter's changes, like SCM's sinkage plot: ground
    /// lowered by `range` (m) or more gets the top color of the colormap, undisturbed ground keeps the
    /// material color. Ruts a few centimeters deep are hard to see by shading alone; this makes them
    /// obvious. Tiles are drawn with vertex colors while it is on. Pass null to turn it off.
    void SetDeformationColoring(std::shared_ptr<ChDeformationFilter> filter,
                                double range = 0.05,
                                ChColormap::Type colormap = ChColormap::Type::JET);

    /// Turn deformation coloring on or off, keeping the filter (applied on the next frame).
    void EnableDeformationColoring(bool val) { m_coloring_requested = val; }
    bool IsDeformationColoringEnabled() const { return m_coloring_requested; }
    bool HasDeformationColoring() const { return m_deformation != nullptr; }

    /// Leave the terrain out of a rectangle of site x/y, cutting the faces that cross its edges, where other ground
    /// takes over, such as a work site modeled on its own. The tiles are rebuilt on the next frame.
    void AddHole(const ChSiteRegion& hole) {
        m_holes.push_back(hole);
        m_holes_changed = true;
    }

    /// Show a small "Planet terrain" panel with a wireframe checkbox and tile counts (default: true).
    /// Call before attaching the plugin to have it take effect from the first frame.
    void SetShowGui(bool val);

    /// Show or hide the terrain.
    void SetVisible(bool val);

    /// Number of tiles currently in the scene.
    size_t GetNumTiles() const { return m_tiles.size(); }

    /// The quadtree this plugin drives.
    std::shared_ptr<ChPlanetQuadtree> GetWorld() const { return m_world; }

    virtual void OnAttach() override;
    virtual void OnBindAssets() override;
    virtual void OnRender() override;

  private:
    class GuiVSG;

    /// Build a VSG subgraph for one tile mesh, in site coordinates.
    vsg::ref_ptr<vsg::Node> BuildTile(const ChTileMesh& mesh) const;

    /// Add and drop tiles so the scene matches the world's resident meshes.
    void SyncTiles();

    std::shared_ptr<ChPlanetQuadtree> m_world;
    ChSiteFrame m_site;
    std::shared_ptr<ChVisualMaterial> m_material;
    bool m_wireframe;            ///< wireframe state of the tiles in the scene
    bool m_wireframe_requested;  ///< wireframe state asked for, applied on the next frame
    bool m_visible;
    bool m_show_gui;
    std::shared_ptr<ChDeformationFilter> m_deformation;  ///< coloring source, if any
    double m_color_range;                               ///< depth (m) that gets the top color
    std::unique_ptr<ChColormap> m_colormap;
    bool m_coloring;             ///< coloring state of the tiles in the scene
    bool m_coloring_requested;   ///< coloring state asked for, applied on the next frame
    std::vector<ChSiteRegion> m_holes;  ///< rectangles of site x/y left out
    bool m_holes_changed = false;       ///< holes added since the tiles in the scene were built
    std::shared_ptr<vsg3d::ChGuiComponentVSG> m_gui;

    vsg::ref_ptr<vsg::Switch> m_terrain_scene;  ///< VSG scene holding the tiles
    std::unordered_map<std::uint64_t, vsg::ref_ptr<vsg::Node>> m_tiles;  ///< tiles in the scene, by mesh id
    unsigned long long m_mesh_version;          ///< world mesh set version last mirrored
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
