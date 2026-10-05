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
// Quadtree planetary terrain in a VSG window: a ball drops onto a rigid
// PlanetTerrain patch while the ChPlanetVisualizationVSG plugin streams the
// quadtree tiles around the camera, on the Moon preset. GeoTIFF paths on the
// command line form the DEM stack; with none, the Moon DEM resources shipped
// with Chrono (global low resolution plus the Apollo 17 landing site) are used.
//
// =============================================================================

#include <iostream>
#include <vector>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"
#include "chrono_planet/planets/moon/ChMoon.h"

#include "PlanetDemoSetup.h"
#include "chrono_planet/visualization/ChPlanetVisualizationVSG.h"

#include "chrono_vehicle/terrain/PlanetTerrain.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::planet;
using namespace chrono::vehicle;

// Landing site (Apollo 17 region) and the quadtree zoom the physics surface is sampled at.
const double site_lon = 30.75;
const double site_lat = 20.19;
const int zoom = 15;

// Quadtree root tile size (deg) and ring half-width (tiles), as the interactive renderer uses.
const double root_tile_deg = 16.0;
const int view_range_tiles = 10;

const double step_size = 1e-3;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // GeoTIFF paths from the command line.
    std::vector<ChGeoTiffSource> dems;
    for (int i = 1; i < argc; ++i)
        dems.push_back({argv[i], 0, 30});

    // One surface for physics and rendering, so the wheels ride the drawn ground.
    auto surface = CreateDemoSurface(dems, zoom, root_tile_deg);
    if (!surface)
        return 1;
    const ChSiteFrame site = surface->MakeSiteFrame(site_lon, site_lat);
    auto world = chrono_types::make_shared<ChPlanetQuadtree>(surface, view_range_tiles);

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -surface->GetBody().GetSurfaceGravity()));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    PlanetTerrain terrain(&sys, surface, site);
    terrain.SetPatchSize(20.0);
    terrain.SetPatchResolution(0.25);
    terrain.SetRebuildMargin(4.0);
    terrain.Initialize();

    const double radius = 0.3;
    auto material = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());
    auto ball = chrono_types::make_shared<ChBodyEasySphere>(radius, 1000, true, true, material);
    ball->SetPos(ChVector3d(0, 0, terrain.GetHeight(ChVector3d(0, 0, 0)) + 1.5));
    ball->SetPosDt(ChVector3d(0.5, 0, 0));
    sys.Add(ball);

    auto vis_planet = chrono_types::make_shared<ChPlanetVisualizationVSG>(world, site);

    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->AttachPlugin(vis_planet);
    vis->SetWindowTitle("Planet terrain");
    vis->SetWindowSize(1280, 800);
    vis->SetWindowPosition(100, 100);
    vis->AddCamera(ChVector3d(-8, -8, 4), ball->GetPos());
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.2, 0.5);
    vis->Initialize();

    int step = 0;
    while (vis->Run()) {
        vis->BeginScene();
        vis->SetCameraTarget(ball->GetPos());
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(step_size);
        terrain.UpdatePatch(ball->GetPos());

        if (++step % 300 == 0)
            std::cout << "t = " << sys.GetChTime() << " s, ball z = " << ball->GetPos().z() << " m, tiles in scene: "
                      << vis_planet->GetNumTiles() << std::endl;
    }

    std::cout << "Tiles in scene at exit: " << vis_planet->GetNumTiles() << std::endl;
    return 0;
}
