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
// Headless demonstration of the planetary terrain models: a rigid body settles
// on a PlanetTerrain patch that follows it, then a wheel sinks into a
// PlanetSCMTerrain, on the Moon preset of the Planet module. With no DEM paths
// given the surface is the Moon's procedural relief alone, so the demo needs no
// data files; pass GeoTIFF paths on the command line to run over a real
// elevation model.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <chrono>
#include <optional>
#include <thread>

#include "chrono/utils/ChOpenMP.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/planets/moon/ChMoon.h"

#include "chrono_vehicle/terrain/PlanetSCMTerrain.h"
#include "chrono_vehicle/terrain/PlanetTerrain.h"

using namespace chrono;
using namespace chrono::planet;
using namespace chrono::vehicle;

// Landing site (Apollo 17 region) and the quadtree zoom the surface is sampled at.
const double site_lon = 30.75;
const double site_lat = 20.19;
const int zoom = 15;

const double gravity = moon::kGravity;
const double step_size = 1e-3;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // DEM stack from the command line: each argument is a GeoTIFF serving all zooms.
    std::vector<ChGeoTiffSource> dems;
    for (int i = 1; i < argc; ++i)
        dems.push_back({argv[i], 0, 30});

    std::shared_ptr<ChPlanetSurface> surface;
    try {
        surface = moon::CreateSurface(dems, zoom);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    const ChSiteFrame site = surface->MakeSiteFrame(site_lon, site_lat);
    std::cout << "Site origin elevation: " << site.GetOriginElevation() << " m" << std::endl;

    // ---------------------------------------------------------------
    // Rigid patch: a sphere settles on the terrain, then walks off the patch
    // ---------------------------------------------------------------
    {
        ChSystemNSC sys;
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

        PlanetTerrain terrain(&sys, surface, site);
        terrain.SetPatchSize(20.0);
        terrain.SetPatchResolution(0.25);
        terrain.SetRebuildMargin(4.0);
        terrain.Initialize();

        const double radius = 0.3;
        auto material = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());
        auto ball = chrono_types::make_shared<ChBodyEasySphere>(radius, 1000, true, true, material);
        ball->SetPos(ChVector3d(0, 0, terrain.GetHeight(ChVector3d(0, 0, 0)) + 1.0));
        sys.Add(ball);

        while (sys.GetChTime() < 3.0)
            sys.DoStepDynamics(step_size);

        const double ground = terrain.GetHeight(ball->GetPos());
        std::cout << "Rigid: ball rests at z = " << ball->GetPos().z() << " m, terrain " << ground
                  << " m, gap " << ball->GetPos().z() - ground - radius << " m" << std::endl;

        int rebuilds = 0;
        for (double x = 0; x <= 30.0; x += 1.0)
            if (terrain.UpdatePatch(ChVector3d(x, 0, 0)))
                ++rebuilds;
        std::cout << "Rigid: " << rebuilds << " patch rebuilds over 30 m, patch now centered at x = "
                  << terrain.GetPatchCenter().x() << " m" << std::endl;
    }

    // ---------------------------------------------------------------
    // SCM: a wheel drops onto deformable regolith
    // ---------------------------------------------------------------
    {
        ChSystemNSC sys;
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        // SCM ray-casts every wheel's grid nodes in parallel on Chrono's threads, and it dominates the step.
        // Gains flatten past about 8 threads.
        sys.SetNumThreads(std::min(8, ChOMP::GetNumProcs()), 1, 4);

        const double radius = 0.25;
        const double width = 0.2;
        // SCM finds the wheel by ray-casting the collision system, so the wheel needs a collision shape.
        auto wheel_mat = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());
        auto wheel = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, radius, width, 2000, true, true, wheel_mat);
        wheel->SetPos(ChVector3d(0, 0, surface->GetElevation(site_lon, site_lat) - site.GetOriginElevation() + 0.5));
        sys.Add(wheel);

        PlanetSCMTerrain terrain(&sys, surface, site);
        PlanetSCMTerrain::Params params;
        params.delta = 0.05;
        terrain.Initialize(params, {{wheel, radius, width}});

        while (sys.GetChTime() < 2.0)
            sys.DoStepDynamics(step_size);

        terrain.RequestRutStats();
        // The summary is computed on a worker; poll briefly for it.
        std::optional<PlanetSCMTerrain::RutStats> stats;
        for (int i = 0; i < 1000 && !(stats = terrain.TakeRutStats()); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        std::cout << "SCM: wheel at z = " << wheel->GetPos().z() << " m, soil force " << terrain.GetContactForce(wheel)
                  << " N (weight " << wheel->GetMass() * gravity << " N), " << terrain.GetNumCachedNodes()
                  << " nodes sampled, " << terrain.GetNumRayCasts() << " ray casts last step" << std::endl;
        if (stats)
            std::cout << "SCM: " << stats->deformed << " deformed nodes, max depth " << stats->max_depth_m
                      << " m, mean depth " << stats->mean_depth_m << " m" << std::endl;
    }

    return 0;
}
