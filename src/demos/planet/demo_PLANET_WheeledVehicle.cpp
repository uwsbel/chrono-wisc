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
// A Chrono::Vehicle wheeled vehicle on planetary terrain. An HMMWV with rigid
// tires is driven interactively (keyboard: W/S throttle, A/D steering, X brake)
// at a lunar landing site under lunar gravity, over a PlanetTerrain collision
// patch that follows the vehicle. The vehicle visual system draws the usual
// vehicle GUI and chase camera, the ChPlanetVisualizationVSG plugin streams the
// quadtree tiles around the camera, and procedural boulders are spawned as
// fixed obstacles around the vehicle as it goes. This shows that any Chrono::
// Vehicle model runs on a PlanetTerrain like on any other ChTerrain.
//
// Command line: GeoTIFF paths form the DEM stack shared by physics and
// rendering; with none, the Moon DEM resources shipped with Chrono
// (global low resolution plus the Apollo 17 landing site) are used. Flags: --no-rocks skips the
// boulder field, --earth uses Earth gravity.
//
// =============================================================================

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/solver/ChIterativeSolver.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"
#include "chrono_planet/planets/moon/ChMoon.h"

#include "PlanetDemoSetup.h"
#include "chrono_planet/visualization/ChPlanetVisualizationVSG.h"

#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/PlanetTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"

#include "PlanetBoulderField.h"

using namespace chrono;
using namespace chrono::planet;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// Landing site (Apollo 17 region) and the quadtree zoom the physics surface is sampled at.
const double site_lon = 30.75;
const double site_lat = 20.19;
const int zoom = 15;

// Quadtree root tile size (deg) and ring half-width (tiles).
const double root_tile_deg = 16.0;
const int view_range_tiles = 10;

// Physics
const double step_size = 1e-3;
const double render_step_size = 1.0 / 50;

// Rigid patch that follows the vehicle
const double patch_size = 80.0;
const double patch_resolution = 0.5;
const double rebuild_margin = 15.0;

// Boulders: only rocks big enough to matter to a vehicle, within this distance of it
const double rock_min_radius = 0.25;
const double rock_spawn_radius = 60.0;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Command line: flags, then GeoTIFF paths.
    bool use_rocks = true;
    double gravity = moon::kGravity;
    std::vector<ChGeoTiffSource> dems;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--no-rocks"))
            use_rocks = false;
        else if (!std::strcmp(argv[i], "--earth"))
            gravity = 9.81;
        else {
            dems.push_back({argv[i], 0, 30});
        }
    }

    // Surface, site frame and quadtree world
    // One surface for physics and rendering, so the wheels ride the drawn ground.
    auto surface = CreateDemoSurface(dems, zoom, root_tile_deg);
    if (!surface)
        return 1;
    ChSiteFrame site = surface->MakeSiteFrame(site_lon, site_lat);
    std::cout << "Site origin elevation: " << site.GetOriginElevation() << " m" << std::endl;

    auto world = chrono_types::make_shared<ChPlanetQuadtree>(surface, view_range_tiles);

    // Ground height in the site frame, from the surface itself (exact anywhere, no patch needed)
    auto height_at = [&](double x, double y) {
        double lon, lat;
        site.ToLonLat(x, y, lon, lat);
        return surface->GetElevation(lon, lat) - site.GetOriginElevation();
    };

    // Vehicle at the site, facing east, just above the highest ground under its footprint.
    // It is created before the terrain so the terrain can share its system.
    double ground = -1e9;
    for (double wx : {-2.0, 2.0})
        for (double wy : {-1.0, 1.0})
            ground = std::max(ground, height_at(wx, wy));

    HMMWV_Reduced hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(ChContactMethod::NSC);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetChassisCollisionType(CollisionType::NONE);
    hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(0, 0, ground + 0.7), QUNIT));
    hmmwv.SetEngineType(EngineModelType::SIMPLE);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.SetBrakeType(BrakeType::SHAFTS);
    hmmwv.SetTireType(TireModelType::RIGID);
    hmmwv.Initialize();
    hmmwv.SetChassisVisualizationType(VisualizationType::MESH);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    ChSystem* sys = hmmwv.GetSystem();
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
    sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys->GetSolver()->AsIterative()->SetMaxIterations(150);
    sys->SetNumThreads(1, 1, 4);

    auto ground_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    ground_mat->SetFriction(0.9f);
    ground_mat->SetRestitution(0.01f);

    // Rigid terrain patch
    PlanetTerrain terrain(sys, surface, site);
    terrain.SetPatchSize(patch_size);
    terrain.SetPatchResolution(patch_resolution);
    terrain.SetRebuildMargin(rebuild_margin);
    terrain.SetContactMaterial(ground_mat);
    terrain.Initialize();
    // The ground body gets the family the boulders ignore; a rebuild creates a new body, so this is repeated then.
    terrain.GetGroundBody()->GetCollisionModel()->SetFamily(PlanetBoulderField::kTerrainFamily);

    auto& vehicle = hmmwv.GetVehicle();

    // Boulder field, seeded around the site before the vehicle moves
    PlanetBoulderField boulders(sys, surface, site, ground_mat, rock_min_radius);
    if (use_rocks)
        boulders.Update(vehicle.GetPos(), rock_spawn_radius);
    std::cout << "Boulders around the site: " << boulders.GetNumRocks() << std::endl;

    // Interactive driver
    ChInteractiveDriver driver(vehicle);
    driver.SetSteeringDelta(render_step_size / 1.0);
    driver.SetThrottleDelta(render_step_size / 1.0);
    driver.SetBrakingDelta(render_step_size / 0.3);
    driver.Initialize();

    // Visualization: vehicle chase camera and GUI, plus the quadtree tiles around the camera
    auto vis_planet = chrono_types::make_shared<ChPlanetVisualizationVSG>(world, site);

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
    vis->SetWindowTitle("HMMWV on planetary terrain");
    vis->SetWindowSize(1280, 800);
    vis->SetWindowPosition(100, 100);
    vis->SetBackgroundColor(ChColor(0.01f, 0.01f, 0.02f));
    vis->AttachVehicle(&vehicle);
    vis->AttachDriver(&driver);
    vis->AttachTerrain(&terrain);
    vis->AttachPlugin(vis_planet);
    vis->SetChaseCamera(ChVector3d(0, 0, 1.75), 8.0, 0.5);
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.2, 0.35);
    vis->Initialize();

    // Simulation loop
    const int render_steps = static_cast<int>(std::ceil(render_step_size / step_size));
    int step = 0;
    int rebuilds = 0;
    vehicle.EnableRealtime(true);
    while (vis->Run()) {
        const double time = sys->GetChTime();
        const ChVector3d pos = vehicle.GetPos();

        if (step % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        DriverInputs driver_inputs = driver.GetInputs();

        driver.Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        driver.Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        vis->Advance(step_size);

        // Keep the collision patch and the boulder field around the vehicle
        if (terrain.UpdatePatch(pos)) {
            ++rebuilds;
            terrain.GetGroundBody()->GetCollisionModel()->SetFamily(PlanetBoulderField::kTerrainFamily);
        }
        if (use_rocks && step % 200 == 0)
            boulders.Update(pos, rock_spawn_radius, vis.get());

        if (++step % 2000 == 0) {
            double lon, lat;
            site.ToLonLat(pos.x(), pos.y(), lon, lat);
            std::cout << "t = " << time << " s, vehicle at (" << lon << ", " << lat << ") deg, elevation "
                      << site.GetOriginElevation() + pos.z() << " m, speed " << vehicle.GetSpeed()
                      << " m/s, patch rebuilds " << rebuilds << ", boulders " << boulders.GetNumRocks()
                      << ", tiles " << vis_planet->GetNumTiles() << std::endl;
        }
    }

    return 0;
}
