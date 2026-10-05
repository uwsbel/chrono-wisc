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
// VIPER rover on rigid planetary terrain. The rover drives a gentle S-curve
// away from a lunar landing site over a PlanetTerrain collision patch that is
// rebuilt as the rover nears its edge, through a field of procedural boulders
// that are spawned around it as it goes. The ChPlanetVisualizationVSG plugin
// streams the quadtree tiles around the chase camera, and an ImGui panel shows
// where the rover is on the planet.
//
// Command line: GeoTIFF paths form the DEM stack shared by physics and
// rendering; with none, the Moon DEM resources shipped with Chrono
// (global low resolution plus the Apollo 17 landing site) are used. Flags: --no-rocks skips the
// boulder field, --wireframe draws the terrain tiles as wireframe.
//
// =============================================================================

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/collision/ChCollisionShapeCylinder.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolver.h"

#include "chrono_models/robot/viper/Viper.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"
#include "chrono_planet/planets/moon/ChMoon.h"

#include "PlanetDemoSetup.h"
#include "chrono_planet/visualization/ChPlanetVisualizationVSG.h"

#include "chrono_vehicle/terrain/PlanetTerrain.h"

#include "chrono_vsg/ChGuiComponentVSG.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

#include "PlanetBoulderField.h"

using namespace chrono;
using namespace chrono::planet;
using namespace chrono::vehicle;
using namespace chrono::viper;

// Landing site (Apollo 17 region) and the quadtree zoom the physics surface is sampled at.
const double site_lon = 30.75;
const double site_lat = 20.19;
const int zoom = 15;

// Quadtree root tile size (deg) and ring half-width (tiles).
const double root_tile_deg = 16.0;
const int view_range_tiles = 10;

// Physics
const double gravity = moon::kGravity;
const double step_size = 1e-3;
const int render_steps = 20;  // simulation steps per rendered frame

// Rigid patch that follows the rover
const double patch_size = 40.0;
const double patch_resolution = 0.25;
const double rebuild_margin = 8.0;

// Boulders: every rock at least this big within this distance of the rover
const double rock_min_radius = 0.12;
const double rock_spawn_radius = 35.0;

// VIPER wheels. The rover is drawn with its real wheels, but for contact each wheel's triangle mesh is
// replaced by a cylinder: Bullet resolves a cylinder against the terrain mesh far faster than mesh against
// mesh (the terrain patch is tens of thousands of triangles), and on rigid ground the grousers add little.
// Set cylinder_wheel_collision to false to keep the wheel meshes for contact.
const ViperWheelType wheel_type = ViperWheelType::RealWheel;
const bool cylinder_wheel_collision = true;
const double wheel_radius = 0.25;
const double wheel_width = 0.29;

// -----------------------------------------------------------------------------
// Values the GUI panel shows
// -----------------------------------------------------------------------------
struct RoverStats {
    double lon = 0, lat = 0, elev = 0;
    double speed = 0, distance = 0;
    double pitch_deg = 0, roll_deg = 0;
    int tiles = 0, rebuilds = 0, rocks = 0;
};

class PlanetStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    PlanetStatsVSG(const RoverStats& stats) : m_stats(stats) {}

    virtual void render(vsg::CommandBuffer& cb) override {
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin("Planet");
        if (ImGui::BeginTable("Rover", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit)) {
            Row("Longitude (deg)", "%10.6f", m_stats.lon);
            Row("Latitude (deg)", "%10.6f", m_stats.lat);
            Row("Elevation (m)", "%10.2f", m_stats.elev);
            Row("Speed (m/s)", "%6.2f", m_stats.speed);
            Row("Distance (m)", "%8.1f", m_stats.distance);
            Row("Pitch / roll (deg)", "%5.1f / %5.1f", m_stats.pitch_deg, m_stats.roll_deg);
            ImGui::EndTable();
        }
        if (ImGui::BeginTable("Terrain", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit)) {
            Row("Tiles in scene", "%d", m_stats.tiles);
            Row("Patch rebuilds", "%d", m_stats.rebuilds);
            Row("Boulders", "%d", m_stats.rocks);
            ImGui::EndTable();
        }
        ImGui::End();
    }

  private:
    template <typename... Args>
    static void Row(const char* label, const char* fmt, Args... args) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(label);
        ImGui::TableNextColumn();
        ImGui::Text(fmt, args...);
    }

    const RoverStats& m_stats;
};

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Command line: flags, then GeoTIFF paths.
    bool use_rocks = true;
    bool wireframe = false;
    std::vector<ChGeoTiffSource> dems;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--no-rocks"))
            use_rocks = false;
        else if (!std::strcmp(argv[i], "--wireframe"))
            wireframe = true;
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

    // Multibody system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetNumThreads(1, 1, 4);

    auto ground_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    ground_mat->SetFriction(0.9f);
    ground_mat->SetRestitution(0.01f);

    // Rigid terrain patch
    PlanetTerrain terrain(&sys, surface, site);
    terrain.SetPatchSize(patch_size);
    terrain.SetPatchResolution(patch_resolution);
    terrain.SetRebuildMargin(rebuild_margin);
    terrain.SetContactMaterial(ground_mat);
    terrain.Initialize();
    // The ground body gets the family the boulders ignore; a rebuild creates a new body, so this is repeated then.
    terrain.GetGroundBody()->GetCollisionModel()->SetFamily(PlanetBoulderField::kTerrainFamily);

    // Rover, facing east, resting just above the highest ground under its wheels
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    Viper viper(&sys, wheel_type);
    viper.SetDriver(driver);
    viper.SetWheelContactMaterial(ground_mat);

    double ground = -1e9;
    for (double wx : {-0.64, 0.64})
        for (double wy : {-0.61, 0.61})
            ground = std::max(ground, terrain.GetHeight(ChVector3d(wx, wy, 0)));
    viper.Initialize(ChFrame<>(ChVector3d(0, 0, ground + wheel_radius + 0.1), QUNIT));
    auto chassis = viper.GetChassis()->GetBody();

    // Swap the wheel collision meshes for cylinders (before the first step, when collision models are bound).
    // The wheel spins about its local y axis, so the cylinder's axis (z by default) is rotated onto y.
    if (cylinder_wheel_collision) {
        for (auto id : {V_LF, V_RF, V_LB, V_RB}) {
            auto wheel = viper.GetWheel(id)->GetBody();
            wheel->GetCollisionModel()->Clear();
            auto cyl = chrono_types::make_shared<ChCollisionShapeCylinder>(ground_mat, wheel_radius, wheel_width);
            wheel->AddCollisionShape(cyl, ChFrame<>(VNULL, Q_ROTATE_Z_TO_Y));
        }
    }

    // Boulder field, seeded around the site before the rover moves
    PlanetBoulderField boulders(&sys, surface, site, ground_mat, rock_min_radius);
    if (use_rocks)
        boulders.Update(chassis->GetPos(), rock_spawn_radius);
    std::cout << "Boulders around the site: " << boulders.GetNumRocks() << std::endl;

    // Visualization: quadtree tiles around the camera, plus the rover and rocks
    auto vis_planet = chrono_types::make_shared<ChPlanetVisualizationVSG>(world, site);
    vis_planet->SetWireframe(wireframe);

    RoverStats stats;
    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->AttachPlugin(vis_planet);
    vis->AddGuiComponent(chrono_types::make_shared<PlanetStatsVSG>(stats));
    vis->SetWindowTitle("VIPER on rigid planetary terrain");
    vis->SetWindowSize(1280, 800);
    vis->SetWindowPosition(100, 100);
    vis->SetBackgroundColor(ChColor(0.01f, 0.01f, 0.02f));
    vis->AddCamera(chassis->GetPos() + ChVector3d(-6, -4, 2.5), chassis->GetPos());
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.2, 0.35);
    vis->Initialize();

    // Simulation loop
    const auto wall_start = std::chrono::steady_clock::now();
    int step = 0;
    ChVector3d last_pos = chassis->GetPos();
    while (vis->Run()) {
        const double time = sys.GetChTime();
        const ChVector3d pos = chassis->GetPos();
        const ChQuaterniond rot = chassis->GetRot();

        if (step % render_steps == 0) {
            // Chase camera behind the rover, along its heading
            const ChVector3d heading = rot.GetAxisX();
            vis->SetCameraPosition(pos - 6.0 * heading + ChVector3d(0, 0, 2.5));
            vis->SetCameraTarget(pos + ChVector3d(0, 0, 0.5));

            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        // Steering schedule: straight for 10 s, then a gentle S-curve (about 13 m turning radius) so the rover
        // keeps making headway and crosses patch and boulder spawn boundaries
        const double steering = time < 10.0 ? 0.0 : 0.10 * std::sin(0.20 * (time - 10.0));
        driver->SetSteering(steering);

        sys.DoStepDynamics(step_size);
        viper.Update();

        // Keep the collision patch and the boulder field around the rover
        if (terrain.UpdatePatch(pos)) {
            ++stats.rebuilds;
            terrain.GetGroundBody()->GetCollisionModel()->SetFamily(PlanetBoulderField::kTerrainFamily);
        }
        if (use_rocks && step % 200 == 0)
            boulders.Update(pos, rock_spawn_radius, vis.get());

        // Stats
        stats.distance += (pos - last_pos).Length();
        last_pos = pos;
        if (step % render_steps == 0) {
            site.ToLonLat(pos.x(), pos.y(), stats.lon, stats.lat);
            stats.elev = site.GetOriginElevation() + pos.z();
            stats.speed = chassis->GetLinVel().Length();
            const ChVector3d euler = rot.GetCardanAnglesXYZ();
            stats.roll_deg = euler.x() * CH_RAD_TO_DEG;
            stats.pitch_deg = euler.y() * CH_RAD_TO_DEG;
            stats.tiles = static_cast<int>(vis_planet->GetNumTiles());
            stats.rocks = static_cast<int>(boulders.GetNumRocks());
        }

        if (++step % 2000 == 0) {
            const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
            std::cout << "t = " << time << " s (" << time / wall << "x real time), at (" << pos.x() << ", " << pos.y()
                      << ") m, distance " << stats.distance
                      << " m, speed " << stats.speed << " m/s, patch rebuilds " << stats.rebuilds << ", boulders "
                      << stats.rocks << ", tiles " << stats.tiles << std::endl;
        }
    }

    return 0;
}
