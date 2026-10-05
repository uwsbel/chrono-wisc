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
// Quadtree level-of-detail flyover, with no physics. A scripted camera descends
// from a few kilometers up onto a lunar landing site and then circles it at low
// altitude, while the ChPlanetVisualizationVSG plugin refines and coarsens the
// terrain tiles around it. The LOD is stepped on simulation time, so the tree
// at any moment depends only on the camera path and the run is repeatable.
// Use --wireframe to watch the tile edges and vertex density change.
//
// Command line: GeoTIFF paths form the DEM stack; with none, the Moon DEM
// resources shipped with Chrono (global low resolution plus the Apollo 17
// landing site) are used. Flags: --wireframe, --hold (skip the descent and start
// circling the site immediately).
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"
#include "chrono_planet/planets/moon/ChMoon.h"

#include "PlanetDemoSetup.h"
#include "chrono_planet/visualization/ChPlanetVisualizationVSG.h"

#include "chrono_vsg/ChGuiComponentVSG.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::planet;

// Landing site (Apollo 17 region) and the quadtree zoom the site elevation is sampled at.
const double site_lon = 30.75;
const double site_lat = 20.19;
const int zoom = 15;

// Quadtree root tile size (deg) and ring half-width (tiles).
const double root_tile_deg = 16.0;
const int view_range_tiles = 10;

// Flight plan, in site meters. Descent from start to hover, then a circle about the site.
const ChVector3d start_pos(-2500, -6000, 3500);
const ChVector3d hover_pos(0, -80, 30);
const double descent_time = 45.0;
const double circle_radius = 80.0;
const double circle_altitude = 30.0;
const double circle_period = 90.0;

// Frame time; the LOD tick is ChPlanetQuadtree::kTickS, so this advances one tick per frame.
const double frame_time = ChPlanetQuadtree::kTickS;

// -----------------------------------------------------------------------------
// Values the GUI panel shows
// -----------------------------------------------------------------------------
struct FlightStats {
    double lon = 0, lat = 0, altitude = 0, ground = 0;
    double distance_to_site = 0;
    int tiles = 0, roots = 0;
    long long tick = 0;
    const char* phase = "";
};

class FlightStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    FlightStatsVSG(const FlightStats& stats) : m_stats(stats) {}

    virtual void render(vsg::CommandBuffer& cb) override {
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin("Flyover");
        if (ImGui::BeginTable("Camera", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit)) {
            Row("Phase", "%s", m_stats.phase);
            Row("Longitude (deg)", "%10.6f", m_stats.lon);
            Row("Latitude (deg)", "%10.6f", m_stats.lat);
            Row("Height above ground (m)", "%9.1f", m_stats.altitude - m_stats.ground);
            Row("Ground elevation (m)", "%9.1f", m_stats.ground);
            Row("Distance to site (m)", "%9.1f", m_stats.distance_to_site);
            ImGui::EndTable();
        }
        if (ImGui::BeginTable("Quadtree", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit)) {
            Row("LOD tick", "%lld", m_stats.tick);
            Row("Root tiles", "%d", m_stats.roots);
            Row("Leaf meshes in scene", "%d", m_stats.tiles);
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

    const FlightStats& m_stats;
};

// Smooth step easing in [0, 1].
static double Ease(double s) {
    s = std::clamp(s, 0.0, 1.0);
    return s * s * (3.0 - 2.0 * s);
}

// Camera position at flight time t.
static ChVector3d CameraAt(double t, bool hold, const char*& phase) {
    if (!hold && t < descent_time) {
        phase = "descent";
        const double s = Ease(t / descent_time);
        // Altitude eases faster than ground distance, so the camera flattens out on approach.
        ChVector3d p = start_pos + s * (hover_pos - start_pos);
        p.z() = start_pos.z() + Ease(s) * (hover_pos.z() - start_pos.z());
        return p;
    }
    phase = "circling";
    const double tc = hold ? t : t - descent_time;
    const double a = -CH_PI_2 + CH_2PI * tc / circle_period;
    return ChVector3d(circle_radius * std::cos(a), circle_radius * std::sin(a), circle_altitude);
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    bool wireframe = false;
    bool hold = false;
    std::vector<ChGeoTiffSource> dems;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--wireframe"))
            wireframe = true;
        else if (!std::strcmp(argv[i], "--hold"))
            hold = true;
        else {
            dems.push_back({argv[i], 0, 30});
        }
    }

    // The surface only supplies the site elevation and the ground height under the camera.
    // One surface for physics and rendering, so the wheels ride the drawn ground.
    auto surface = CreateDemoSurface(dems, zoom, root_tile_deg);
    if (!surface)
        return 1;
    ChSiteFrame site = surface->MakeSiteFrame(site_lon, site_lat);

    auto world = chrono_types::make_shared<ChPlanetQuadtree>(surface, view_range_tiles);

    // An empty system, only to carry the simulation clock the LOD is stepped on.
    ChSystemNSC sys;

    auto vis_planet = chrono_types::make_shared<ChPlanetVisualizationVSG>(world, site);
    vis_planet->SetWireframe(wireframe);

    FlightStats stats;
    const char* phase = "";
    const ChVector3d cam0 = CameraAt(0.0, hold, phase);

    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->AttachPlugin(vis_planet);
    vis->AddGuiComponent(chrono_types::make_shared<FlightStatsVSG>(stats));
    vis->SetWindowTitle("Planet quadtree flyover");
    vis->SetWindowSize(1280, 800);
    vis->SetWindowPosition(100, 100);
    vis->SetBackgroundColor(ChColor(0.0f, 0.0f, 0.0f));
    vis->AddCamera(cam0, ChVector3d(0, 0, 0));
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetCameraAngleDeg(50.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.2, 0.3);
    vis->Initialize();

    // The flight is paced to real time, one LOD tick per frame.
    ChRealtimeStepTimer realtime;
    int frame = 0;
    while (vis->Run()) {
        const double t = sys.GetChTime();
        const ChVector3d cam = CameraAt(t, hold, phase);
        vis->SetCameraPosition(cam);
        vis->SetCameraTarget(ChVector3d(0, 0, 0));

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Advance the clock one LOD tick; the plugin steps the tree to it on the next frame.
        sys.SetChTime(t + frame_time);
        realtime.Spin(frame_time);

        if (frame % 10 == 0) {
            site.ToLonLat(cam.x(), cam.y(), stats.lon, stats.lat);
            stats.altitude = site.GetOriginElevation() + cam.z();
            stats.ground = surface->GetElevation(stats.lon, stats.lat);
            stats.distance_to_site = ChVector2d(cam.x(), cam.y()).Length();
            stats.tiles = static_cast<int>(vis_planet->GetNumTiles());
            stats.roots = world->GetNumRootTiles();
            stats.tick = world->GetTick();
            stats.phase = phase;
        }
        if (++frame % 300 == 0)
            std::cout << "t = " << t << " s (" << phase << "), height above ground "
                      << stats.altitude - stats.ground << " m, tick " << stats.tick << ", root tiles " << stats.roots
                      << ", leaf meshes " << stats.tiles << std::endl;
    }

    return 0;
}
