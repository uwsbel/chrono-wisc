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
// VIPER rover on deformable planetary regolith. The rover drives away from a
// lunar landing site over a PlanetSCMTerrain, the SCM soil model with its
// undeformed surface taken from the ChPlanetSurface, so the soil is unbounded
// and ruts persist for the whole run. Procedural boulders are spawned around the rover
// as fixed bodies (note that SCM also senses a rock that enters a wheel's
// active box, since it finds wheels by ray-casting the collision system). The
// quadtree tiles drawn around the camera show the ruts: SCM publishes its
// sinkage into a ChDeformationFilter at the end of a view of the physics
// surface, and the tiles the ruts touch are rebuilt. The SCM plugin shows the
// wheel active domains, and a panel reports sinkage, soil
// forces and a periodic rut summary. Wheel sinkage and soil forces are written
// to a CSV file at exit.
//
// Command line: GeoTIFF paths form the DEM stack shared by physics and
// rendering; with none, the Moon DEM resources shipped with Chrono
// (global low resolution plus the Apollo 17 landing site) are used. Flags: --no-rocks skips the
// boulder field, --bulldozing enables soil displacement at the rut borders,
// --cpu-raycast forces SCM's CPU ray casting in a build with the SCM GPU backend.
//
// =============================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "chrono/utils/ChOpenMP.h"
#include "chrono/core/ChDataPath.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolver.h"

#include "chrono_models/robot/viper/Viper.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/filters/ChDeformationFilter.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"
#include "chrono_planet/planets/moon/ChMoon.h"

#include "PlanetDemoSetup.h"
#include "chrono_planet/visualization/ChPlanetVisualizationVSG.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/terrain/PlanetSCMTerrain.h"
#include "chrono_vehicle/visualization/ChScmVisualizationVSG.h"

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
const int render_steps = 20;     // simulation steps per rendered frame
const double rut_stats_period = 2.0;  // sim seconds between rut summaries

// Soil grid
const double scm_delta = 0.05;

// Boulders: every rock at least this big within this distance of the rover
const double rock_min_radius = 0.15;
const double rock_spawn_radius = 30.0;

// VIPER geometry (real wheel)
const double wheel_radius = 0.25;
const double wheel_width = 0.30;

// -----------------------------------------------------------------------------
// Values the GUI panel shows
// -----------------------------------------------------------------------------
struct RoverStats {
    double lon = 0, lat = 0, elev = 0;
    double speed = 0, distance = 0;
    std::array<double, 4> sinkage{};  ///< wheel center depth below the undeformed surface (m)
    std::array<double, 4> soil_force{};
    double soil_total = 0, weight = 0;
    int tiles = 0, rocks = 0;
    size_t cached_nodes = 0;
    int ray_casts = 0;
    std::optional<PlanetSCMTerrain::RutStats> rut;
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
            ImGui::EndTable();
        }
        if (ImGui::BeginTable("Soil", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit)) {
            Row("Sinkage LF/RF (mm)", "%5.1f / %5.1f", 1e3 * m_stats.sinkage[0], 1e3 * m_stats.sinkage[1]);
            Row("Sinkage LB/RB (mm)", "%5.1f / %5.1f", 1e3 * m_stats.sinkage[2], 1e3 * m_stats.sinkage[3]);
            Row("Soil force LF/RF (N)", "%6.0f / %6.0f", m_stats.soil_force[0], m_stats.soil_force[1]);
            Row("Soil force LB/RB (N)", "%6.0f / %6.0f", m_stats.soil_force[2], m_stats.soil_force[3]);
            Row("Total / weight (N)", "%6.0f / %6.0f", m_stats.soil_total, m_stats.weight);
            Row("Memoized nodes", "%zu", m_stats.cached_nodes);
            Row("Ray casts (last step)", "%d", m_stats.ray_casts);
            if (m_stats.rut) {
                Row("Rut area (m2)", "%7.2f", m_stats.rut->area_m2);
                Row("Rut depth max / mean (mm)", "%5.1f / %5.1f", 1e3 * m_stats.rut->max_depth_m,
                    1e3 * m_stats.rut->mean_depth_m);
            }
            ImGui::EndTable();
        }
        if (ImGui::BeginTable("Terrain", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit)) {
            Row("Tiles in scene", "%d", m_stats.tiles);
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
    bool bulldozing = false;
    bool cpu_raycast = false;
    std::vector<ChGeoTiffSource> dems;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--no-rocks"))
            use_rocks = false;
        else if (!std::strcmp(argv[i], "--cpu-raycast"))
            cpu_raycast = true;
        else if (!std::strcmp(argv[i], "--bulldozing"))
            bulldozing = true;
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

    // The drawn terrain is a view of the physics surface with the soil deformation on top. SCM samples the
    // undeformed surface; the view adds the ruts it publishes, and the quadtree rebuilds the tiles they touch.
    auto deformation = chrono_types::make_shared<ChDeformationFilter>(site, scm_delta);
    auto drawn_surface = surface->CreateView(deformation);
    auto world = chrono_types::make_shared<ChPlanetQuadtree>(drawn_surface, view_range_tiles);

    // Output directory
    const std::string out_dir = GetChronoOutputPath() + "PLANET_Viper_SCM";
    if (!CreateOutputDirectory(out_dir)) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Multibody system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    // SCM ray-casts every wheel's grid nodes in parallel on Chrono's threads, and it dominates the step.
    // Gains flatten past about 8 threads.
    sys.SetNumThreads(std::min(8, ChOMP::GetNumProcs()), 1, 4);

    auto rock_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    rock_mat->SetFriction(0.8f);
    rock_mat->SetRestitution(0.01f);

    // Rover, facing east, wheels resting on the undeformed surface
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    Viper viper(&sys, ViperWheelType::RealWheel);
    viper.SetDriver(driver);
    viper.SetWheelContactMaterial(rock_mat);

    auto height_at = [&](double x, double y) {
        double lon, lat;
        site.ToLonLat(x, y, lon, lat);
        return surface->GetElevation(lon, lat) - site.GetOriginElevation();
    };
    double ground = -1e9;
    for (double wx : {-0.64, 0.64})
        for (double wy : {-0.61, 0.61})
            ground = std::max(ground, height_at(wx, wy));
    viper.Initialize(ChFrame<>(ChVector3d(0, 0, ground + wheel_radius + 0.05), QUNIT));
    auto chassis = viper.GetChassis()->GetBody();

    const std::array<ViperWheelID, 4> wheel_ids = {V_LF, V_RF, V_LB, V_RB};
    std::vector<PlanetSCMTerrain::Wheel> wheels;
    for (auto id : wheel_ids)
        wheels.push_back({viper.GetWheel(id)->GetBody(), wheel_radius, wheel_width});

    // Deformable terrain
    PlanetSCMTerrain terrain(&sys, surface, site);
    PlanetSCMTerrain::Params params;
    params.delta = scm_delta;
    // Soft, loose regolith, so the rover leaves clear ruts: the lunar set of demo_ROBOT_Viper_SCM_Sensor with
    // half its stiffnesses and 1 kPa of cohesion, the upper end of Apollo estimates for the lunar surface.
    params.bekker_kphi = 0.1e6;   // frictional modulus (Pa/m^n)
    params.bekker_kc = 0;         // cohesive modulus (Pa/m^(n-1))
    params.bekker_n = 1.1;        // sinkage exponent
    params.mohr_cohesion = 1e3;   // cohesion (Pa)
    params.mohr_friction = 30;    // internal friction angle (deg)
    params.janosi_shear = 0.01;   // Janosi shear displacement (m)
    params.elastic_k = 2e7;       // elastic stiffness (Pa/m), must exceed bekker_kphi
    params.damping_r = 3e4;      // vertical damping (Pa s/m)
    params.bulldozing = bulldozing;
    terrain.Initialize(params, wheels);
    terrain.SetDeformationFilter(deformation);
#ifdef CHRONO_HAS_SCM_GPU
    // With the SCM GPU backend, ray casting runs on the GPU (VIPER's wheels are triangle meshes and each
    // wheel has an active domain); --cpu-raycast keeps it on the CPU for comparison.
    if (cpu_raycast)
        terrain.EnableRaycastGpuHip(false);
#else
    if (cpu_raycast)
        std::cout << "--cpu-raycast has no effect: this build has no SCM GPU backend" << std::endl;
#endif
    terrain.SetPlotType(SCMTerrain::PLOT_SINKAGE, 0, 0.15);

    // Boulder field, seeded around the site before the rover moves
    PlanetBoulderField boulders(&sys, surface, site, rock_mat, rock_min_radius);
    if (use_rocks)
        boulders.Update(chassis->GetPos(), rock_spawn_radius);
    std::cout << "Boulders around the site: " << boulders.GetNumRocks() << std::endl;

    // Visualization: undeformed quadtree tiles, SCM active boxes, rover and rocks
    auto vis_planet = chrono_types::make_shared<ChPlanetVisualizationVSG>(world, site);
    // Ruts are a few centimeters deep: color them by depth, on the same scale as the SCM sinkage legend.
    vis_planet->SetDeformationColoring(deformation, 0.15);
    auto vis_scm = chrono_types::make_shared<ChScmVisualizationVSG>(&terrain);

    RoverStats stats;
    stats.weight = viper.GetRoverMass() * gravity;

    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->AttachPlugin(vis_planet);
    vis->AttachPlugin(vis_scm);
    vis->AddGuiComponent(chrono_types::make_shared<PlanetStatsVSG>(stats));
    vis->SetWindowTitle("VIPER on planetary SCM regolith");
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
    ChWriterCSV csv(" ");
    csv << "time x y z speed sink_LF sink_RF sink_LB sink_RB force_LF force_RF force_LB force_RB" << std::endl;

    const auto wall_start = std::chrono::steady_clock::now();
    int step = 0;
    double next_rut_request = rut_stats_period;
    ChVector3d last_pos = chassis->GetPos();
    while (vis->Run()) {
        const double time = sys.GetChTime();
        const ChVector3d pos = chassis->GetPos();
        const ChVector3d heading = chassis->GetRot().GetAxisX();

        if (step % render_steps == 0) {
            terrain.PublishDeformation();  // ruts into the drawn terrain
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


        // Rut summary, computed on a worker thread
        if (time >= next_rut_request) {
            terrain.RequestRutStats();
            next_rut_request += rut_stats_period;
        }
        if (auto rut = terrain.TakeRutStats())
            stats.rut = rut;

        // Boulders around the rover
        if (use_rocks && step % 200 == 0)
            boulders.Update(pos, rock_spawn_radius, vis.get());

        // Stats
        stats.distance += (pos - last_pos).Length();
        last_pos = pos;
        if (step % render_steps == 0) {
            site.ToLonLat(pos.x(), pos.y(), stats.lon, stats.lat);
            stats.elev = site.GetOriginElevation() + pos.z();
            stats.speed = chassis->GetLinVel().Length();
            stats.soil_total = 0;
            for (int i = 0; i < 4; ++i) {
                const auto& wheel = wheels[i].body;
                const ChVector3d wp = wheel->GetPos();
                stats.sinkage[i] = height_at(wp.x(), wp.y()) + wheel_radius - wp.z();
                stats.soil_force[i] = terrain.GetContactForce(wheel);
                stats.soil_total += stats.soil_force[i];
            }
            stats.cached_nodes = terrain.GetNumCachedNodes();
            stats.ray_casts = terrain.GetNumRayCasts();
            stats.tiles = static_cast<int>(vis_planet->GetNumTiles());
            stats.rocks = static_cast<int>(boulders.GetNumRocks());

            csv << time << pos.x() << pos.y() << pos.z() << stats.speed;
            for (int i = 0; i < 4; ++i)
                csv << stats.sinkage[i];
            for (int i = 0; i < 4; ++i)
                csv << stats.soil_force[i];
            csv << std::endl;
        }

        if (++step % 2000 == 0) {
            const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
            std::cout << "t = " << time << " s (" << time / wall << "x real time), at (" << pos.x() << ", " << pos.y()
                      << ") m, distance " << stats.distance
                      << " m, speed " << stats.speed << " m/s, soil force " << stats.soil_total << " N (weight "
                      << stats.weight << " N), " << stats.cached_nodes << " memoized nodes, "
                      << deformation->GetNumNodes() << " deformed nodes drawn, " << world->GetNumRebuiltTiles()
                      << " tiles rebuilt, GPU ray-cast steps " << terrain.GetNumRaycastGpuSteps();
            if (stats.rut)
                std::cout << ", rut area " << stats.rut->area_m2 << " m2, max depth " << stats.rut->max_depth_m << " m";
            std::cout << std::endl;
        }
    }

    const std::string csv_file = out_dir + "/sinkage.csv";
    csv.WriteToFile(csv_file);
    std::cout << "Wrote " << csv_file << std::endl;

    return 0;
}
