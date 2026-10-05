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
// VIPER rover on CRM regolith at a lunar landing site. The soil under the rover
// is CRM (Chrono::FSI SPH continuum soil) in a window of the planet surface
// that follows the rover (vehicle::PlanetCRMWindow): seeded from the surface
// near the Apollo 17 landing site with the ruts made so far, and moved ahead of
// the rover when it nears the window's edge. The ruts the wheels leave go into a
// deformation filter on the drawn surface, so the quadtree terrain shows them,
// colored by depth, all along the drive.
//
// Flags: --spacing <m> (default 0.1) particle spacing; --active <m> (default
// 0.5) half-size of the soil kept active around each wheel (smaller is faster,
// but soil frozen close to a wheel changes how it drives); --proximity <n>
// (default 10) steps between neighbor searches; --snapshots <s> saves the
// window this often; --video <fps> renders frames evenly in simulated time and
// saves every one (DEMO_OUTPUT/PLANET_Viper_CRM/video), once the rover has
// settled; --speed <rad/s> (default 0.4) wheel speed; --cohesion <Pa> (default
// 200), --friction (default 0.5), --young <Pa>, --viscosity soil; --window
// <L> <W>, --boundary adami|holmes, --d0, --overburden, --gravity-ramp <s>;
// --exchange <s> coupling step; --time <s> ends the run; --no-vis runs without
// a window. To record without a window: xvfb-run -a ./demo_PLANET_Viper_CRM
// --video 30.
//
// =============================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_models/robot/viper/Viper.h"

#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/filters/ChDeformationFilter.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"
#include "chrono_planet/planets/moon/ChMoon.h"
#include "chrono_planet/visualization/ChMeshVisualizationVSG.h"
#include "chrono_planet/visualization/ChPlanetVisualizationVSG.h"

#include "chrono_vehicle/terrain/PlanetCRMWindow.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "PlanetDemoSetup.h"

using namespace chrono;
using namespace chrono::planet;
using namespace chrono::vehicle;
using namespace chrono::viper;
using namespace chrono::fsi::sph;

// Landing site (Apollo 17 region), quadtree zoom of the physics surface
const double site_lon = 30.75;
const double site_lat = 20.19;
const int zoom = 15;

const double gravity = moon::kGravity;

// Where the rover starts in the site frame (m) and its heading (deg from east): the smoothest 25 m drive nearby, clear
// of craters
const double start_x = 24, start_y = 42, start_heading = 120;

double drive_speed = 0.4;  // wheel speed (rad/s): about 0.1 m/s, VIPER's pace when traversing
const double settle_time = 2.0;  // the rover settles into the soil at rest first (s)

// Wheels held still while the rover settles, then brought up to speed over a second
class SettleThenDrive : public ViperDriver {
  public:
    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::SPEED; }
    virtual void Update(double time) override {
        const double speed = drive_speed * std::clamp(time - settle_time, 0.0, 1.0);
        drive_speeds = {speed, speed, speed, speed};
    }
};

// Soil window following the rover: size, soil depth below its lowest ground, and how close to its edge the rover
// comes before it moves. The margin keeps the wheels' active domains inside the window.
double window_length = 5.0, window_width = 3.5, window_depth = 0.25, window_margin = 1.3;

// Time steps: SPH (variable, at most this; in practice the sound speed limits it), and the exchange with the multibody
// system
const double cfd_step = 5e-4;
const double publish_period = 0.1;  // ruts written into the drawn terrain and frames rendered this often (s)

// The wheels as the soil sees them: solid cylinders the size of robot/viper/col/viper_wheel.obj, rim at 0.24 m and
// grousers to 0.25 m, too small for the soil's particles to tell apart
const double wheel_radius = 0.245, wheel_width = 0.29;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    double spacing = 0.1;
    double end_time = 1e300;
    bool use_vis = true;
    int proximity_steps = 10;
    double active = 0.5;
    double snapshot_period = 0;
    double video_fps = 0;
    double exchange_step = 2.5e-3;
    double young = 1e6, viscosity = 0.5;
    double cohesion = 200, friction = 0.5;  // loose surface regolith: the wheels leave ruts a few cm deep
    double d0 = 1.3;
    bool wheel_mesh = false;
    double gravity_ramp = 0;  // the rover's weight comes on over this long (s)
    bool holmes = false, overburden = false;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--spacing") && i + 1 < argc)
            spacing = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--time") && i + 1 < argc)
            end_time = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--window") && i + 2 < argc) {
            window_length = std::atof(argv[++i]);
            window_width = std::atof(argv[++i]);
        } else if (!std::strcmp(argv[i], "--d0") && i + 1 < argc)
            d0 = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--boundary") && i + 1 < argc)
            holmes = !std::strcmp(argv[++i], "holmes");
        else if (!std::strcmp(argv[i], "--gravity-ramp") && i + 1 < argc)
            gravity_ramp = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--wheel-mesh"))
            wheel_mesh = true;
        else if (!std::strcmp(argv[i], "--overburden"))
            overburden = true;
        else if (!std::strcmp(argv[i], "--speed") && i + 1 < argc)
            drive_speed = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--cohesion") && i + 1 < argc)
            cohesion = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--friction") && i + 1 < argc)
            friction = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--young") && i + 1 < argc)
            young = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--viscosity") && i + 1 < argc)
            viscosity = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--exchange") && i + 1 < argc)
            exchange_step = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--video") && i + 1 < argc)
            video_fps = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--snapshots") && i + 1 < argc)
            snapshot_period = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--active") && i + 1 < argc)
            active = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--proximity") && i + 1 < argc)
            proximity_steps = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--no-vis"))
            use_vis = false;
    }

    // Surface and site frame; the drawn terrain is a view of the surface with the ruts on top
    auto surface = CreateSiteSurface({}, zoom);
    if (!surface)
        return 1;
    ChSiteFrame site = surface->MakeSiteFrame(site_lon, site_lat);
    auto ruts = chrono_types::make_shared<ChDeformationFilter>(site, 0.025);
    auto world = chrono_types::make_shared<ChPlanetQuadtree>(surface->CreateView(ruts), 10);

    // Rover, resting on the surface, headed along the start's smooth path
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
    Viper viper(&sys, ViperWheelType::RealWheel);
    // Wheels turned at a set speed once the rover has settled; the default DC motors (stall torque 300 N m) are several
    // times stronger than a wheel's lunar load can push against the soil, and spin the wheels up whenever they unload.
    viper.SetDriver(chrono_types::make_shared<SettleThenDrive>());
    auto height_at = [&](double x, double y) {
        double lon, lat;
        site.ToLonLat(x, y, lon, lat);
        return surface->GetElevation(lon, lat) - site.GetOriginElevation();
    };
    const ChQuaternion<> start_rot = QuatFromAngleZ(start_heading * CH_DEG_TO_RAD);
    double ground = -1e9;
    for (double wx : {-0.64, 0.64})
        for (double wy : {-0.61, 0.61}) {
            const ChVector3d w = start_rot.Rotate(ChVector3d(wx, wy, 0));
            ground = std::max(ground, height_at(start_x + w.x(), start_y + w.y()));
        }
    viper.Initialize(ChFrame<>(ChVector3d(start_x, start_y, ground + wheel_radius + 0.005), start_rot));
    auto chassis = viper.GetChassis()->GetBody();

    // The wheels as the soil sees them: cylinders about the axle (y in a wheel's reference frame, where the soil model
    // places a body's geometry), which unlike the wheel's mesh give closed BCE sets at coarse spacing too. Soil particles
    // keep about a spacing from the outer markers, half a spacing more than from each other, so the markers' cylinder is
    // half a spacing smaller than the wheel.
    // With --wheel-mesh, the wheel's own collision mesh (grousers and all) instead, shrunk alike, and turned end for
    // end on the left side as the rover's model turns it.
    std::array<std::shared_ptr<utils::ChBodyGeometry>, 4> wheel_geometry;
    for (int i = 0; i < 4; ++i) {
        wheel_geometry[i] = chrono_types::make_shared<utils::ChBodyGeometry>();
        wheel_geometry[i]->materials.push_back(ChContactMaterialData());
        if (wheel_mesh) {
            const double tip_radius = 0.25;
            const bool left = (i == V_LF || i == V_LB);
            wheel_geometry[i]->coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(
                VNULL, left ? QuatFromAngleZ(CH_PI) : QUNIT, GetChronoDataFile("robot/viper/col/viper_wheel.obj"),
                ChVector3d(0, 0, 0), (tip_radius - 0.5 * spacing) / tip_radius));
        } else {
            wheel_geometry[i]->coll_cylinders.push_back(
                utils::ChBodyGeometry::CylinderShape(VNULL, ChVector3d(0, 1, 0), wheel_radius - 0.5 * spacing, wheel_width));
        }
    }

    // CRM soil in a window that follows the rover; each window is set up alike
    auto setup = [&](PlanetCRMTerrain& crm) {
        crm.SetVerbose(false);
        crm.SetInitialOverburden(overburden);
        crm.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
        crm.SetStepSizeCFD(cfd_step);
        ChFsiFluidSystemSPH::SoilProperties soil;
        soil.density = 1700;
        soil.Young_modulus = young;
        soil.Poisson_ratio = 0.3;
        soil.mu_I0 = 0.04;
        soil.mu_fric_s = friction;
        soil.mu_fric_2 = friction;
        soil.average_diam = 0.005;
        soil.cohesion_coeff = cohesion;
        crm.SetCrmSPH(soil);
        ChFsiFluidSystemSPH::SPHParameters sph;
        sph.integration_scheme = IntegrationScheme::RK2;
        sph.initial_spacing = spacing;
        sph.d0_multiplier = d0;
        sph.free_surface_threshold = 2.4;
        sph.artificial_viscosity = viscosity;
        sph.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
        sph.boundary_method = holmes ? BoundaryMethod::HOLMES : BoundaryMethod::ADAMI;
        sph.use_variable_time_step = true;
        sph.num_proximity_search_steps = proximity_steps;
        crm.SetSPHParameters(sph);
        // A window seeded under a rover already in its ruts: drop particles the wheels would hold
        for (int i = 0; i < 4; ++i)
            crm.AddRigidBody(viper.GetWheels()[i]->GetBody(), wheel_geometry[i], true);
        crm.SetActiveDomain(ChVector3d(active, active, active));
    };
    PlanetCRMWindow soil(sys, surface, site, ruts, spacing, setup);
    soil.SetWindow(window_length, window_width, window_depth, window_margin);
    // Ruts from the wheels' footprints: as deep as they sink, sharp at any particle spacing
    for (int i = 0; i < 4; ++i)
        soil.AddWheel(viper.GetWheels()[i]->GetBody(), wheel_radius, wheel_width);
    soil.Initialize(chassis);
    std::cout << "CRM window: " << soil.GetTerrain().GetNumSPHParticles() << " particles" << std::endl;

    std::shared_ptr<ChMeshVisualizationVSG> vis_loose;
    std::shared_ptr<vsg3d::ChVisualSystemVSG> vis;
    if (use_vis) {
        auto vis_planet = chrono_types::make_shared<ChPlanetVisualizationVSG>(world, site);
        vis_planet->SetDeformationColoring(ruts, 0.03);
        vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        vis->AttachSystem(&sys);
        vis->AttachPlugin(vis_planet);
        // Soil thrown up or pushed aside by the wheels, as clods
        auto clod = chrono_types::make_shared<ChVisualMaterial>();
        clod->SetDiffuseColor(ChColor(0.33f, 0.32f, 0.30f));
        clod->SetRoughness(0.95f);
        clod->SetMetallic(0.0f);
        vis_loose = chrono_types::make_shared<ChMeshVisualizationVSG>(clod);
        vis->AttachPlugin(vis_loose);
        vis->SetWindowTitle("VIPER on CRM regolith at a lunar landing site");
        vis->SetWindowSize(1280, 800);
        vis->SetBackgroundColor(ChColor(0.01f, 0.01f, 0.02f));
        vis->AddCamera(chassis->GetPos() + ChVector3d(-4.5, -3.0, 2.2), chassis->GetPos());
        vis->SetCameraVertical(CameraVerticalDir::Z);
        vis->SetLightIntensity(1.0f);
        vis->SetLightDirection(1.2, 0.5);
        vis->Initialize();
    }

    const std::string out_dir = GetChronoOutputPath() + "PLANET_Viper_CRM";
    if ((snapshot_period > 0 || video_fps > 0) && !CreateOutputDirectory(out_dir)) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    double next_snapshot = snapshot_period;
    int snapshot = 0;
    if (video_fps > 0 && !CreateOutputDirectory(out_dir + "/video")) {
        std::cerr << "Error creating directory " << out_dir << "/video" << std::endl;
        return 1;
    }
    int frame = 0;
    ChVector3d camera_target = chassis->GetPos();
    ChVector3d camera_heading(std::cos(start_heading * CH_DEG_TO_RAD), std::sin(start_heading * CH_DEG_TO_RAD), 0);
    // Frames rendered this often (s): the ruts' publish period, or evenly at the video's rate
    const double frame_period = video_fps > 0 ? 1 / video_fps : publish_period;

    // Simulation loop
    double next_publish = 0, next_report = 1.0;
    double distance = 0;
    double publish_ms = 0;
    double bounce = 0;  // integral of the chassis' vertical speed squared over the report period
    int publishes = 0;
    ChVector3d last = chassis->GetPos();
    const auto wall_start = std::chrono::steady_clock::now();
    while (sys.GetChTime() < end_time) {
        const double time = sys.GetChTime();
        if (vis && !vis->Run())
            break;
        if (time >= next_publish) {
            const auto p0 = std::chrono::steady_clock::now();
            soil.Publish();
            if (vis_loose)
                vis_loose->SetMesh(soil.UpdateLooseSoil());
            publish_ms += 1e3 * std::chrono::duration<double>(std::chrono::steady_clock::now() - p0).count();
            ++publishes;
            if (vis) {
                // Behind and to the right of the rover, turning with it slowly
                ChVector3d heading = chassis->GetRot().GetAxisX();
                heading.z() = 0;
                heading.Normalize();
                const double follow = 1 - std::exp(-frame_period / 1.5);
                camera_heading = (camera_heading + follow * (heading - camera_heading)).GetNormalized();
                const ChVector3d side(camera_heading.y(), -camera_heading.x(), 0);
                // following the chassis smoothly, so its bounce does not shake the view
                const double track = 1 - std::exp(-frame_period / 0.5);
                camera_target += track * (chassis->GetPos() - camera_target);
                vis->SetCameraPosition(camera_target - 4.5 * camera_heading + 3.0 * side + ChVector3d(0, 0, 2.2));
                vis->SetCameraTarget(camera_target);
                vis->Render();
                if (snapshot_period > 0 && time >= next_snapshot) {
                    char name[64];
                    std::snprintf(name, sizeof(name), "/snapshot_%03d.png", snapshot++);
                    vis->WriteImageToFile(out_dir + name);
                    next_snapshot += snapshot_period;
                }
            }
            if (vis && video_fps > 0 && time >= settle_time) {
                char name[64];
                std::snprintf(name, sizeof(name), "/video/frame_%05d.png", frame++);
                vis->WriteImageToFile(out_dir + name);
            }
            next_publish += frame_period;
        }

        if (gravity_ramp > 0) {
            const double ramp = std::min(1.0, time / gravity_ramp);
            sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity * ramp));
        }
        viper.Update();
        soil.Advance(exchange_step);

        const ChVector3d pos = chassis->GetPos();
        bounce += chassis->GetPosDt().z() * chassis->GetPosDt().z() * exchange_step;
        distance += (pos - last).Length();
        last = pos;
        if (sys.GetChTime() >= next_report) {
            next_report += 1.0;
            const double bounce_now = bounce;
            bounce = 0;
            const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
            double sinkage = 0;
            for (int i = 0; i < 4; ++i) {
                const ChVector3d w = viper.GetWheels()[i]->GetBody()->GetPos();
                sinkage += 0.25 * (height_at(w.x(), w.y()) + wheel_radius - w.z());
            }
            std::cout << "t = " << sys.GetChTime() << " s (" << sys.GetChTime() / wall << "x real time), at (" << pos.x() << ", " << pos.y()
                      << "), speed " << chassis->GetPosDt().Length() << " m/s, bounce " << std::sqrt(bounce_now) * 100 << " cm/s, distance " << distance << " m, window moves " << soil.GetNumMoves()
                      << ", " << soil.GetTerrain().GetNumSPHParticles() << " particles, wheel sinkage " << sinkage * 100 << " cm, "
                      << ruts->GetNumNodes() << " rut nodes, " << soil.GetNumLooseParticles() << " loose, publish " << publish_ms / std::max(publishes, 1) << " ms" << std::endl;
        }
    }
    return 0;
}
