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
// CRM soil in a window of a planet's surface that follows a rover, leaving
// ruts behind in a deformation filter.
//
// =============================================================================

#ifndef PLANET_CRM_WINDOW_H
#define PLANET_CRM_WINDOW_H

#include <functional>
#include <cstdint>
#include <memory>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/terrain/PlanetCRMTerrain.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/filters/ChDeformationFilter.h"
#include "chrono_vehicle/terrain/ChDustField.h"
#include "chrono_planet/geometry/ChSdfMesher.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// CRM soil (vehicle::PlanetCRMTerrain) over a window of a planet surface that follows a body, such as a rover's
/// chassis, over ground of any extent. The soil is seeded from the surface with the ruts a deformation filter holds, and
/// the ruts it makes go back into the filter (Publish), where the drawn terrain shows them and the next window seeds
/// from them. When the body comes within a margin of the window's edge, the ruts are published, the window is dropped
/// and a new one is seeded around the body, ahead of it along its motion. Where the windows overlap, the new window
/// takes the old one's particles over as they are, with their velocities and stresses; fresh soil at rest fills the
/// rest.
///
/// A setup function configures each window before it is seeded: soil and SPH parameters, the bodies that touch the
/// soil (AddRigidBody, such as the wheels), and their active domains. Advance the coupled system with Advance, which
/// moves the window when it has to, instead of stepping the multibody system.
class CH_VEHICLE_API PlanetCRMWindow {
  public:
    /// Configures a new window before it is seeded.
    using Setup = std::function<void(PlanetCRMTerrain& terrain)>;

    /// CRM soil of the given particle spacing (m) over `surface` in the `site` frame, with ruts in `ruts`.
    PlanetCRMWindow(ChSystem& sys,
                    std::shared_ptr<const planet::ChPlanetSurface> surface,
                    const planet::ChSiteFrame& site,
                    std::shared_ptr<planet::ChDeformationFilter> ruts,
                    double spacing,
                    Setup setup);
    ~PlanetCRMWindow();

    /// Size of the window (m, defaults 4 x 3), depth of soil below its lowest ground (m, default 0.25), and how close to
    /// its edge the followed body may come before it moves (m, default 0.8). Call before Initialize.
    void SetWindow(double length, double width, double depth, double margin);

    /// Draw ruts from a wheel's footprint instead of the soil's particles: after every step, the ground under the
    /// wheel, a cylinder of the given radius and width (m) about `axis` through the origin of the wheel body's reference
    /// frame, is lowered to the wheel's bottom where the wheel sits below it, with walls sloping out from its edges at
    /// 35 degrees. The rut is then as deep as the wheel sank into the soil, and as sharp as the deformation filter's
    /// spacing, whatever the particle spacing; its floor carries the wheel's heights, so the drawn terrain's finer relief
    /// is pressed flat there (planet::ChDeformationFilter::SetFlattenDepth); soil the wheels push aside shows only as berms (SetBerms). Once a wheel
    /// is added, Publish writes the footprints alone.
    void AddWheel(std::shared_ptr<ChBody> wheel, double radius, double width, const ChVector3d& axis = ChVector3d(0, 1, 0));

    /// Seed the first window around a body to follow.
    void Initialize(std::shared_ptr<ChBody> follow);

    /// Advance the coupled system by `step` (s), first moving the window if the followed body nears its edge.
    void Advance(double step);

    /// Move the window now, centered on `center` (site x/y).
    void MoveTo(const ChVector2d& center) {
        Seed(center);
        ++m_moves;
    }

    /// Write the ruts into the deformation filter. Returns the nodes changed.
    size_t Publish();

    /// Raise the ground where the wheels pile soil up, from the soil's surface, at each Publish (with wheels added):
    /// berms along the ruts and soil pushed ahead of the wheels (default: false).
    void SetBerms(bool val) { m_berms = val; }

    /// Scale the soil forces applied to the wheels (default: 1; see PlanetCRMTerrain::SetForceScale). Holds for the
    /// windows seeded later as the window moves.
    void SetForceScale(double scale);

    /// Soil a wheel threw up since the previous call: mass (kg), and the mean and spread (1 sigma per axis) of where
    /// it was and how fast it moved when caught in free flight.
    struct Ejecta {
        double mass = 0;
        ChVector3d pos;
        ChVector3d vel;
        double vel_spread = 0;
    };

    /// Measure the soil each wheel (AddWheel) throws up: particles in free flight, as EmitDust finds them, each counted
    /// once, attributed to the nearest wheel within reach. Call as often as EmitDust would be. A dust model can then
    /// emit that mass smoothly (ChDustField::EmitSource) instead of in particle-sized lumps.
    std::vector<Ejecta> MeasureEjecta(double time, double min_speed = 0.05);

    /// Hand soil thrown up to a dust field: particles in free flight (clear of the drawn ground, moving faster than
    /// `min_speed` (m/s), and accelerated by gravity alone since the previous call), each once, their mass shared among
    /// the dust's grain sizes and among `splits` super-particles, released along its path since the previous call with
    /// a spread of speeds and directions. Call often enough to catch particles in flight (every 10 ms, say). Returns the
    /// particles handed over.
    size_t EmitDust(ChDustField& dust, double time, double min_speed = 0.05, int splits = 64);

    /// Mesh the loose soil: the particles whose centers rise above the drawn ground (the surface with the ruts the filter
    /// holds), such as soil thrown up or pushed aside by the wheels, as clods, spheres of about the particle spacing
    /// barely blended (planet::ChSparseSdfGrid). Call after Publish, and draw the mesh (site
    /// coordinates) with planet::ChMeshVisualizationVSG. Returns the new mesh.
    std::shared_ptr<ChTriangleMeshConnected> UpdateLooseSoil();

    /// Number of loose particles at the last UpdateLooseSoil.
    size_t GetNumLooseParticles() const { return m_num_loose; }

    PlanetCRMTerrain& GetTerrain() { return *m_terrain; }
    const planet::ChSiteRegion& GetWindow() const { return m_terrain->GetWindow(); }
    int GetNumMoves() const { return m_moves; }

    /// Ground height (m) at site x/y the next window would be seeded to: the surface with the ruts.
    double GetHeight(double x, double y) const;

  private:
    struct Wheel {
        std::shared_ptr<ChBody> body;
        double radius;
        double width;
        ChVector3d axis;
    };

    void Seed(const ChVector2d& center);
    // Soil surface over the window's lattice
    struct Surface {
        std::vector<float> top;
        int nx = 0, ny = 0;
        planet::ChSiteRegion window;
        bool Sample(double x, double y, double spacing, double& z) const;
    };

    Surface SoilSurface(const std::vector<ChVector3d>& positions) const;
    double GetSeedHeight(double x, double y) const;
    void RaiseBerms();
    void Stamp(const Wheel& wheel);
    double GetGround(const ChVector2i& node);

    ChSystem& m_sys;
    std::shared_ptr<const planet::ChPlanetSurface> m_surface;
    planet::ChSiteFrame m_site;
    std::shared_ptr<planet::ChDeformationFilter> m_ruts;
    double m_spacing;
    Setup m_setup;
    std::shared_ptr<ChBody> m_follow;
    double m_length = 4, m_width = 3, m_depth = 0.25, m_margin = 0.8;
    std::unique_ptr<PlanetCRMTerrain> m_terrain;
    int m_moves = 0;
    std::vector<Wheel> m_wheels;
    std::unordered_map<int64_t, double> m_ground;   // surface height at filter nodes under the wheels
    // Node height changes stamped since the last Publish, with the heights they leave (above the reference sphere;
    // NaN where they raise the ground)
    struct Pending {
        double delta;
        double height;
    };
    std::unordered_map<int64_t, Pending> m_pending;
    std::vector<float> m_columns;                   // surface height over the window's lattice columns
    int m_columns_nx = 0, m_columns_ny = 0;
    ChVector2d m_columns_origin;
    std::unique_ptr<planet::ChSparseSdfGrid> m_loose;
    Surface m_carried;                              // soil surface of the window being replaced
    bool m_berms = false;
    double m_force_scale = 1;
    std::unordered_set<size_t> m_emitted;           // particles handed to a dust field
    std::vector<ChVector3d> m_prev_velocities;      // particle velocities at the previous EmitDust
    double m_prev_time = 0;
    std::mt19937 m_rng{7};
    size_t m_num_loose = 0;
};

/// @} vehicle_terrain

}  // namespace vehicle
}  // namespace chrono

#endif
