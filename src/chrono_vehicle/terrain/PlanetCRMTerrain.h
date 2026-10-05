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
// CRM soil over a window of a planet site, taken from and given back to the
// ground the rest of the scene draws: a height field with ruts in a
// planet::ChDeformationFilter.
//
// =============================================================================

#ifndef PLANET_CRM_TERRAIN_H
#define PLANET_CRM_TERRAIN_H

#include <functional>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/filters/ChDeformationFilter.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// CRM terrain (SPH continuum soil, Chrono::FSI) over a rectangle of a planet site, taken from and given back to
/// ground the rest of the scene draws: a height field with ruts in a planet::ChDeformationFilter where a rover drives
/// (ConstructFromHeight, PublishToDeformation; see PlanetCRMWindow for a window that follows the rover).
///
/// Positions are in the site frame, z up. Set the soil and SPH parameters, add the bodies that touch the soil
/// (AddRigidBody), construct from the ground's heights, and Initialize, as for any CRMTerrain.
class CH_VEHICLE_API PlanetCRMTerrain : public CRMTerrain {
  public:
    /// CRM soil of the given particle spacing (m).
    PlanetCRMTerrain(ChSystem& sys, double spacing);
    ~PlanetCRMTerrain();

    /// Room above the highest ground in the window, and around the window, that the soil may be carried to (m,
    /// defaults: 2 and 1): the computational domain of the soil model. Particles that leave it drop out of the
    /// simulation, so it must hold what the wheels throw up. Call before ConstructFromHeight.
    void SetHeadroom(double above, double around) {
        m_headroom = above;
        m_margin = around;
    }

    /// Fill `window` with particles below the ground a height function gives (site x/y to z, m), down to `depth` (m)
    /// below the lowest ground in it, with boundary markers under that floor and, for `side_flags`
    /// (fsi::sph::BoxSide), walls on the window's sides. Each column of particles is shifted up or down by less
    /// than half a spacing so the soil's surface lies at the ground, not below it by up to a spacing.
    void ConstructFromHeight(const std::function<double(double x, double y)>& height,
                             const planet::ChSiteRegion& window,
                             double depth,
                             int side_flags = fsi::sph::BoxSide::Z_NEG);

    /// State of a soil particle, to carry soil over from one soil model to another.
    struct ParticleState {
        ChVector3d pos;
        ChVector3d vel;
        double rho;
        double pressure;
        double mu;
        ChVector3d tau_diag;
        ChVector3d tau_offdiag;
    };

    /// The state of the soil's particles.
    std::vector<ParticleState> GetParticleStates() const;

    /// Soil to take over, as it is, where it lies over `region` (site x/y): those particles are seeded with their
    /// state in place of the lattice's, which then fills only the columns under them. Soil carried over from a window
    /// being replaced keeps the stress and compaction a rover has given it, so the rover does not sink into fresh soil
    /// when the window moves. Call before ConstructFromHeight.
    void SetCarriedParticles(std::vector<ParticleState> particles, const planet::ChSiteRegion& region);

    /// Seed the soil carrying its own weight: each particle starts at the pressure of the soil above it, up to the
    /// ground a height function gives (ConstructFromHeight), instead of at rest with no stress, which leaves the top
    /// layers unconfined and weak until they settle. Call before ConstructFromHeight (default: false).
    void SetInitialOverburden(bool val) { m_overburden = val; }

    /// Write how far the soil's top has moved since the window was seeded into a deformation filter, over the window,
    /// added to the height changes the filter held there then: ruts from before carry over, and soil the particles did
    /// not move shows no change. The first call, which should come right after Initialize, only records where the
    /// soil starts. Returns the nodes changed by more than `tolerance` (m).
    size_t PublishToDeformation(planet::ChDeformationFilter& filter, double tolerance = 5e-4);

    /// Scale the soil forces applied to the bodies in contact (default: 1). The soil still moves as the bodies press
    /// into it; only the forces it applies to them are scaled. Used to hand the bodies over to another soil model
    /// gradually.
    void SetForceScale(double scale) { m_force_scale = scale; }
    double GetForceScale() const { return m_force_scale; }

    /// The soil's wrench on a body in contact at the last exchange, before the force scale: force at the body's center
    /// of mass and torque about it, both in the absolute frame. Returns false for a body not added to the soil.
    bool GetSoilWrench(const ChBody& body, ChVector3d& force, ChVector3d& torque) const;

    const planet::ChSiteRegion& GetWindow() const { return m_window; }
    /// Height of the floor of the soil (m).
    double GetFloor() const { return m_floor; }

  private:
    // Initial state of the particles: carried over, or at rest (with the soil's weight above, if m_overburden)
    class SeedCallback : public fsi::sph::ChFsiFluidSystemSPH::ParticlePropertiesCallback {
      public:
        explicit SeedCallback(const PlanetCRMTerrain& terrain) : m_terrain(terrain) {}
        virtual void set(const fsi::sph::ChFsiFluidSystemSPH& sysSPH, const ChVector3d& pos) override;

      private:
        const PlanetCRMTerrain& m_terrain;
    };

    // Advances the multibody system over an exchange step with the soil forces on the bodies scaled
    class ScaledAdvance : public fsi::ChFsiSystem::MBDCallback {
      public:
        explicit ScaledAdvance(PlanetCRMTerrain& terrain) : m_terrain(terrain) {}
        virtual void Advance(double step, double threshold) override;

      private:
        PlanetCRMTerrain& m_terrain;
    };

    virtual ChVector3d Grid2Point(const ChVector3i& p) override;
    static int64_t CarriedKey(const ChVector3d& pos);

    void Build(const planet::ChSiteRegion& window,
               double depth,
               int side_flags,
               const std::function<double(double, double)>& top_at,
               const std::function<bool(const ChVector3d&)>& soil_at);

    double m_force_scale = 1;         // scale of the soil forces on the bodies
    std::unordered_map<const ChBody*, std::pair<ChVector3d, ChVector3d>> m_wrenches;  // unscaled, absolute
    int m_nx = 0, m_ny = 0;           // lattice columns over the window
    int m_k_floor = 0;                // lattice level of the floor
    std::vector<double> m_shift;      // height shift of each column's particles (ConstructFromHeight)
    std::vector<double> m_column_ground;     // ground over each column (ConstructFromHeight)
    bool m_overburden = false;
    std::vector<ParticleState> m_carried;               // soil taken over as it is (SetCarriedParticles)
    planet::ChSiteRegion m_carried_region;
    std::unordered_map<int64_t, size_t> m_carried_index;  // carried particles by quantized position
    static constexpr int kCarried = -(1 << 29);          // lattice x index marking a carried particle (y: its index)
    std::vector<float> m_reference;   // top of the soil over each column when first published
    std::vector<float> m_base;        // height changes of the filter's nodes over the window then
    planet::ChSiteRegion m_window;
    double m_floor = 0;
    double m_headroom = 2.0;
    double m_margin = 1.0;
};

/// @} vehicle_terrain

}  // namespace vehicle
}  // namespace chrono

#endif
