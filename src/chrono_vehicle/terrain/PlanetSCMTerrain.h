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
// Deformable planetary terrain: Chrono's SCM soil model over a ChPlanetSurface,
// with the undeformed height supplied procedurally so the patch is unbounded.
//
// =============================================================================

#ifndef PLANET_SCM_TERRAIN_H
#define PLANET_SCM_TERRAIN_H

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/filters/ChDeformationFilter.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Undeformed SCM height from a ChPlanetSurface, memoized per grid node.
/// SCM queries this from inside an OpenMP ray-casting loop, so the memo is sharded by node hash and
/// each shard has its own lock; a surface sample costs a full DEM plus crater evaluation.
class CH_VEHICLE_API PlanetSCMHeightFunctor : public SCMTerrain::HeightFunctor {
  public:
    PlanetSCMHeightFunctor(std::shared_ptr<const planet::ChPlanetSurface> surface, const planet::ChSiteFrame& site);

    /// Undeformed height at a grid node (SCM callback; reentrant).
    virtual double GetInitHeight(const ChVector2i& loc, double delta) override;

    /// Undeformed heights for many nodes at once, in the order given.
    void GetInitHeights(const std::vector<ChVector2i>& locs, double delta, std::vector<double>& out) const;

    /// Number of nodes currently memoized.
    std::size_t GetNumCachedNodes() const;

  private:
    double HeightOf(const ChVector2i& loc, double delta) const;

    static constexpr int kShards = 64;
    struct Shard {
        mutable std::mutex mutex;
        std::unordered_map<std::int64_t, double> heights;
    };

    static std::int64_t Key(const ChVector2i& loc) {
        return (static_cast<std::int64_t>(loc.x()) << 32) | static_cast<std::uint32_t>(loc.y());
    }
    static int ShardOf(std::int64_t k) {
        const std::uint64_t mixed = static_cast<std::uint64_t>(k) * 0x9E3779B97F4A7C15ull;
        return static_cast<int>((mixed >> 58) & (kShards - 1));
    }

    std::shared_ptr<const planet::ChPlanetSurface> m_surface;
    planet::ChSiteFrame m_site;
    mutable std::array<Shard, kShards> m_shards;
};

/// SCM terrain whose undeformed surface is a ChPlanetSurface in a site frame.
/// The SCM reference frame is the site ENU frame, so grid coordinates are site meters and the
/// terrain never needs re-centering; ruts persist for the whole run. Only the registered wheels
/// interact with the soil, so there is no collision mesh for anything else to rest on.
class CH_VEHICLE_API PlanetSCMTerrain : public SCMTerrain {
  public:
    /// Grid and soil settings.
    struct Params {
        double delta = 0.05;  ///< SCM grid spacing (m)

        double bekker_kphi = 820000.0;  ///< frictional modulus (Pa/m^n)
        double bekker_kc = 1400.0;      ///< cohesive modulus (Pa/m^(n-1))
        double bekker_n = 1.0;          ///< sinkage exponent
        double mohr_cohesion = 170.0;   ///< cohesion (Pa)
        double mohr_friction = 35.0;    ///< internal friction angle (deg)
        double janosi_shear = 0.018;    ///< Janosi shear displacement (m)
        double elastic_k = 2e8;         ///< elastic stiffness (Pa/m), must exceed bekker_kphi
        double damping_r = 3e4;         ///< vertical damping (Pa s/m)

        bool bulldozing = false;      ///< enable bulldozing (soil displacement)
        double erosion_angle = 40.0;  ///< bulldozing erosion angle (deg)

        double domain_pad = 0.10;   ///< margin added to each wheel's active domain box (m)
        double test_height = 0.10;  ///< SCM ray test height above the undeformed surface (m)

    };

    /// A wheel body whose footprint SCM samples.
    struct Wheel {
        std::shared_ptr<ChBody> body;  ///< wheel body, spinning about its local y axis
        double radius;                 ///< wheel radius (m)
        double width;                  ///< wheel width (m)
    };

    /// Deformation summary over every node SCM has touched.
    struct RutStats {
        std::size_t nodes = 0;     ///< grid nodes touched since t = 0
        std::size_t deformed = 0;  ///< of those, nodes below the undeformed level
        double area_m2 = 0.0;      ///< nodes times delta squared
        double max_depth_m = 0.0;  ///< deepest depression below the undeformed level
        double mean_depth_m = 0.0; ///< mean depth over depressed nodes
    };

    /// Construct the terrain over a surface in the given site frame (no visualization mesh).
    PlanetSCMTerrain(ChSystem* system,
                     std::shared_ptr<const planet::ChPlanetSurface> surface,
                     const planet::ChSiteFrame& site);
    ~PlanetSCMTerrain();

    /// Initialize SCM with the given settings and wheel active domains.
    /// Throws std::invalid_argument if no wheels are given.
    void Initialize(const Params& params, const std::vector<Wheel>& wheels);

    /// Re-apply the soil parameters to the live terrain (grid settings are fixed).
    void SetSoil(const Params& params);

    /// Current settings.
    const Params& GetParams() const { return m_params; }

    /// Magnitude of the soil force on a body, or zero if it is not in contact.
    double GetContactForce(const std::shared_ptr<ChBody>& body) const;

    /// Ask for a deformation summary, computed on a worker thread.
    /// A request that arrives while one is in flight is dropped.
    void RequestRutStats();

    /// Take the last completed summary, if any.
    std::optional<RutStats> TakeRutStats();


    /// Number of undeformed heights memoized so far.
    std::size_t GetNumCachedNodes() const { return m_functor->GetNumCachedNodes(); }

    /// Publish the soil deformation into a filter, so a renderer drawing a view of the surface with it
    /// (see planet::ChPlanetSurface::CreateView) shows the ruts. The filter must be on this terrain's site
    /// frame with the SCM grid spacing; MakeDeformationFilter returns one. Throws std::invalid_argument
    /// otherwise. Call after Initialize.
    void SetDeformationFilter(std::shared_ptr<planet::ChDeformationFilter> filter);

    /// A deformation filter matching this terrain's site frame and grid, already set on it.
    std::shared_ptr<planet::ChDeformationFilter> MakeDeformationFilter();

    /// Copy the current sinkage and height of every node SCM has touched into the deformation filter, if one
    /// is set: the drawn terrain then shows compacted ground as SCM has it, flattened by the wheels.
    /// Call it as often as the drawn terrain should refresh, for example once per rendered frame.
    /// Returns the number of nodes whose height changed by more than a millimeter.
    std::size_t PublishDeformation();

    /// The height functor, for consumers that sample the undeformed surface directly.
    std::shared_ptr<PlanetSCMHeightFunctor> GetHeightFunctor() const { return m_functor; }

  private:
    void StatsLoop();

    Params m_params;
    planet::ChSiteFrame m_site;
    std::shared_ptr<PlanetSCMHeightFunctor> m_functor;
    std::shared_ptr<planet::ChDeformationFilter> m_deformation;

    std::atomic<bool> m_stop{false};

    std::thread m_stats_thread;
    std::mutex m_stats_mutex;
    std::condition_variable m_stats_cv;
    std::vector<SCMTerrain::NodeLevel> m_stats_job;
    bool m_stats_in_flight = false;
    RutStats m_stats_result;
    bool m_stats_ready = false;


};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
