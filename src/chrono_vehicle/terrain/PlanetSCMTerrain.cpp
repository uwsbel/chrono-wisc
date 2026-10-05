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

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "chrono_vehicle/terrain/PlanetSCMTerrain.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// PlanetSCMHeightFunctor
// -----------------------------------------------------------------------------

PlanetSCMHeightFunctor::PlanetSCMHeightFunctor(std::shared_ptr<const planet::ChPlanetSurface> surface,
                                               const planet::ChSiteFrame& site)
    : m_surface(std::move(surface)), m_site(site) {}

double PlanetSCMHeightFunctor::GetInitHeight(const ChVector2i& loc, double delta) {
    return HeightOf(loc, delta);
}

double PlanetSCMHeightFunctor::HeightOf(const ChVector2i& loc, double delta) const {
    const std::int64_t k = Key(loc);
    Shard& shard = m_shards[ShardOf(k)];
    {
        std::lock_guard<std::mutex> lock(shard.mutex);
        auto it = shard.heights.find(k);
        if (it != shard.heights.end())
            return it->second;
    }
    // Evaluate outside the lock: the surface sample is the expensive part and is thread-safe.
    double lon, lat;
    m_site.ToLonLat(loc.x() * delta, loc.y() * delta, lon, lat);
    const double h = m_surface->GetElevation(lon, lat) - m_site.GetOriginElevation();
    std::lock_guard<std::mutex> lock(shard.mutex);
    shard.heights.emplace(k, h);
    return h;
}

void PlanetSCMHeightFunctor::GetInitHeights(const std::vector<ChVector2i>& locs,
                                            double delta,
                                            std::vector<double>& out) const {
    out.resize(locs.size());
    for (std::size_t i = 0; i < locs.size(); ++i)
        out[i] = HeightOf(locs[i], delta);
}

std::size_t PlanetSCMHeightFunctor::GetNumCachedNodes() const {
    std::size_t n = 0;
    for (const auto& shard : m_shards) {
        std::lock_guard<std::mutex> lock(shard.mutex);
        n += shard.heights.size();
    }
    return n;
}

// -----------------------------------------------------------------------------
// PlanetSCMTerrain
// -----------------------------------------------------------------------------

PlanetSCMTerrain::PlanetSCMTerrain(ChSystem* system,
                                   std::shared_ptr<const planet::ChPlanetSurface> surface,
                                   const planet::ChSiteFrame& site)
    : SCMTerrain(system, false),
      m_site(site),
      m_functor(std::make_shared<PlanetSCMHeightFunctor>(std::move(surface), site)) {}

PlanetSCMTerrain::~PlanetSCMTerrain() {
    m_stop.store(true, std::memory_order_release);
    // Notify under the mutex so the worker between its predicate test and wait cannot miss the wake.
    {
        std::lock_guard<std::mutex> lock(m_stats_mutex);
        m_stats_cv.notify_all();
    }
    if (m_stats_thread.joinable())
        m_stats_thread.join();
}

void PlanetSCMTerrain::SetDeformationFilter(std::shared_ptr<planet::ChDeformationFilter> filter) {
    if (filter) {
        const planet::ChSiteFrame& fs = filter->GetSiteFrame();
        const bool same_site = fs.GetRadius() == m_site.GetRadius() &&
                               fs.GetOriginLongitude() == m_site.GetOriginLongitude() &&
                               fs.GetOriginLatitude() == m_site.GetOriginLatitude() &&
                               fs.GetOriginElevation() == m_site.GetOriginElevation();
        if (!same_site || std::abs(filter->GetSpacing() - m_params.delta) > 1e-12)
            throw std::invalid_argument(
                "PlanetSCMTerrain::SetDeformationFilter: the filter must use this terrain's site frame and grid spacing");
    }
    m_deformation = std::move(filter);
}

std::shared_ptr<planet::ChDeformationFilter> PlanetSCMTerrain::MakeDeformationFilter() {
    auto filter = std::make_shared<planet::ChDeformationFilter>(m_site, m_params.delta);
    SetDeformationFilter(filter);
    return filter;
}

std::size_t PlanetSCMTerrain::PublishDeformation() {
    if (!m_deformation)
        return 0;
    // With each node's height (site z is height above the origin's elevation), so the drawn terrain shows
    // compacted ground as the soil model has it.
    const auto nodes = GetModifiedNodes(true);
    std::vector<planet::ChDeformationFilter::Node> changes;
    changes.reserve(nodes.size());
    for (const auto& [loc, level] : nodes)
        changes.push_back({loc, level - m_functor->GetInitHeight(loc, m_params.delta), level + m_site.GetOriginElevation()});
    return m_deformation->SetNodes(changes);
}

void PlanetSCMTerrain::Initialize(const Params& params, const std::vector<Wheel>& wheels) {
    if (wheels.empty())
        throw std::invalid_argument(
            "PlanetSCMTerrain::Initialize: no wheels; SCM would ray-cast the AABB of every collision shape.");

    m_params = params;

    SetSoilParameters(m_params.bekker_kphi, m_params.bekker_kc, m_params.bekker_n, m_params.mohr_cohesion,
                      m_params.mohr_friction, m_params.janosi_shear, m_params.elastic_k, m_params.damping_r);
    SetTestHeight(m_params.test_height);
    if (m_params.bulldozing) {
        EnableBulldozing(true);
        SetBulldozingParameters(m_params.erosion_angle);
    }

    // Active domains are registered before Initialize. The box is in the wheel's reference frame with full
    // dimensions: the wheel spins about its local y, so x and z span the diameter and y the width.
    for (const auto& w : wheels) {
        const double d = 2.0 * w.radius + 2.0 * m_params.domain_pad;
        const double b = w.width + 2.0 * m_params.domain_pad;
        AddActiveDomain(w.body, ChVector3d(0, 0, 0), ChVector3d(d, b, d));
    }

    SCMTerrain::Initialize(m_functor, m_params.delta);

    // The statistics worker starts only once the grid exists.
    m_stats_thread = std::thread([this] { StatsLoop(); });
}

void PlanetSCMTerrain::SetSoil(const Params& params) {
    // Grid geometry is fixed at Initialize; keep ours.
    const double delta = m_params.delta;
    const double domain_pad = m_params.domain_pad;
    m_params = params;
    m_params.delta = delta;
    m_params.domain_pad = domain_pad;

    SetSoilParameters(m_params.bekker_kphi, m_params.bekker_kc, m_params.bekker_n, m_params.mohr_cohesion,
                      m_params.mohr_friction, m_params.janosi_shear, m_params.elastic_k, m_params.damping_r);
    SetTestHeight(m_params.test_height);
    EnableBulldozing(m_params.bulldozing);
    if (m_params.bulldozing)
        SetBulldozingParameters(m_params.erosion_angle);
}

double PlanetSCMTerrain::GetContactForce(const std::shared_ptr<ChBody>& body) const {
    ChVector3d force, torque;
    if (!GetContactForceBody(body, force, torque))
        return 0.0;
    return force.Length();
}

// -----------------------------------------------------------------------------
// Rut statistics. The walk grows with the ground driven over, so only the snapshot happens on the
// simulation thread (Chrono's node map is read between steps); the arithmetic runs on the worker.
// -----------------------------------------------------------------------------

void PlanetSCMTerrain::RequestRutStats() {
    {
        std::lock_guard<std::mutex> lock(m_stats_mutex);
        if (m_stats_in_flight)
            return;
        m_stats_in_flight = true;
    }

    auto nodes = GetModifiedNodes(true);

    std::lock_guard<std::mutex> lock(m_stats_mutex);
    if (nodes.empty()) {
        // An empty job would never wake the worker and would leave the in-flight flag stuck.
        m_stats_result = RutStats{};
        m_stats_ready = true;
        m_stats_in_flight = false;
        return;
    }
    m_stats_job = std::move(nodes);
    m_stats_cv.notify_one();
}

std::optional<PlanetSCMTerrain::RutStats> PlanetSCMTerrain::TakeRutStats() {
    std::lock_guard<std::mutex> lock(m_stats_mutex);
    if (!m_stats_ready)
        return std::nullopt;
    m_stats_ready = false;
    return m_stats_result;
}

void PlanetSCMTerrain::StatsLoop() {
    std::vector<SCMTerrain::NodeLevel> job;
    std::vector<ChVector2i> locs;
    std::vector<double> init;
    const double delta = m_params.delta;

    while (true) {
        {
            std::unique_lock<std::mutex> lock(m_stats_mutex);
            m_stats_cv.wait(lock, [this] { return !m_stats_job.empty() || m_stop.load(std::memory_order_acquire); });
            if (m_stop.load(std::memory_order_acquire))
                return;
            job.swap(m_stats_job);
            m_stats_job.clear();
        }

        RutStats st;
        st.nodes = job.size();
        st.area_m2 = static_cast<double>(st.nodes) * delta * delta;

        locs.clear();
        locs.reserve(job.size());
        for (const auto& nl : job)
            locs.push_back(nl.first);
        m_functor->GetInitHeights(locs, delta, init);

        double sum = 0.0;
        for (std::size_t v = 0; v < job.size(); ++v) {
            const double d = init[v] - job[v].second;  // undeformed minus current level
            if (d <= 0.0)
                continue;  // untouched ground, or a bulldozed rim
            sum += d;
            ++st.deformed;
            if (d > st.max_depth_m)
                st.max_depth_m = d;
        }
        st.mean_depth_m = st.deformed ? sum / st.deformed : 0.0;
        job.clear();

        std::lock_guard<std::mutex> lock(m_stats_mutex);
        m_stats_result = st;
        m_stats_ready = true;
        m_stats_in_flight = false;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
