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
#include <unordered_set>

#include "chrono_planet/lod/ChPlanetQuadtree.h"
#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/core/SphereMath.h"
#include "chrono_planet/lod/QuadtreeTile.h"
#include "chrono_planet/lod/SphericalCoordinates.h"

namespace chrono {
namespace planet {

using Traits = CoordinateTraits<Spherical>;

namespace {

// The active ring widens by one tile per this much altitude.
constexpr double kAltitudePerExtraTileM = 10000.0;
// Ring half-width bounds in tiles.
constexpr int kMinRingTiles = 2, kMaxRingTiles = 90;
// Keeps root add/remove events distinct from per-tile mesh changes in GetMeshSetVersion().
constexpr unsigned long long kRootVersionStride = 1000003ULL;

}  // namespace

// The streaming state: the root ring, the per-root trees.
struct ChPlanetQuadtree::Impl {
    Impl(std::shared_ptr<const ChPlanetSurface> surface, int view_range)
        : m_surface(std::move(surface)),
          m_radius(m_surface->GetBody().GetRadius()),
          m_tile_size(m_surface->GetRootTileSize()),
          m_view_range(view_range) {
        // Changes made before the quadtree existed are already in the tiles it builds.
        ChGeoRegion ignored;
        m_seen_version = m_surface->GetFilterChain()->GetChanges(0, ignored);
    }

    void Update(const std::function<util::Vec3(double)>& pose_at, double sim_time);
    void Step(const util::Vec3& camera);
    int Invalidate(const ChGeoRegion& region);
    void PollChanges();

    std::shared_ptr<const ChPlanetSurface> m_surface;
    double m_radius;          // body radius
    double m_tile_size;       // root tile side, degrees
    int m_view_range;         // ring half-width in tiles at ground level
    double m_split_tiles = 1.0;       // split distance, in widths of the tile
    double m_horizon_margin = 1000.0; // terrain depth assumed for horizon tests (m), < 0: off
    int m_min_zoom = 0;               // tiles above the horizon split to at least this zoom
    int m_max_zoom = 17;              // tiles never split past this zoom

    LodParams Lod() const {
        const double root_width = m_tile_size * util::kPi / 180.0 * m_radius;
        return {m_split_tiles * root_width, m_horizon_margin, m_radius, m_min_zoom, m_max_zoom};
    }
    std::uint64_t m_seen_version = 0;  // latest filter change already in the tiles
    long long m_rebuilt = 0;           // tiles rebuilt for changes
    util::Vec3 m_camera;      // position of the last tick
    bool m_have_tick = false;
    long long m_tick = 0;
    double m_prev_time = 0;   // previous Update() call, for interpolation
    util::Vec3 m_prev_position;
    std::unordered_map<TileKey, std::unique_ptr<QuadtreeTile<Spherical>>> m_tiles;
    unsigned long long m_root_version = 0;  // bumped when a root tile is added or removed
};

namespace {
util::Vec3 ToVec3(const ChVector3d& v) {
    return {v.x(), v.y(), v.z()};
}
}  // namespace

ChPlanetQuadtree::ChPlanetQuadtree(std::shared_ptr<const ChPlanetSurface> surface, int view_range_tiles) {
    if (!surface)
        throw std::invalid_argument("ChPlanetQuadtree: null surface");
    m_impl = std::make_unique<Impl>(std::move(surface), view_range_tiles);
}

ChPlanetQuadtree::~ChPlanetQuadtree() = default;

void ChPlanetQuadtree::Update(const PoseFn& pose_at, double sim_time) {
    m_impl->Update([&](double t) { return ToVec3(pose_at(t)); }, sim_time);
}

void ChPlanetQuadtree::Impl::Update(const std::function<util::Vec3(double)>& pose_at, double sim_time) {
    // The tick this time falls in. The epsilon keeps t = k / 60 exactly on tick k.
    const long long tick = static_cast<long long>(std::floor(sim_time / kTickS + 1e-6));
    const bool restart = !m_have_tick || sim_time < m_prev_time;  // first call, or the sim clock was reset
    if (restart) {
        m_have_tick = true;
    }
    // Step every tick since the last call, or only this one after a restart.
    for (long long k = restart ? tick : m_tick + 1; k <= tick; ++k) {
        Step(pose_at(k * kTickS));
    }
    m_tick = tick;
    m_prev_time = sim_time;
    PollChanges();
}

void ChPlanetQuadtree::Impl::PollChanges() {
    ChGeoRegion changed;
    const std::uint64_t latest = m_surface->GetFilterChain()->GetChanges(m_seen_version, changed);
    if (latest > m_seen_version) {
        m_seen_version = latest;
        Invalidate(changed);
    }
}

int ChPlanetQuadtree::Impl::Invalidate(const ChGeoRegion& region) {
    if (region.IsEmpty())
        return 0;
    int rebuilt = 0;
    for (auto& p : m_tiles)
        rebuilt += p.second->invalidate(region.min_lon, region.min_lat, region.max_lon, region.max_lat, region.max_spacing);
    m_rebuilt += rebuilt;
    return rebuilt;
}

void ChPlanetQuadtree::Update(const ChVector3d& camera, double sim_time) {
    Impl& m = *m_impl;
    const double t0 = m.m_prev_time;
    const util::Vec3 from = m.m_prev_position, to = ToVec3(camera);
    const bool have = m.m_have_tick && sim_time > t0;
    m.Update(
        [&](double t) {
            const double u = have ? std::clamp((t - t0) / (sim_time - t0), 0.0, 1.0) : 1.0;
            return util::Vec3{util::lerp(from.x, to.x, u), util::lerp(from.y, to.y, u), util::lerp(from.z, to.z, u)};
        },
        sim_time);
    m.m_prev_position = to;
}

void ChPlanetQuadtree::Impl::Step(const util::Vec3& camera) {
    m_camera = camera;

    const util::LonLat cam = util::lonLatOf(camera.x, camera.y, camera.z);
    const auto [centreX, centreY] = Traits::computeTileIndices({cam.lon, cam.lat}, m_tile_size);

    // Widen the active ring with altitude.
    const int extraTiles = static_cast<int>(std::ceil(Traits::elevationOf(camera.x, camera.y, camera.z, m_radius) / kAltitudePerExtraTileM));
    const int range = std::clamp(m_view_range + extraTiles, kMinRingTiles, kMaxRingTiles);

    const int nLon = static_cast<int>(std::round(360.0 / m_tile_size));
    const int nLat = static_cast<int>(std::round(180.0 / m_tile_size));

    std::unordered_set<TileKey> needed;
    for (int dy = -range; dy <= range; ++dy) {
        for (int dx = -range; dx <= range; ++dx) {
            const TileKey key{((centreX + dx) % nLon + nLon) % nLon, std::clamp(centreY + dy, 0, nLat - 1)};
            needed.insert(key);
            if (m_tiles.count(key)) {
                continue;
            }
            const auto [cLon, cLat] = Traits::tileCenterPosition(key, m_tile_size);
            const double half = m_tile_size * 0.5;
            m_tiles[key] = std::make_unique<QuadtreeTile<Spherical>>(Spherical::Boundary{cLon, cLat, half, half}, m_surface);
            m_root_version++;
        }
    }

    for (auto it = m_tiles.begin(); it != m_tiles.end();) {
        if (needed.count(it->first)) {
            ++it;
            continue;
        }
        it = m_tiles.erase(it);
        m_root_version++;
    }

    const LodParams lod = Lod();
    for (auto& p : m_tiles) {
        p.second->updateLOD(camera, lod);
    }

}

int ChPlanetQuadtree::Invalidate(const ChGeoRegion& region) {
    return m_impl->Invalidate(region);
}

long long ChPlanetQuadtree::GetNumRebuiltTiles() const {
    return m_impl->m_rebuilt;
}

void ChPlanetQuadtree::SetSplitDistance(double tile_widths) {
    if (!(tile_widths > 0))
        throw std::invalid_argument("ChPlanetQuadtree::SetSplitDistance: must be positive");
    m_impl->m_split_tiles = tile_widths;
}

void ChPlanetQuadtree::SetMinZoom(int zoom) {
    if (zoom < 0)
        throw std::invalid_argument("ChPlanetQuadtree::SetMinZoom: must not be negative");
    m_impl->m_min_zoom = zoom;
}

int ChPlanetQuadtree::GetMinZoom() const {
    return m_impl->m_min_zoom;
}

void ChPlanetQuadtree::SetMaxZoom(int zoom) {
    if (zoom < 0)
        throw std::invalid_argument("ChPlanetQuadtree::SetMaxZoom: must not be negative");
    m_impl->m_max_zoom = zoom;
}

int ChPlanetQuadtree::GetMaxZoom() const {
    return m_impl->m_max_zoom;
}

double ChPlanetQuadtree::GetSplitDistance() const {
    return m_impl->m_split_tiles;
}

void ChPlanetQuadtree::SetHorizonMargin(double margin) {
    m_impl->m_horizon_margin = margin;
}

double ChPlanetQuadtree::GetHorizonMargin() const {
    return m_impl->m_horizon_margin;
}

long long ChPlanetQuadtree::GetTick() const {
    return m_impl->m_tick;
}

int ChPlanetQuadtree::GetNumRootTiles() const {
    return static_cast<int>(m_impl->m_tiles.size());
}

std::shared_ptr<const ChPlanetSurface> ChPlanetQuadtree::GetSurface() const {
    return m_impl->m_surface;
}

std::vector<const ChTileMesh*> ChPlanetQuadtree::GetMeshes() const {
    std::vector<const ChTileMesh*> out;
    for (const auto& p : m_impl->m_tiles) {
        for (const auto& kv : p.second->getMeshes()) {
            if (!kv.first->isDivided()) {
                out.push_back(&kv.second);  // divided nodes keep a hidden mesh
            }
        }
    }
    std::sort(out.begin(), out.end(), [](const ChTileMesh* a, const ChTileMesh* b) { return a->id < b->id; });
    return out;
}

std::optional<double> ChPlanetQuadtree::GetElevation(double lon_deg, double lat_deg, int zoom) const {
    const auto [x, y] = Traits::computeTileIndices({lon_deg, lat_deg}, m_impl->m_tile_size);
    const auto it = m_impl->m_tiles.find(TileKey{x, y});
    if (it == m_impl->m_tiles.end()) {
        return std::nullopt;
    }
    return it->second->getElevation({lon_deg, lat_deg}, zoom);
}

unsigned long long ChPlanetQuadtree::GetMeshSetVersion() const {
    unsigned long long v = m_impl->m_root_version * kRootVersionStride;
    for (const auto& p : m_impl->m_tiles) {
        v += p.second->meshSetVersion();
    }
    return v;
}

ChVector3d ChPlanetQuadtree::GetCameraPosition() const {
    const util::Vec3& c = m_impl->m_camera;
    return ChVector3d(c.x, c.y, c.z);
}

ChVector2d ChPlanetQuadtree::GetCameraLonLat() const {
    const util::Vec3& c = m_impl->m_camera;
    const util::LonLat ll = util::lonLatOf(c.x, c.y, c.z);
    return ChVector2d(ll.lon, ll.lat);
}

double ChPlanetQuadtree::GetCameraElevation() const {
    const util::Vec3& c = m_impl->m_camera;
    return Traits::elevationOf(c.x, c.y, c.z, m_impl->m_radius);
}

}  // namespace planet
}  // namespace chrono
