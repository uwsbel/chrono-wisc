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

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/core/Parallel.h"
#include "chrono_planet/core/SphereMath.h"
#include "chrono_planet/procedural/ChIsotropicRelief.h"

namespace chrono {
namespace planet {

namespace {

// The polar frame: the body's turned a quarter turn about its x axis. A direction (x, y, z) of the body is
// (x, z, -y) in it, so the north pole is at 90 degrees east on its equator and the south pole at 90 degrees west.
// Its own poles are on the body's equator at 90 degrees east and west, and its dateline on the body's equator too:
// none is in the polar caps, the only places it is used.
util::Vec3 toFrame(const util::Vec3& v) {
    return {v.x, v.z, -v.y};
}
util::Vec3 fromFrame(const util::Vec3& v) {
    return {v.x, -v.z, v.y};
}

// A rock's own number in [0, 1), from its id (splitmix64): which of the two frames' rocks stand in the blend band.
double unitOf(std::uint64_t id) {
    id += 0x9e3779b97f4a7c15ull;
    id = (id ^ (id >> 30)) * 0xbf58476d1ce4e5b9ull;
    id = (id ^ (id >> 27)) * 0x94d049bb133111ebull;
    id ^= id >> 31;
    return static_cast<double>(id >> 11) * (1.0 / 9007199254740992.0);
}

// Marks a rock of the polar frame, whose cells' numbers are those of other rocks in the body's own.
constexpr std::uint64_t kPolarRock = 1ull << 59;

// Is a longitude in [min, max), at any wrap.
bool inLongitudes(double lon, double minLon, double maxLon) {
    if (maxLon - minLon >= 360.0) {
        return true;
    }
    double offset = std::fmod(lon - minLon, 360.0);
    if (offset < 0.0) {
        offset += 360.0;
    }
    return offset < maxLon - minLon;
}

}  // namespace

namespace {

// The rock layer of a relief: itself, or the first in its chain. Null with none.
std::shared_ptr<ChRockLayer> rocksOf(const std::shared_ptr<ChSurfaceFilter>& relief) {
    if (auto chain = std::dynamic_pointer_cast<ChFilterChain>(relief)) {
        return chain->Find<ChRockLayer>();
    }
    return std::dynamic_pointer_cast<ChRockLayer>(relief);
}

// Elevation data asked for by the polar frame's longitude and latitude.
class PolarFrameData : public ChElevationSampler {
  public:
    explicit PolarFrameData(std::shared_ptr<const ChElevationSampler> data) : m_data(std::move(data)) {}
    virtual std::optional<double> GetHeight(double lon_deg, double lat_deg, int zoom) const override {
        double lon, lat;
        ChIsotropicRelief::FromPolarFrame(lon_deg, lat_deg, lon, lat);
        return m_data->GetHeight(lon, lat, zoom);
    }

  private:
    std::shared_ptr<const ChElevationSampler> m_data;
};

}  // namespace

ChIsotropicRelief::ChIsotropicRelief(std::shared_ptr<ChSurfaceFilter> relief, double blend_from_deg, double blend_to_deg)
    : ChIsotropicRelief(relief, relief, blend_from_deg, blend_to_deg) {}

ChIsotropicRelief::ChIsotropicRelief(std::shared_ptr<ChSurfaceFilter> relief,
                                     std::shared_ptr<ChSurfaceFilter> polar_relief,
                                     double blend_from_deg,
                                     double blend_to_deg)
    : m_relief(std::move(relief)), m_polar(std::move(polar_relief)), m_blend_from(blend_from_deg), m_blend_to(blend_to_deg) {
    if (!m_relief || !m_polar) {
        throw std::invalid_argument("ChIsotropicRelief: no relief to wrap");
    }
    if (!(blend_from_deg > 0.0 && blend_from_deg < blend_to_deg && blend_to_deg < 90.0)) {
        throw std::invalid_argument("ChIsotropicRelief: wants 0 < blend_from_deg < blend_to_deg < 90");
    }
    m_rocks = rocksOf(m_relief);
    m_polar_rocks = rocksOf(m_polar);
}

std::shared_ptr<ChElevationSampler> ChIsotropicRelief::PolarFrameSampler(std::shared_ptr<const ChElevationSampler> data) {
    if (!data) {
        throw std::invalid_argument("ChIsotropicRelief: no elevation data to read in the polar frame");
    }
    return chrono_types::make_shared<PolarFrameData>(std::move(data));
}

double ChIsotropicRelief::GetPolarWeight(double lat_deg) const {
    const double t = (std::abs(lat_deg) - m_blend_from) / (m_blend_to - m_blend_from);
    return util::smoothstep01(std::clamp(t, 0.0, 1.0));
}

void ChIsotropicRelief::ToPolarFrame(double lon_deg, double lat_deg, double& frame_lon_deg, double& frame_lat_deg) {
    const util::Vec3 in_frame = toFrame(util::dirFromLonLat(lon_deg, lat_deg));
    const util::LonLat at = util::lonLatOf(in_frame.x, in_frame.y, in_frame.z);
    frame_lon_deg = at.lon;
    frame_lat_deg = at.lat;
}

void ChIsotropicRelief::FromPolarFrame(double frame_lon_deg, double frame_lat_deg, double& lon_deg, double& lat_deg) {
    const util::Vec3 on_body = fromFrame(util::dirFromLonLat(frame_lon_deg, frame_lat_deg));
    const util::LonLat at = util::lonLatOf(on_body.x, on_body.y, on_body.z);
    lon_deg = at.lon;
    lat_deg = at.lat;
}

double ChIsotropicRelief::GetFrameTurn(double lon_deg, double lat_deg) {
    double frame_lon, frame_lat;
    ToPolarFrame(lon_deg, lat_deg, frame_lon, frame_lat);
    // The frame's east there, as a direction of the body, against the body's own east and north. At a pole of the
    // body its east is any direction: enuAt gives one, and the angle is from that.
    const util::Vec3 frame_east = fromFrame(util::enuAt(frame_lon, frame_lat).east);
    const util::EnuFrame body = util::enuAt(lon_deg, lat_deg);
    return std::atan2(util::dot(frame_east, body.north), util::dot(frame_east, body.east));
}

double ChIsotropicRelief::Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const {
    const double weight = GetPolarWeight(lat_deg);
    if (weight <= 0.0) {
        return m_relief->Apply(lon_deg, lat_deg, spacing_deg, height);
    }
    // A spacing is an angle on the body, the same distance on the ground wherever it is: it is the same in both frames
    double frame_lon, frame_lat;
    ToPolarFrame(lon_deg, lat_deg, frame_lon, frame_lat);
    const double polar = m_polar->Apply(frame_lon, frame_lat, spacing_deg, 0.0);
    if (weight >= 1.0) {
        return height + polar;
    }
    return height + weight * polar + (1.0 - weight) * m_relief->Apply(lon_deg, lat_deg, spacing_deg, 0.0);
}

void ChIsotropicRelief::ApplyGrid(const ChGeoGrid& grid, std::vector<double>& heights) const {
    const int n = grid.n;
    const double lat1 = grid.lat0 + (n - 1) * grid.step_lat;
    // All of it in the body's own frame: the wrapped relief's grid pass, as without this filter
    if (std::max(std::abs(grid.lat0), std::abs(lat1)) <= m_blend_from) {
        m_relief->ApplyGrid(grid, heights);
        return;
    }
    const bool parallel = util::parallelGrid(static_cast<size_t>(n) * n);
#pragma omp parallel for num_threads(util::parallelThreads()) schedule(dynamic, 4) if (parallel)
    for (int j = 0; j < n; ++j) {
        const double lat = grid.lat0 + j * grid.step_lat;
        double* row = &heights[static_cast<size_t>(j) * n];
        for (int i = 0; i < n; ++i) {
            row[i] = Apply(util::wrapLongitude(grid.lon0 + i * grid.step_lon), lat, grid.spacing, row[i]);
        }
    }
}

std::vector<ChRockInstance> ChIsotropicRelief::QueryRocks(double min_lon, double min_lat, double max_lon, double max_lat, double min_radius) const {
    std::vector<ChRockInstance> rocks;
    if (!(max_lon > min_lon) || !(max_lat > min_lat)) {
        return rocks;
    }
    const double least = std::min(std::abs(min_lat), std::abs(max_lat)), most = std::max(std::abs(min_lat), std::abs(max_lat));
    const bool spans_equator = min_lat < 0.0 && max_lat > 0.0;
    // Those of the body's own frame, where any of the rectangle is short of the polar caps
    if (m_rocks && (spans_equator || least < m_blend_to)) {
        for (const ChRockInstance& rock : m_rocks->Query(min_lon, min_lat, max_lon, max_lat, min_radius)) {
            if (unitOf(rock.id) >= GetPolarWeight(rock.latDeg)) {
                rocks.push_back(rock);
            }
        }
    }
    if (!m_polar_rocks || most <= m_blend_from) {
        return rocks;
    }
    // Those of the polar frame: of the rectangle that holds this one there, from its edges and its middle, and a
    // little over for what lies between the points taken. Then each put back on the body, and kept if it is in
    constexpr int kAlong = 8;
    double frame_min_lon = 1e300, frame_max_lon = -1e300, frame_min_lat = 1e300, frame_max_lat = -1e300;
    for (int j = 0; j <= kAlong; ++j) {
        for (int i = 0; i <= kAlong; ++i) {
            const double lat = std::clamp(min_lat + (max_lat - min_lat) * j / kAlong, -90.0, 90.0);
            if (std::abs(lat) < m_blend_from) {
                continue;
            }
            double frame_lon, frame_lat;
            ToPolarFrame(min_lon + (max_lon - min_lon) * i / kAlong, lat, frame_lon, frame_lat);
            frame_min_lon = std::min(frame_min_lon, frame_lon);
            frame_max_lon = std::max(frame_max_lon, frame_lon);
            frame_min_lat = std::min(frame_min_lat, frame_lat);
            frame_max_lat = std::max(frame_max_lat, frame_lat);
        }
    }
    if (frame_min_lon > frame_max_lon) {
        return rocks;
    }
    const double over_lon = 0.05 * (frame_max_lon - frame_min_lon) + 1e-9, over_lat = 0.05 * (frame_max_lat - frame_min_lat) + 1e-9;
    for (ChRockInstance rock : m_polar_rocks->Query(frame_min_lon - over_lon, frame_min_lat - over_lat, frame_max_lon + over_lon,
                                             frame_max_lat + over_lat, min_radius)) {
        double lon, lat;
        FromPolarFrame(rock.lonDeg, rock.latDeg, lon, lat);
        if (lat < min_lat || lat >= max_lat || !inLongitudes(lon, min_lon, max_lon)) {
            continue;
        }
        if (unitOf(rock.id) >= GetPolarWeight(lat)) {
            continue;
        }
        const float turn = static_cast<float>(GetFrameTurn(lon, lat));
        rock.lonDeg = lon;
        rock.latDeg = lat;
        rock.yawRad += turn;
        rock.tiltAzRad += turn;
        rock.id ^= kPolarRock;
        rocks.push_back(rock);
    }
    return rocks;
}

std::shared_ptr<ChRockLayer> ChIsotropicRelief::FindRocks(const ChPlanetSurface& surface) {
    if (const auto wrapped = surface.FindFilter<ChIsotropicRelief>()) {
        return wrapped->m_rocks;
    }
    return surface.FindFilter<ChRockLayer>();
}

std::vector<ChRockInstance> ChIsotropicRelief::QueryRocks(const ChPlanetSurface& surface,
                                                          double min_lon,
                                                          double min_lat,
                                                          double max_lon,
                                                          double max_lat,
                                                          double min_radius) {
    if (const auto wrapped = surface.FindFilter<ChIsotropicRelief>()) {
        return wrapped->QueryRocks(min_lon, min_lat, max_lon, max_lat, min_radius);
    }
    if (const auto rocks = surface.FindFilter<ChRockLayer>()) {
        return rocks->Query(min_lon, min_lat, max_lon, max_lat, min_radius);
    }
    return {};
}

}  // namespace planet
}  // namespace chrono
