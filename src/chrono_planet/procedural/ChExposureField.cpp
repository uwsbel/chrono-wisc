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
#include <mutex>
#include <stdexcept>

#include "chrono_planet/core/FieldHash.h"
#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/core/SphereMath.h"
#include "chrono_planet/core/Perlin.h"
#include "chrono_planet/procedural/ChExposureField.h"

namespace chrono {
namespace planet {

ChExposureField::ChExposureField(const ChPlanetBody& body, std::shared_ptr<const ChElevationSampler> data, const Params& params)
    : m_data(std::move(data)), m_params(params), m_m_per_deg(field::kmPerDeg(body.GetRadius()) * 1000.0), m_radius(body.GetRadius()) {
    if (!m_data)
        throw std::invalid_argument("ChExposureField: needs elevation data");
    if (!(params.reach > 0) || !(params.rise_rock > params.rise_soil) || !(params.slope_rock > params.slope_soil))
        throw std::invalid_argument("ChExposureField: reach must be positive, and each rock threshold over its soil one");
}

double ChExposureField::GetExposure(double lon_deg, double lat_deg, double reach) const {
    if (reach > 0) {
        return At(lon_deg, lat_deg, reach);
    }
    // Square in degrees, a reach from north to south. Bilinear between its four nodes.
    const double cell = m_params.reach / m_m_per_deg;
    const double u = lon_deg / cell, v = lat_deg / cell;
    const int i = field::fastFloor(u), j = field::fastFloor(v);
    const double fu = u - i, fv = v - j;
    return (1 - fu) * (1 - fv) * Node(i, j) + fu * (1 - fv) * Node(i + 1, j) + (1 - fu) * fv * Node(i, j + 1) + fu * fv * Node(i + 1, j + 1);
}

double ChExposureField::GetExposureNear(double lon_deg, double lat_deg) const {
    const double cell = m_params.reach / m_m_per_deg;
    return Node(field::fastFloor(lon_deg / cell + 0.5), field::fastFloor(lat_deg / cell + 0.5));
}

double ChExposureField::Node(int a, int b) const {
    const double cell = m_params.reach / m_m_per_deg;
    const std::int64_t key = (static_cast<std::int64_t>(a) << 32) | static_cast<std::uint32_t>(b);
    {
        std::shared_lock<std::shared_mutex> lock(m_mutex);
        const auto found = m_nodes.find(key);
        if (found != m_nodes.end()) {
            return static_cast<double>(found->second);
        }
    }
    const double e = At(a * cell, b * cell, m_params.reach);
    std::unique_lock<std::shared_mutex> lock(m_mutex);
    m_nodes.emplace(key, static_cast<float>(e));
    return e;
}

double ChExposureField::At(double lon_deg, double lat_deg, double r) const {
    const auto here = m_data->GetHeight(lon_deg, lat_deg, m_params.zoom);
    if (!here) {
        return 0.0;
    }
    // Eight points a reach away: east, north-east, north and so on round.
    const double dLat = r / m_m_per_deg, dLon = dLat / util::cosLatClamped(lat_deg);
    constexpr double kDiagonal = 0.70710678118654752;
    const double east[8] = {1, kDiagonal, 0, -kDiagonal, -1, -kDiagonal, 0, kDiagonal};
    const double north[8] = {0, kDiagonal, 1, kDiagonal, 0, -kDiagonal, -1, -kDiagonal};
    double h[8], mean = 0.0;
    for (int k = 0; k < 8; ++k) {
        const auto there = m_data->GetHeight(lon_deg + east[k] * dLon, lat_deg + north[k] * dLat, m_params.zoom);
        h[k] = there ? *there : *here;
        mean += h[k] / 8.0;
    }
    const double rise = (*here - mean) / r;
    const double slope = std::hypot(h[0] - h[4], h[2] - h[6]) / (2.0 * r);
    const auto share = [](double x, double lo, double hi) { return util::smoothstep01(std::clamp((x - lo) / (hi - lo), 0.0, 1.0)); };
    double exposure = std::max(share(rise, m_params.rise_soil, m_params.rise_rock), share(slope, m_params.slope_soil, m_params.slope_rock));
    if (m_params.floor_cover > 0) {
        // The noise, stretched as the rock layer's patches are, is near even from -1 to 1: what of it is over the
        // threshold is the cover asked for. A patch's edge is a fifth of the range wide
        // Two sizes of it, so a patch's outline is not a round blob's
        const auto dir = util::dirFromLonLat(lon_deg, lat_deg);
        const double n = std::clamp(2.5 * (0.7 * Perlin::onSphere(dir, m_radius / m_params.floor_patch) +
                                           0.3 * Perlin::onSphere(dir, 4.0 * m_radius / m_params.floor_patch)),
                                    -1.0, 1.0);
        exposure = std::max(exposure, share(n, 1.0 - 2.0 * m_params.floor_cover - 0.2, 1.0 - 2.0 * m_params.floor_cover + 0.2));
    }
    return exposure;
}

}  // namespace planet
}  // namespace chrono
