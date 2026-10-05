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
#include <climits>
#include <limits>
#include <cmath>
#include <mutex>
#include <stdexcept>

#include "chrono/utils/ChConstants.h"

#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/filters/ChDeformationFilter.h"

namespace chrono {
namespace planet {

namespace {
// Changes kept for GetChanges; older ones collapse into "everything".
constexpr size_t kMaxLog = 1024;
// Relief spacing, in node spacings, where changes are fully visible and where they are gone.
constexpr double kFullBelow = 2.0, kGoneAbove = 4.0;
}  // namespace

ChDeformationFilter::ChDeformationFilter(const ChSiteFrame& site, double spacing)
    : m_site(site),
      m_spacing(spacing),
      m_meters_per_degree(site.GetRadius() * CH_PI / 180.0),
      m_i0(INT_MAX),
      m_j0(INT_MAX),
      m_i1(INT_MIN),
      m_j1(INT_MIN) {
    if (!(spacing > 0))
        throw std::invalid_argument("ChDeformationFilter: spacing must be positive");
}

double ChDeformationFilter::Weight(double spacing_deg) const {
    const double s = spacing_deg * m_meters_per_degree / m_spacing;  // relief spacing in node spacings
    if (s <= kFullBelow)
        return 1.0;
    if (s >= kGoneAbove)
        return 0.0;
    return util::smoothstep01((kGoneAbove - s) / (kGoneAbove - kFullBelow));
}

double ChDeformationFilter::DeltaAt(double x, double y) const {
    const double fx = x / m_spacing, fy = y / m_spacing;
    const int i = static_cast<int>(std::floor(fx)), j = static_cast<int>(std::floor(fy));
    if (i + 1 < m_i0 || i > m_i1 || j + 1 < m_j0 || j > m_j1)
        return 0.0;
    const double tx = fx - i, ty = fy - j;
    auto at = [&](int a, int b) {
        const auto it = m_deltas.find(Key(a, b));
        return it == m_deltas.end() ? 0.0 : it->second.delta;
    };
    const double south = at(i, j) * (1 - tx) + at(i + 1, j) * tx;
    const double north = at(i, j + 1) * (1 - tx) + at(i + 1, j + 1) * tx;
    return south * (1 - ty) + north * ty;
}

double ChDeformationFilter::HeightAt(double x, double y, double w, double height) const {
    const double delta = w * DeltaAt(x, y);
    const double moved = height + delta;
    if (!(m_flatten_depth > 0) || delta >= 0)
        return moved;
    const double c = w * m_flatten_strength * util::smoothstep01(std::min(1.0, -delta / m_flatten_depth));
    if (c <= 0)
        return moved;
    // The nodes' own heights, bilinear; a node without one stands in with the moved incoming height, so the
    // blend is continuous where compacted ground meets ground the soil model has not touched.
    const double fx = x / m_spacing, fy = y / m_spacing;
    const int i = static_cast<int>(std::floor(fx)), j = static_cast<int>(std::floor(fy));
    const double tx = fx - i, ty = fy - j;
    auto at = [&](int a, int b) {
        const auto it = m_deltas.find(Key(a, b));
        return it == m_deltas.end() || std::isnan(it->second.height) ? moved : it->second.height;
    };
    const double south = at(i, j) * (1 - tx) + at(i + 1, j) * tx;
    const double north = at(i, j + 1) * (1 - tx) + at(i + 1, j + 1) * tx;
    return moved + c * (south * (1 - ty) + north * ty - moved);
}

double ChDeformationFilter::Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const {
    const double w = Weight(spacing_deg);
    if (w <= 0)
        return height;
    const ChVector3d p = m_site.ToLocal(lon_deg, lat_deg, 0.0);
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    if (m_deltas.empty())
        return height;
    return HeightAt(p.x(), p.y(), w, height);
}

double ChDeformationFilter::GetDelta(double lon_deg, double lat_deg, double spacing_deg) const {
    const double w = Weight(spacing_deg);
    if (w <= 0)
        return 0.0;
    const ChVector3d p = m_site.ToLocal(lon_deg, lat_deg, 0.0);
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    return m_deltas.empty() ? 0.0 : w * DeltaAt(p.x(), p.y());
}

void ChDeformationFilter::ApplyGrid(const ChGeoGrid& grid, std::vector<double>& heights) const {
    const double w = Weight(grid.spacing);
    if (w <= 0)
        return;
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    if (m_deltas.empty())
        return;
    // Most tiles lie far from any change: reject them on their site-frame bounding box.
    const int n = grid.n;
    double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
    for (int c = 0; c < 4; ++c) {
        const double lon = grid.lon0 + ((c & 1) ? (n - 1) * grid.step_lon : 0.0);
        const double lat = grid.lat0 + ((c & 2) ? (n - 1) * grid.step_lat : 0.0);
        const ChVector3d p = m_site.ToLocal(lon, lat, 0.0);
        x0 = std::min(x0, p.x()), x1 = std::max(x1, p.x());
        y0 = std::min(y0, p.y()), y1 = std::max(y1, p.y());
    }
    if (x1 < (m_i0 - 1) * m_spacing || x0 > (m_i1 + 1) * m_spacing || y1 < (m_j0 - 1) * m_spacing ||
        y0 > (m_j1 + 1) * m_spacing)
        return;
    for (int j = 0; j < n; ++j) {
        const double lat = grid.lat0 + j * grid.step_lat;
        double* row = &heights[static_cast<size_t>(j) * n];
        for (int i = 0; i < n; ++i) {
            const ChVector3d p = m_site.ToLocal(grid.lon0 + i * grid.step_lon, lat, 0.0);
            row[i] = HeightAt(p.x(), p.y(), w, row[i]);
        }
    }
}

ChGeoRegion ChDeformationFilter::RegionOf(int i0, int j0, int i1, int j1) const {
    ChGeoRegion region;
    for (int c = 0; c < 4; ++c) {
        const double x = ((c & 1) ? i1 + 1 : i0 - 1) * m_spacing;
        const double y = ((c & 2) ? j1 + 1 : j0 - 1) * m_spacing;
        double lon, lat;
        m_site.ToLonLat(x, y, lon, lat);
        region.Include(lon, lat);
    }
    region.max_spacing = kGoneAbove * m_spacing / m_meters_per_degree;
    return region;
}

size_t ChDeformationFilter::SetDeltas(const std::vector<std::pair<ChVector2i, double>>& deltas, double tolerance) {
    std::vector<Node> nodes;
    nodes.reserve(deltas.size());
    for (const auto& [node, dz] : deltas)
        nodes.push_back({node, dz, std::numeric_limits<double>::quiet_NaN()});
    return SetNodes(nodes, tolerance);
}

size_t ChDeformationFilter::SetNodes(const std::vector<Node>& nodes, double tolerance) {
    std::unique_lock<std::shared_mutex> lock(m_mutex);
    size_t changed = 0;
    int i0 = INT_MAX, j0 = INT_MAX, i1 = INT_MIN, j1 = INT_MIN;
    for (const auto& [node, dz, height] : nodes) {
        const std::int64_t k = Key(node.x(), node.y());
        const auto it = m_deltas.find(k);
        const double old = it == m_deltas.end() ? 0.0 : it->second.delta;
        if (std::abs(dz - old) <= tolerance)
            continue;
        m_deltas[k] = {dz, height};
        ++changed;
        i0 = std::min(i0, node.x()), i1 = std::max(i1, node.x());
        j0 = std::min(j0, node.y()), j1 = std::max(j1, node.y());
    }
    if (changed == 0)
        return 0;
    m_i0 = std::min(m_i0, i0), m_i1 = std::max(m_i1, i1);
    m_j0 = std::min(m_j0, j0), m_j1 = std::max(m_j1, j1);
    m_log.push_back({NextVersion(), RegionOf(i0, j0, i1, j1)});
    if (m_log.size() > kMaxLog) {
        m_dropped = m_log.front().version;
        m_log.pop_front();
    }
    return changed;
}

void ChDeformationFilter::Clear() {
    std::unique_lock<std::shared_mutex> lock(m_mutex);
    if (m_deltas.empty())
        return;
    m_log.push_back({NextVersion(), RegionOf(m_i0, m_j0, m_i1, m_j1)});
    if (m_log.size() > kMaxLog) {
        m_dropped = m_log.front().version;
        m_log.pop_front();
    }
    m_deltas.clear();
}

double ChDeformationFilter::GetDelta(const ChVector2i& node) const {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    const auto it = m_deltas.find(Key(node.x(), node.y()));
    return it == m_deltas.end() ? 0.0 : it->second.delta;
}

size_t ChDeformationFilter::GetNumNodes() const {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    return m_deltas.size();
}

std::uint64_t ChDeformationFilter::GetChanges(std::uint64_t since, ChGeoRegion& region) const {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    if (m_log.empty())
        return 0;
    if (since < m_dropped && m_i0 <= m_i1) {
        // Some of what the caller missed is no longer logged: report everything ever touched.
        region.Include(RegionOf(m_i0, m_j0, m_i1, m_j1));
        return m_log.back().version;
    }
    for (auto it = m_log.rbegin(); it != m_log.rend() && it->version > since; ++it)
        region.Include(it->region);
    return m_log.back().version;
}

}  // namespace planet
}  // namespace chrono
