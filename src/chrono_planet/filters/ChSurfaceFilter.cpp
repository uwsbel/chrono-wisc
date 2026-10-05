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
#include <atomic>
#include <cmath>
#include <stdexcept>

#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/core/Parallel.h"
#include "chrono_planet/core/SphereMath.h"
#include "chrono_planet/filters/ChSurfaceFilter.h"

namespace chrono {
namespace planet {

ChSurfaceFilter::~ChSurfaceFilter() = default;

std::uint64_t ChSurfaceFilter::NextVersion() {
    static std::atomic<std::uint64_t> counter{0};
    return ++counter;
}

void ChSurfaceFilter::ApplyGrid(const ChGeoGrid& grid, std::vector<double>& heights) const {
    const int n = grid.n;
    const bool par = util::parallelGrid(static_cast<size_t>(n) * n);
#pragma omp parallel for num_threads(util::parallelThreads()) schedule(static) if (par)
    for (int j = 0; j < n; ++j) {
        const double lat = grid.lat0 + j * grid.step_lat;
        double* row = &heights[static_cast<size_t>(j) * n];
        for (int i = 0; i < n; ++i)
            row[i] = Apply(grid.lon0 + i * grid.step_lon, lat, grid.spacing, row[i]);
    }
}

// -----------------------------------------------------------------------------

ChFilterChain::ChFilterChain(std::vector<std::shared_ptr<ChSurfaceFilter>> filters) {
    for (auto& f : filters)
        AddFilter(std::move(f));
}

void ChFilterChain::AddFilter(std::shared_ptr<ChSurfaceFilter> filter) {
    InsertFilter(m_filters.size(), std::move(filter));
}

void ChFilterChain::InsertFilter(size_t index, std::shared_ptr<ChSurfaceFilter> filter) {
    if (!filter)
        throw std::invalid_argument("ChFilterChain: null filter");
    if (filter.get() == this)
        throw std::invalid_argument("ChFilterChain: a chain cannot contain itself");
    m_filters.insert(m_filters.begin() + std::min(index, m_filters.size()), std::move(filter));
}

bool ChFilterChain::RemoveFilter(const std::shared_ptr<ChSurfaceFilter>& filter) {
    const auto it = std::find(m_filters.begin(), m_filters.end(), filter);
    if (it == m_filters.end())
        return false;
    m_filters.erase(it);
    return true;
}

double ChFilterChain::Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const {
    for (const auto& f : m_filters)
        height = f->Apply(lon_deg, lat_deg, spacing_deg, height);
    return height;
}

void ChFilterChain::ApplyGrid(const ChGeoGrid& grid, std::vector<double>& heights) const {
    for (const auto& f : m_filters)
        f->ApplyGrid(grid, heights);
}

std::uint64_t ChFilterChain::GetChanges(std::uint64_t since, ChGeoRegion& region) const {
    std::uint64_t latest = 0;
    for (const auto& f : m_filters)
        latest = std::max(latest, f->GetChanges(since, region));
    return latest;
}

bool ChFilterChain::IsDynamic() const {
    for (const auto& f : m_filters)
        if (f->IsDynamic())
            return true;
    return false;
}

void ChFilterChain::Flatten(std::vector<const ChSurfaceFilter*>& out) const {
    for (const auto& f : m_filters) {
        if (auto chain = dynamic_cast<const ChFilterChain*>(f.get()))
            chain->Flatten(out);
        else
            out.push_back(f.get());
    }
}

// -----------------------------------------------------------------------------

ChScaleFilter::ChScaleFilter(double scale, double offset) : m_scale(scale), m_offset(offset) {}

double ChScaleFilter::Apply(double, double, double, double height) const {
    return height * m_scale + m_offset;
}

ChClampFilter::ChClampFilter(double min_height, double max_height) : m_min(min_height), m_max(max_height) {
    if (!(min_height <= max_height))
        throw std::invalid_argument("ChClampFilter: min_height exceeds max_height");
}

double ChClampFilter::Apply(double, double, double, double height) const {
    return std::clamp(height, m_min, m_max);
}

ChFunctionFilter::ChFunctionFilter(Function f) : m_function(std::move(f)) {
    if (!m_function)
        throw std::invalid_argument("ChFunctionFilter: empty function");
}

double ChFunctionFilter::Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const {
    return m_function(lon_deg, lat_deg, spacing_deg, height);
}

// -----------------------------------------------------------------------------

ChRegionFilter::ChRegionFilter(std::shared_ptr<ChSurfaceFilter> filter,
                               double min_lon,
                               double min_lat,
                               double max_lon,
                               double max_lat,
                               double feather_deg)
    : m_filter(std::move(filter)), m_min_lat(min_lat), m_max_lat(max_lat), m_feather(feather_deg) {
    if (!m_filter)
        throw std::invalid_argument("ChRegionFilter: null filter");
    if (!(min_lat <= max_lat) || feather_deg < 0)
        throw std::invalid_argument("ChRegionFilter: need min_lat <= max_lat and a non-negative feather");
    // Longitude span, going east from min_lon, so a rectangle across the dateline works.
    double span = max_lon - min_lon;
    if (span < 0)
        span += 360.0;
    m_half_lon = 0.5 * span;
    m_center_lon = util::wrapLongitude(min_lon + m_half_lon);
}

double ChRegionFilter::GetWeight(double lon_deg, double lat_deg) const {
    const double dlon = std::max(0.0, std::abs(util::wrapLongitude(lon_deg - m_center_lon)) - m_half_lon);
    const double dlat = std::max({0.0, m_min_lat - lat_deg, lat_deg - m_max_lat});
    const double d = std::max(dlon, dlat);  // degrees outside the rectangle
    if (d <= 0)
        return 1.0;
    if (d >= m_feather)
        return 0.0;
    return util::smoothstep01(1.0 - d / m_feather);
}

double ChRegionFilter::Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const {
    const double w = GetWeight(lon_deg, lat_deg);
    if (w <= 0)
        return height;
    const double filtered = m_filter->Apply(lon_deg, lat_deg, spacing_deg, height);
    return w >= 1 ? filtered : height + w * (filtered - height);
}

std::uint64_t ChRegionFilter::GetChanges(std::uint64_t since, ChGeoRegion& region) const {
    // A change of the inner filter only shows inside the rectangle and its feather.
    ChGeoRegion inner;
    const std::uint64_t latest = m_filter->GetChanges(since, inner);
    if (!inner.IsEmpty()) {
        ChGeoRegion clipped = inner;
        const double lon0 = m_center_lon - m_half_lon - m_feather, lon1 = m_center_lon + m_half_lon + m_feather;
        clipped.min_lon = std::max(inner.min_lon, lon0);
        clipped.max_lon = std::min(inner.max_lon, lon1);
        clipped.min_lat = std::max(inner.min_lat, m_min_lat - m_feather);
        clipped.max_lat = std::min(inner.max_lat, m_max_lat + m_feather);
        region.Include(clipped);
    }
    return latest;
}

void ChRegionFilter::ApplyGrid(const ChGeoGrid& grid, std::vector<double>& heights) const {
    const int n = grid.n;
    const size_t count = static_cast<size_t>(n) * n;
    std::vector<double> weights(count);
    bool any = false, all = true;
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const double w = GetWeight(grid.lon0 + i * grid.step_lon, grid.lat0 + j * grid.step_lat);
            weights[static_cast<size_t>(j) * n + i] = w;
            any = any || w > 0;
            all = all && w >= 1;
        }
    if (!any)
        return;
    if (all) {
        m_filter->ApplyGrid(grid, heights);
        return;
    }
    std::vector<double> filtered = heights;
    m_filter->ApplyGrid(grid, filtered);
    for (size_t v = 0; v < count; ++v) {
        const double w = weights[v];
        if (w >= 1)
            heights[v] = filtered[v];
        else if (w > 0)
            heights[v] += w * (filtered[v] - heights[v]);
    }
}

}  // namespace planet
}  // namespace chrono
