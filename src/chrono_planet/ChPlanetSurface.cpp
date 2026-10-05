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

#include <cmath>
#include <limits>
#include <stdexcept>

#include "chrono/core/ChTypes.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/core/BenchProfiler.h"
#include "chrono_planet/core/Parallel.h"
#include "chrono_planet/core/SphereMath.h"

namespace chrono {
namespace planet {

ChPlanetSurface::ChPlanetSurface(const ChPlanetBody& body, int zoom)
    : m_body(body),
      m_filters(chrono_types::make_shared<ChFilterChain>()),
      m_zoom(zoom),
      m_spacing(0) {}

ChPlanetSurface::~ChPlanetSurface() = default;

void ChPlanetSurface::AddGeoTiff(const ChGeoTiffSource& source) {
    auto stack = GetGeoTiffStack();
    if (!stack) {
        if (m_sampler)
            throw std::logic_error("ChPlanetSurface::AddGeoTiff: the sampler is not a GeoTIFF stack");
        stack = chrono_types::make_shared<ChGeoTiffStack>(m_body);
        m_sampler = stack;
    }
    stack->AddSource(source);
}

std::shared_ptr<ChGeoTiffStack> ChPlanetSurface::GetGeoTiffStack() const {
    return std::dynamic_pointer_cast<ChGeoTiffStack>(m_sampler);
}

void ChPlanetSurface::SetFilterChain(std::shared_ptr<ChFilterChain> chain) {
    m_filters = chain ? std::move(chain) : chrono_types::make_shared<ChFilterChain>();
}

double ChPlanetSurface::GetSampleSpacing() const {
    return m_spacing > 0 ? m_spacing : GetSampleSpacingAtZoom(m_zoom);
}

double ChPlanetSurface::Fallback(double lon_deg, double lat_deg, int zoom) const {
    if (!m_fallback)
        return 0.0;
    return m_fallback->GetHeight(lon_deg, lat_deg, zoom).value_or(0.0);
}

std::optional<double> ChPlanetSurface::GetDataElevation(double lon_deg, double lat_deg, int zoom) const {
    if (!m_sampler)
        return std::nullopt;
    return m_sampler->GetHeight(util::wrapLongitude(lon_deg), lat_deg, zoom);
}

double ChPlanetSurface::GetElevationAtZoom(double lon_deg, double lat_deg, int zoom, double spacing_deg) const {
    lon_deg = util::wrapLongitude(lon_deg);
    if (!(spacing_deg > 0))
        spacing_deg = GetSampleSpacingAtZoom(zoom);
    const auto data = GetDataElevation(lon_deg, lat_deg, zoom);
    const double h = data && !std::isnan(*data) ? *data : Fallback(lon_deg, lat_deg, zoom);
    return m_filters->Apply(lon_deg, lat_deg, spacing_deg, h);
}

void ChPlanetSurface::SplitFilters(std::vector<const ChSurfaceFilter*>& statics,
                                   std::vector<const ChSurfaceFilter*>& dynamics) const {
    std::vector<const ChSurfaceFilter*> all;
    m_filters->Flatten(all);
    size_t split = 0;
    while (split < all.size() && !all[split]->IsDynamic())
        ++split;
    statics.assign(all.begin(), all.begin() + split);
    dynamics.assign(all.begin() + split, all.end());
}

void ChPlanetSurface::ApplyFilters(const std::vector<const ChSurfaceFilter*>& filters,
                                   const ChGeoGrid& grid,
                                   std::vector<double>& out) const {
    if (filters.empty())
        return;
    const int n = grid.n;
    const double startLon = util::wrapLongitude(grid.lon0);
    // Filters take a grid that does not cross the dateline; one that does is evaluated per sample.
    BENCH_SCOPE("grid_filters");
    if (util::wrapLongitude(startLon + (n - 1) * grid.step_lon) >= startLon) {
        const ChGeoGrid wrapped{startLon, grid.lat0, grid.step_lon, grid.step_lat, n, grid.spacing};
        for (const ChSurfaceFilter* f : filters)
            f->ApplyGrid(wrapped, out);
    } else {
        const bool par = util::parallelGrid(static_cast<size_t>(n) * n);
#pragma omp parallel for num_threads(util::parallelThreads()) schedule(dynamic, 64) if (par)
        for (int j = 0; j < n; ++j) {
            const double lat = grid.lat0 + j * grid.step_lat;
            for (int i = 0; i < n; ++i) {
                const double lon = util::wrapLongitude(startLon + i * grid.step_lon);
                double& h = out[static_cast<size_t>(j) * n + i];
                for (const ChSurfaceFilter* f : filters)
                    h = f->Apply(lon, lat, grid.spacing, h);
            }
        }
    }
}

void ChPlanetSurface::SampleElevationGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& out) const {
    const int n = grid.n;
    if (n <= 0)
        throw std::invalid_argument("ChPlanetSurface::GetElevationGrid: n must be positive");

    // Elevation data on the grid as given, the fallback where it has none.
    {
        BENCH_SCOPE("grid_dem");
        if (m_sampler)
            m_sampler->GetHeightGrid(grid, zoom, out);
        else
            out.assign(static_cast<size_t>(n) * n, std::numeric_limits<double>::quiet_NaN());
    }
    const double startLon = util::wrapLongitude(grid.lon0);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            double& h = out[static_cast<size_t>(j) * n + i];
            if (std::isnan(h))
                h = Fallback(util::wrapLongitude(startLon + i * grid.step_lon), grid.lat0 + j * grid.step_lat, zoom);
        }
}

void ChPlanetSurface::GetElevationGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& out) const {
    SampleElevationGrid(grid, zoom, out);
    ApplyFilters({m_filters.get()}, grid, out);
}

void ChPlanetSurface::GetStaticElevationGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& out) const {
    SampleElevationGrid(grid, zoom, out);
    std::vector<const ChSurfaceFilter*> statics, dynamics;
    SplitFilters(statics, dynamics);
    ApplyFilters(statics, grid, out);
}

void ChPlanetSurface::ApplyDynamicFilters(const ChGeoGrid& grid, std::vector<double>& heights) const {
    std::vector<const ChSurfaceFilter*> statics, dynamics;
    SplitFilters(statics, dynamics);
    ApplyFilters(dynamics, grid, heights);
}

double ChPlanetSurface::GetElevation(double lon_deg, double lat_deg) const {
    return GetElevationAtZoom(lon_deg, lat_deg, m_zoom, GetSampleSpacing());
}

void ChPlanetSurface::GetElevationGrid(double lon0,
                                       double lat0,
                                       double step_lon,
                                       double step_lat,
                                       int n,
                                       std::vector<double>& out) const {
    GetElevationGrid(ChGeoGrid{lon0, lat0, step_lon, step_lat, n, GetSampleSpacing()}, m_zoom, out);
}

std::shared_ptr<ChPlanetSurface> ChPlanetSurface::CreateView(std::shared_ptr<ChSurfaceFilter> extra) const {
    auto view = chrono_types::make_shared<ChPlanetSurface>(m_body, m_zoom);
    view->m_sampler = m_sampler;
    view->m_fallback = m_fallback;
    view->m_lod = m_lod;
    view->m_spacing = m_spacing;
    view->m_filters = chrono_types::make_shared<ChFilterChain>();
    view->m_filters->AddFilter(m_filters);
    if (extra)
        view->m_filters->AddFilter(std::move(extra));
    return view;
}

ChSiteFrame ChPlanetSurface::MakeSiteFrame(double lon_deg, double lat_deg) const {
    return ChSiteFrame(m_body, lon_deg, lat_deg, GetElevation(lon_deg, lat_deg));
}

}  // namespace planet
}  // namespace chrono
