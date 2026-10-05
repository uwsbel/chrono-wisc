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
#include <limits>
#include <stdexcept>

#include "chrono_planet/samplers/ChElevationSampler.h"
#include "chrono_planet/core/Perlin.h"

namespace chrono {
namespace planet {

namespace {
constexpr double kNoData = std::numeric_limits<double>::quiet_NaN();
}

ChElevationSampler::~ChElevationSampler() = default;

void ChElevationSampler::GetHeightGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& heights) const {
    const int n = grid.n;
    heights.resize(static_cast<size_t>(n) * n);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i)
            heights[static_cast<size_t>(j) * n + i] =
                GetHeight(grid.lon0 + i * grid.step_lon, grid.lat0 + j * grid.step_lat, zoom).value_or(kNoData);
}

// -----------------------------------------------------------------------------

ChNoiseElevation::ChNoiseElevation(double amplitude, double frequency)
    : m_amplitude(amplitude), m_frequency(frequency) {}

std::optional<double> ChNoiseElevation::GetHeight(double lon_deg, double lat_deg, int) const {
    return m_amplitude * Perlin::noise(static_cast<float>(lat_deg * m_frequency),
                                       static_cast<float>(lon_deg * m_frequency));
}

// -----------------------------------------------------------------------------

ChElevationFunction::ChElevationFunction(Function f) : m_function(std::move(f)) {
    if (!m_function)
        throw std::invalid_argument("ChElevationFunction: empty function");
}

std::optional<double> ChElevationFunction::GetHeight(double lon_deg, double lat_deg, int) const {
    return m_function(lon_deg, lat_deg);
}

// -----------------------------------------------------------------------------

ChSamplerChain::ChSamplerChain(std::vector<std::shared_ptr<ChElevationSampler>> samplers) {
    for (auto& s : samplers)
        AddSampler(std::move(s));
}

void ChSamplerChain::AddSampler(std::shared_ptr<ChElevationSampler> sampler) {
    if (!sampler)
        throw std::invalid_argument("ChSamplerChain::AddSampler: null sampler");
    m_samplers.push_back(std::move(sampler));
}

std::optional<double> ChSamplerChain::GetHeight(double lon_deg, double lat_deg, int zoom) const {
    for (const auto& s : m_samplers)
        if (auto h = s->GetHeight(lon_deg, lat_deg, zoom))
            return h;
    return std::nullopt;
}

void ChSamplerChain::GetHeightGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& heights) const {
    heights.assign(static_cast<size_t>(grid.n) * grid.n, kNoData);
    std::vector<double> next;
    for (const auto& s : m_samplers) {
        if (std::none_of(heights.begin(), heights.end(), [](double h) { return std::isnan(h); }))
            break;
        // Fill what is still missing from this sampler; the rest stays missing.
        s->GetHeightGrid(grid, zoom, next);
        for (size_t v = 0; v < heights.size(); ++v)
            if (std::isnan(heights[v]))
                heights[v] = next[v];
    }
}

}  // namespace planet
}  // namespace chrono
