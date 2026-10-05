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

#include "chrono/core/ChTypes.h"

#include "chrono_planet/planets/moon/ChMoon.h"

namespace chrono {
namespace planet {
namespace moon {

ChPlanetBody Body() {
    return ChPlanetBody("Moon", kRadius, kGravity);
}

ChBaseReliefLayer::Params BaseReliefParams() {
    ChBaseReliefLayer::Params p;
    p.frequency = 20;
    p.amplitude = 700;
    return p;
}

ChCraterLayer::Params CraterParams() {
    ChCraterLayer::Params p;
    p.diameters_km = {80.0, 40.0, 20.0, 10.0, 5.0,   2.5,    1.25,    0.6,    0.3,
                      0.15, 0.08, 0.04, 0.02, 0.01, 0.005, 0.0025, 0.00125};
    p.densities = {0.275, 0.25, 0.24, 0.24, 0.25, 0.25, 0.25, 0.25, 0.25,
                   0.25,  0.25, 0.25, 0.45, 0.45, 0.6,  0.7,  0.7};
    // Pike (1977) fresh lunar crater morphometry, D and depths in km.
    p.complex_diameter_km = 15.0;
    p.simple_depth_ratio = 0.196;
    p.simple_rim_ratio = 0.036;
    p.complex_depth_coef = 1.044;
    p.complex_depth_exp = 0.301;
    p.complex_rim_coef = 0.236;
    p.complex_rim_exp = 0.399;
    p.floor_flattening_km = 30.0;
    return p;
}

ChRockLayer::Params RockParams() {
    ChRockLayer::Params p;
    p.coverage = 0.02;
    p.qa = 1.79;
    p.qb = 0.152;
    p.bin_edges_m = {0.0625, 0.125, 0.25, 0.5, 1.0, 2.0, 4.0};
    return p;
}

ChRoughnessLayer::Params RoughnessParams() {
    ChRoughnessLayer::Params p;
    p.coarsest_wavelength = 4.0;
    p.octaves = 8;
    p.slope_rms = 0.02;
    p.fine_boost = 1.4;
    p.clod_below = 0.4;
    p.clod_variance_share = 0.25;
    return p;
}

std::shared_ptr<ChElevationSampler> Fallback() {
    return chrono_types::make_shared<ChNoiseElevation>(1.0, 0.1);
}

std::shared_ptr<ChFilterChain> FilterChain() {
    const ChPlanetBody body = Body();
    auto chain = chrono_types::make_shared<ChFilterChain>();
    chain->AddFilter(chrono_types::make_shared<ChBaseReliefLayer>(BaseReliefParams()));
    chain->AddFilter(chrono_types::make_shared<ChCraterLayer>(body, CraterParams()));
    chain->AddFilter(chrono_types::make_shared<ChRockLayer>(body, RockParams()));
    chain->AddFilter(chrono_types::make_shared<ChRoughnessLayer>(body, RoughnessParams()));
    return chain;
}

std::shared_ptr<ChPlanetSurface> CreateSurface(int zoom) {
    auto surface = chrono_types::make_shared<ChPlanetSurface>(Body(), zoom);
    surface->SetFallback(Fallback());
    surface->SetFilterChain(FilterChain());
    return surface;
}

std::shared_ptr<ChPlanetSurface> CreateSurface(const std::vector<ChGeoTiffSource>& dems, int zoom) {
    auto surface = CreateSurface(zoom);
    for (const auto& dem : dems)
        surface->AddGeoTiff(dem);
    return surface;
}

}  // namespace moon
}  // namespace planet
}  // namespace chrono
