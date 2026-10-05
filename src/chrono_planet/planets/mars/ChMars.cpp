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

#include "chrono_planet/planets/mars/ChMars.h"

namespace chrono {
namespace planet {
namespace mars {

ChPlanetBody Body() {
    return ChPlanetBody("Mars", kRadius, kGravity);
}

ChCraterLayer::Params CraterParams() {
    ChCraterLayer::Params p;
    p.diameters_km = {80.0, 40.0, 20.0, 10.0, 5.0, 2.5, 1.25, 0.6, 0.3, 0.15, 0.08, 0.04, 0.02, 0.01};
    p.densities = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.3, 0.3};
    // Fresh small craters: depth 0.228 D, the mean of the new, dated impacts HiRISE has measured (Daubar et al.,
    // JGR Planets 119, 2014). Rim 0.03 D, the most pristine of 10 m to 1.2 km craters (Sweeney et al., JGR Planets
    // 123, 2018), whose degraded classes, 0.081 D down to 0.046 D deep, the layer's ageing spans.
    p.simple_depth_ratio = 0.228;
    p.simple_rim_ratio = 0.03;
    // Complex craters, 7 to 100 km: depth 0.36 D^0.49 and rim 0.02 D^0.84, D in km, MOLA's (Garvin et al., Sixth
    // International Conference on Mars, 2003, abstract 3277).
    p.complex_diameter_km = 7.0;
    p.complex_depth_coef = 0.36;
    p.complex_depth_exp = 0.49;
    p.complex_rim_coef = 0.02;
    p.complex_rim_exp = 0.84;
    p.floor_flattening_km = 30.0;
    return p;
}

ChRockLayer::Params RockParams() {
    ChRockLayer::Params p;
    // k from 2.7% to 24% about 8%, in patches. Mars's rocks are patchy: about 1% on smooth plains and 35% beside
    // rocky craters at InSight's site (Golombek et al., JGR Planets 125, 2020), and up to 15 to 20% at Jezero from
    // orbit (quoted by Otsu et al., arXiv:1808.00031). The 8% and the patches' 40 m have no source: Mars 2020's
    // descent camera shows Jezero's rocks in clusters and bands some tens of metres across.
    p.coverage = 0.08;
    p.coverage_spread = 3.0;
    p.patch_size = 40.0;
    return p;
}

ChExposureField::Params ExposureParams() {
    // Jezero's floor where Mars 2020 landed is fractured bedrock under a thin, patchy soil: its descent camera shows
    // pale plates over much of the level ground from 100 m down. A fifth of it, in patches 60 m across: by eye from
    // that footage, no measured source.
    ChExposureField::Params p;
    p.floor_cover = 0.20;
    p.floor_patch = 60.0;
    return p;
}

std::vector<ChBedformLayer::Params> BedformParams() {
    // Transverse aeolian ridges, at the small end of their range: wavelengths 6 to 140 m, heights 0.3 to 6.4 m,
    // symmetric (Hugenholtz et al., Icarus 286, 2017, as quoted by Fenton et al., Front. Earth Sci. 8, 2021;
    // Zimbelman, Geomorphology 121, 2010).
    ChBedformLayer::Params ridges;
    ridges.wavelength = 10.0;
    ridges.height = 0.6;
    ridges.cover = 0.07;
    ridges.field_size = 250.0;
    // Large ripples: 1 to 5 m apart and 0.1 to 0.4 m tall (Silvestro et al., JGR Planets 125, 2020), height a tenth
    // of the wavelength, sinuous, with lee faces near 30 degrees (Lapotre et al., Science 353, 2016). 2.1 m is the
    // mean on the Bagnold dunes (Ewing et al., JGR Planets 122, 2017). The lee's share keeps its steepest slope near 30.
    ChBedformLayer::Params ripples;
    ripples.wavelength = 2.1;
    ripples.height = 0.2;
    ripples.lee_fraction = 0.36;
    ripples.sinuosity = 0.6;
    ripples.cover = 0.15;
    ripples.field_size = 80.0;
    ripples.seed = 1;
    // No source: how much ground either covers, the fields' widths, and the wind, taken to blow west.
    for (ChBedformLayer::Params* p : {&ridges, &ripples}) {
        p->wind_azimuth = 270.0;
        p->azimuth_spread = 25.0;
    }
    return {ridges, ripples};
}

ChRoughnessLayer::Params RoughnessParams() {
    // The Moon's, for want of Martian slopes under a meter. At a 1 m sampling it adds 2 degrees RMS; InSight's
    // smooth plains have 3.9 at 1 m in all (Golombek et al., JGR Planets 125, 2020).
    ChRoughnessLayer::Params p;
    p.coarsest_wavelength = 4.0;
    p.octaves = 8;
    p.slope_rms = 0.02;
    p.fine_boost = 1.4;
    p.clod_below = 0.4;
    p.clod_variance_share = 0.25;
    return p;
}

std::shared_ptr<ChFilterChain> FilterChain() {
    const ChPlanetBody body = Body();
    auto chain = chrono_types::make_shared<ChFilterChain>();
    chain->AddFilter(chrono_types::make_shared<ChCraterLayer>(body, CraterParams()));
    for (const ChBedformLayer::Params& bedforms : BedformParams())
        chain->AddFilter(chrono_types::make_shared<ChBedformLayer>(body, bedforms));
    chain->AddFilter(chrono_types::make_shared<ChRockLayer>(body, RockParams()));
    chain->AddFilter(chrono_types::make_shared<ChRoughnessLayer>(body, RoughnessParams()));
    return chain;
}

std::shared_ptr<ChPlanetSurface> CreateSurface(const std::vector<ChGeoTiffSource>& dems, int zoom) {
    auto surface = chrono_types::make_shared<ChPlanetSurface>(Body(), zoom);
    for (const auto& dem : dems)
        surface->AddGeoTiff(dem);
    surface->SetFilterChain(FilterChain());
    return surface;
}

}  // namespace mars
}  // namespace planet
}  // namespace chrono
