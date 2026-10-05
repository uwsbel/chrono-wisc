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
//
// Mars preset: body constants and a starting set of relief parameters.
//
// =============================================================================

#ifndef CH_MARS_H
#define CH_MARS_H

#include <memory>
#include <vector>

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChPlanetBody.h"
#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/dem/ChGeoTiffStack.h"
#include "chrono_planet/filters/ChSurfaceFilter.h"
#include "chrono_planet/procedural/ChExposureField.h"
#include "chrono_planet/procedural/ChBedformLayer.h"
#include "chrono_planet/procedural/ChCraterLayer.h"
#include "chrono_planet/procedural/ChRockLayer.h"
#include "chrono_planet/procedural/ChRoughnessLayer.h"

namespace chrono {
namespace planet {

/// Mars, as a sphere of its mean radius.
/// The relief parameters are each from the literature where it gives one, cited beside it in the source,
/// and say so where it does not. They are not fitted to any one site.
namespace mars {

/// @addtogroup planet_module
/// @{

constexpr double kRadius = 3389500.0;  ///< mean radius (m)
constexpr double kGravity = 3.721;     ///< surface gravity (m/s^2)

/// Mars as a body: radius, gravity, and a spherical geographic system of that radius.
/// DEMs referenced to the areoid (MOLA MEGDR, HiRISE DTMs) are heights relative to a surface within
/// a few kilometers of this sphere; set ChGeoTiffSource::offset if a site needs a common datum.
CH_PLANET_API ChPlanetBody Body();

/// Craters: Martian depths and rims, simple under 7 km. The densities are the Moon's thinned, with no source.
CH_PLANET_API ChCraterLayer::Params CraterParams();

/// Boulders: Golombek-Rapp, k patchy from 1.25% to 20% about 5%.
CH_PLANET_API ChRockLayer::Params RockParams();

/// Wind-blown bedforms, one entry a scale, the larger first.
CH_PLANET_API ChExposureField::Params ExposureParams();
CH_PLANET_API std::vector<ChBedformLayer::Params> BedformParams();

/// Roughness under a meter.
CH_PLANET_API ChRoughnessLayer::Params RoughnessParams();

/// The preset filter chain: the crater, bedform, rock and roughness layers. Each call returns a new chain.
CH_PLANET_API std::shared_ptr<ChFilterChain> FilterChain();

/// A Martian surface: the Mars body, the given DEMs, and the preset filter chain.
/// Throws std::runtime_error if a DEM cannot be loaded.
CH_PLANET_API std::shared_ptr<ChPlanetSurface> CreateSurface(const std::vector<ChGeoTiffSource>& dems = std::vector<ChGeoTiffSource>(),
                                                             int zoom = 15);

/// @} planet_module

}  // namespace mars
}  // namespace planet
}  // namespace chrono

#endif
