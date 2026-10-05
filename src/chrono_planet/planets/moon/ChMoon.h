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
// Moon preset, terrain: body constants, calibrated relief parameters and the
// preset filter chain. DEM resources are in ChMoonDem.h.
//
// =============================================================================

#ifndef CH_MOON_H
#define CH_MOON_H

#include <memory>
#include <string>
#include <vector>

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChPlanetBody.h"
#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/dem/ChGeoTiffStack.h"
#include "chrono_planet/filters/ChSurfaceFilter.h"
#include "chrono_planet/procedural/ChBaseReliefLayer.h"
#include "chrono_planet/procedural/ChCraterLayer.h"
#include "chrono_planet/procedural/ChRockLayer.h"
#include "chrono_planet/procedural/ChRoughnessLayer.h"

namespace chrono {
namespace planet {

/// Earth's Moon. Everything here is built from the generic Chrono::Planet API, so this preset is also
/// the template for adding another body: copy it, change the constants, and keep the same structure.
namespace moon {

/// @addtogroup planet_module
/// @{

constexpr double kRadius = 1737400.0;  ///< mean radius (m), the IAU reference sphere of LOLA products
constexpr double kGravity = 1.62;      ///< surface gravity (m/s^2)

/// The Moon as a body: radius, gravity, and a spherical geographic system of that radius.
CH_PLANET_API ChPlanetBody Body();

/// Low-frequency swell, about 700 m peak over roughly 545 km wavelengths.
CH_PLANET_API ChBaseReliefLayer::Params BaseReliefParams();

/// Crater population from 80 km down to 1.25 m, near the lunar equilibrium density at small sizes,
/// with the Pike (1977) lunar depth and rim laws and a 15 km simple-to-complex transition.
CH_PLANET_API ChCraterLayer::Params CraterParams();

/// Boulders from 6 cm to 4 m covering 2% of the ground (Golombek-Rapp k = 0.02).
CH_PLANET_API ChRockLayer::Params RockParams();

/// Regolith micro-roughness, 4 m down to 3 cm wavelengths.
CH_PLANET_API ChRoughnessLayer::Params RoughnessParams();

/// Meter-scale noise used where no DEM covers the ground.
CH_PLANET_API std::shared_ptr<ChElevationSampler> Fallback();

/// The preset filter chain: the swell, crater, rock and roughness layers, in that order.
/// Each call returns a new chain, which can be extended with filters of your own.
CH_PLANET_API std::shared_ptr<ChFilterChain> FilterChain();

/// A lunar surface with no DEM: the Moon body, the fallback noise and the preset filter chain.
CH_PLANET_API std::shared_ptr<ChPlanetSurface> CreateSurface(int zoom = 15);

/// A lunar surface on the given rasters (any GeoTIFF). Throws std::runtime_error if one cannot be loaded.
CH_PLANET_API std::shared_ptr<ChPlanetSurface> CreateSurface(const std::vector<ChGeoTiffSource>& dems, int zoom = 15);

/// @} planet_module

}  // namespace moon
}  // namespace planet
}  // namespace chrono

#endif
