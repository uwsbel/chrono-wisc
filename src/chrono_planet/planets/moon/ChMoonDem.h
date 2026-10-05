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
// Moon preset, data: the LOLA elevation models as selectable resources.
//
// =============================================================================

#ifndef CH_MOON_DEM_H
#define CH_MOON_DEM_H

#include <memory>
#include <string>
#include <vector>

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/dem/ChGeoTiffStack.h"
#include "chrono_planet/planets/moon/ChMoon.h"

namespace chrono {
namespace planet {
namespace moon {

/// @addtogroup planet_module
/// @{

/// Lunar elevation models available as resources (see data/planet/moon/README.md).
/// Pick the ones a simulation needs and pass them to CreateSurface or AddDem; each serves its own
/// zoom range, so a global DEM and a site DEM combine into one stack.
enum class Dem {
    GLOBAL_LOW_RES,         ///< LOLA global, 4 px/deg (~7.6 km), zooms 0-2. Shipped with Chrono.
    GLOBAL,                 ///< LOLA global, 64 px/deg (~474 m), zooms 0-4. Not shipped (480 MB).
    APOLLO17_LANDING_SITE,  ///< LOLA Apollo 17 region (Taurus-Littrow), 1024 px/deg (~30 m), zooms 5-30. Shipped.
    /// SLDEM2015 (LOLA and SELENE Terrain Camera), 512 px/deg (~59 m), 10 x 10 degrees around the Apollo 17
    /// site, zooms 5-7. Not shipped (100 MB, cut with make_sldem2015_crop.py). Unlike the LOLA site grid, free
    /// of the stripes of LOLA's ground tracks.
    APOLLO17_SLDEM2015,
    /// LROC NAC stereo DTM of Taurus-Littrow, 5 m/px, about 58 x 51 km around the Apollo 17 site, zooms 8-30.
    /// Not shipped (463 MB). Over APOLLO17_SLDEM2015, it replaces APOLLO17_LANDING_SITE.
    APOLLO17_NAC_DTM,
};

/// Directory the Moon DEM resources are looked up in by default: planet/moon/ under the Chrono data directory.
CH_PLANET_API std::string DataDir();

/// The file and zoom range of a DEM resource in `dir`, ready for ChPlanetSurface::AddGeoTiff.
CH_PLANET_API ChGeoTiffSource GetDem(Dem dem, const std::string& dir = DataDir());

/// A one-line description of a DEM resource.
CH_PLANET_API std::string GetDemDescription(Dem dem);

/// True if the DEM resource's file is present in `dir`.
CH_PLANET_API bool IsDemAvailable(Dem dem, const std::string& dir = DataDir());

/// Load a DEM resource into a surface. Throws std::runtime_error, naming the resource and the file, if
/// the file is missing or unusable.
CH_PLANET_API void AddDem(ChPlanetSurface& surface, Dem dem, const std::string& dir = DataDir());

/// A lunar surface on the given DEM resources, for example
/// `moon::CreateSurface({moon::Dem::GLOBAL_LOW_RES, moon::Dem::APOLLO17_LANDING_SITE})`.
/// Throws std::runtime_error if a resource is missing.
CH_PLANET_API std::shared_ptr<ChPlanetSurface> CreateSurface(const std::vector<Dem>& dems,
                                                             int zoom = 15,
                                                             const std::string& dir = DataDir());

/// @} planet_module

}  // namespace moon
}  // namespace planet
}  // namespace chrono

#endif
