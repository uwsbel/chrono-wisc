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

#ifndef CH_API_PLANET_H
#define CH_API_PLANET_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_PLANET
// (so that the symbols with 'CH_PLANET_API' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_PLANET)
    #define CH_PLANET_API ChApiEXPORT
#else
    #define CH_PLANET_API ChApiIMPORT
#endif

/**
    @defgroup planet_module PLANET module
    @brief Planetary terrain for any body, from GeoTIFF elevation models and procedural relief

    This module provides a headless planetary terrain core that is independent of any particular
    body. A ChPlanetBody describes the body (radius, gravity, geographic coordinate system); a
    ChPlanetSurface on it samples heights (usually from a ChGeoTiffStack of GDAL rasters, each serving
    a range of levels of detail, or any chain of samplers), falls back where the data has holes, and
    passes the result through a chain of filters: procedural relief layers (base swell, craters,
    boulder beds, micro-roughness) and filters that scale, clamp or reshape, globally or by region.
    Everything is a pure function of longitude, latitude and level of detail. ChPlanetQuadtree streams terrain tiles from the same
    surface around a moving viewpoint. Chrono::Vehicle builds rigid and SCM terrains on the surface
    (see vehicle::PlanetTerrain and vehicle::PlanetSCMTerrain).

    Presets for particular bodies (chrono::planet::moon, chrono::planet::mars) live under
    chrono_planet/planets and use only the public API; they provide filter chains and, for the Moon,
    DEM resources selected with moon::Dem, and double as templates for new bodies.

    For additional information, see:
    - the [reference manual](@ref manual_planet)
    - the [installation guide](@ref module_planet_installation)
    - the [tutorials](@ref tutorial_root)
*/

namespace chrono {

/// @addtogroup planet_module
/// @{

/// Namespace with classes for the PLANET module.
namespace planet {}

/// @}

}  // namespace chrono

#endif
