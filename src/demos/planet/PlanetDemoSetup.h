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
// Demo helper: the Moon surface every planet demo runs on, from GeoTIFF paths
// given on the command line or else the Moon DEM resources shipped with Chrono.
//
// =============================================================================

#ifndef DEMO_PLANET_DEMO_SETUP_H
#define DEMO_PLANET_DEMO_SETUP_H

#include <exception>
#include <iostream>
#include <memory>
#include <vector>

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/planets/moon/ChMoon.h"
#include "chrono_planet/planets/moon/ChMoonDem.h"

/// The Moon surface a demo runs on, shared by its physics and its renderer.
/// With GeoTIFF paths, those form the DEM stack. Without, the shipped Moon DEM resources do: the low
/// resolution global model and the Apollo 17 landing site (add moon::Dem::GLOBAL for the 64 px/deg
/// global model once its file is installed). Prints the reason and returns null if a DEM cannot be used.
inline std::shared_ptr<chrono::planet::ChPlanetSurface> CreateDemoSurface(
    const std::vector<chrono::planet::ChGeoTiffSource>& geotiffs,
    int zoom,
    double root_tile_deg = 16.0) {
    using namespace chrono::planet;
    std::shared_ptr<ChPlanetSurface> surface;
    try {
        if (geotiffs.empty())
            surface = moon::CreateSurface({moon::Dem::GLOBAL_LOW_RES, moon::Dem::APOLLO17_LANDING_SITE}, zoom);
        else
            surface = moon::CreateSurface(geotiffs, zoom);
        surface->SetRootTileSize(root_tile_deg);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return nullptr;
    }
    return surface;
}

/// The Moon surface around the Apollo 17 site from the best Moon DEM resources installed, for demos that view
/// the site from altitude or at the scale of meters. With GeoTIFF paths, those form the DEM stack, as in
/// CreateDemoSurface. Without, the low resolution global model, then each of the 64 px/deg global model,
/// SLDEM2015 around the site and the LROC NAC DTM of the site that is installed (see
/// data/planet/moon/README.md), and the shipped LOLA site grid only if the NAC DTM is not: that grid is
/// striped by LOLA's ground tracks. Prints the stack, and returns null if a DEM cannot be used.
inline std::shared_ptr<chrono::planet::ChPlanetSurface> CreateSiteSurface(
    const std::vector<chrono::planet::ChGeoTiffSource>& geotiffs,
    int zoom,
    double root_tile_deg = 16.0) {
    using namespace chrono::planet;
    if (!geotiffs.empty())
        return CreateDemoSurface(geotiffs, zoom, root_tile_deg);
    std::vector<moon::Dem> stack = {moon::Dem::GLOBAL_LOW_RES};
    for (auto dem : {moon::Dem::GLOBAL, moon::Dem::APOLLO17_SLDEM2015, moon::Dem::APOLLO17_NAC_DTM}) {
        if (moon::IsDemAvailable(dem))
            stack.push_back(dem);
        else
            std::cout << "Not installed (see data/planet/moon/README.md): " << moon::GetDemDescription(dem) << std::endl;
    }
    if (!moon::IsDemAvailable(moon::Dem::APOLLO17_NAC_DTM))
        stack.push_back(moon::Dem::APOLLO17_LANDING_SITE);
    for (auto dem : stack)
        std::cout << "DEM: " << moon::GetDemDescription(dem) << std::endl;
    std::shared_ptr<ChPlanetSurface> surface;
    try {
        surface = moon::CreateSurface(stack, zoom);
        surface->SetRootTileSize(root_tile_deg);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return nullptr;
    }
    return surface;
}

#endif
