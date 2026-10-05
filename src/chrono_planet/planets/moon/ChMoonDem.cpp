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

#include <fstream>
#include <stdexcept>

#include "chrono/core/ChDataPath.h"

#include "chrono_planet/planets/moon/ChMoonDem.h"

namespace chrono {
namespace planet {
namespace moon {

namespace {

struct DemResource {
    const char* file;
    int min_zoom, max_zoom;
    const char* description;
};

DemResource Resource(Dem dem) {
    switch (dem) {
        case Dem::GLOBAL_LOW_RES:
            return {"ldem_4_global.tif", 0, 2, "LOLA global, 4 px/deg (~7.6 km)"};
        case Dem::GLOBAL:
            return {"ldem_64_fixed.tif", 0, 4, "LOLA global, 64 px/deg (~474 m)"};
        case Dem::APOLLO17_LANDING_SITE:
            return {"ldem_1024_apollo_region.tif", 5, 30, "LOLA Apollo 17 region, 1024 px/deg (~30 m)"};
        case Dem::APOLLO17_SLDEM2015:
            return {"sldem2015_apollo17.tif", 5, 7, "SLDEM2015 around Apollo 17, 512 px/deg (~59 m)"};
        case Dem::APOLLO17_NAC_DTM:
            return {"NAC_DTM_APOLLO17.TIF", 8, 30, "LROC NAC DTM of Taurus-Littrow, 5 m/px"};
    }
    throw std::invalid_argument("moon: unknown DEM resource");
}

std::string Join(const std::string& dir, const char* file) {
    return dir.empty() || dir.back() == '/' ? dir + file : dir + "/" + file;
}

}  // namespace

std::string DataDir() {
    return GetChronoDataFile("planet/moon/");
}

ChGeoTiffSource GetDem(Dem dem, const std::string& dir) {
    const DemResource r = Resource(dem);
    return ChGeoTiffSource{Join(dir, r.file), r.min_zoom, r.max_zoom};
}

std::string GetDemDescription(Dem dem) {
    return Resource(dem).description;
}

bool IsDemAvailable(Dem dem, const std::string& dir) {
    return std::ifstream(GetDem(dem, dir).path).good();
}

void AddDem(ChPlanetSurface& surface, Dem dem, const std::string& dir) {
    const ChGeoTiffSource source = GetDem(dem, dir);
    if (!IsDemAvailable(dem, dir))
        throw std::runtime_error("moon: DEM resource '" + GetDemDescription(dem) + "' not found at " + source.path +
                                 " (see data/planet/moon/README.md)");
    surface.AddGeoTiff(source);
}

std::shared_ptr<ChPlanetSurface> CreateSurface(const std::vector<Dem>& dems, int zoom, const std::string& dir) {
    auto surface = CreateSurface(zoom);
    for (Dem dem : dems)
        AddDem(*surface, dem, dir);
    return surface;
}

}  // namespace moon
}  // namespace planet
}  // namespace chrono
