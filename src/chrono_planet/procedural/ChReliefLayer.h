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
// Procedural relief added on top of the elevation data.
//
// =============================================================================

#ifndef CH_RELIEF_LAYER_H
#define CH_RELIEF_LAYER_H

#include <vector>

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChGeoGrid.h"
#include "chrono_planet/filters/ChSurfaceFilter.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Base class for procedural relief: an additive surface filter, whose height offset (m) is a pure
/// function of longitude, latitude and sampling resolution.
///
/// Layers must be deterministic and thread-safe, since physics and rendering evaluate them
/// independently and must agree. The sampling spacing lets a layer drop features too small to be
/// resolved (they would alias), so a coarse level of detail and a fine one see consistent terrain.
/// The module provides a base swell (ChBaseReliefLayer), craters (ChCraterLayer), boulder beds
/// (ChRockLayer) and micro-roughness (ChRoughnessLayer); derive from this class for others.
class CH_PLANET_API ChReliefLayer : public ChSurfaceFilter {
  public:
    virtual ~ChReliefLayer();

    /// Relief (m) at a longitude and latitude (degrees). The longitude is wrapped to [-180, 180).
    /// Features that cannot be resolved at `spacing_deg` should fade out.
    virtual double GetHeight(double lon_deg, double lat_deg, double spacing_deg) const = 0;

    /// Add this layer's relief to an n x n row-major grid of heights, filtered to grid.spacing.
    /// The grid never crosses the dateline; its lon0 is wrapped to [-180, 180).
    /// The result must match GetHeight at every sample to within rounding. The default calls
    /// GetHeight per sample; override it when a grid pass is cheaper.
    virtual void AddToGrid(const ChGeoGrid& grid, std::vector<double>& heights) const;

    /// As a filter: the incoming height plus this layer's relief.
    virtual double Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const override final {
        return height + GetHeight(lon_deg, lat_deg, spacing_deg);
    }
    virtual void ApplyGrid(const ChGeoGrid& grid, std::vector<double>& heights) const override final {
        AddToGrid(grid, heights);
    }
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
