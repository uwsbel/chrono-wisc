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
// Where bedrock shows through the soil, from the shape of the elevation data.
//
// =============================================================================

#ifndef CH_EXPOSURE_FIELD_H
#define CH_EXPOSURE_FIELD_H

#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <unordered_map>

#include "chrono_planet/ChPlanetBody.h"
#include "chrono_planet/samplers/ChElevationSampler.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// How far the ground at a point is bedrock and how far soil, a pure function of longitude and latitude.
/// Loose material moves downhill and settles in the lows, so rises and steep ground are left bare. The field
/// reads this off the elevation data alone: a point standing above the mean of the ground a reach away is
/// bedrock, as is a slope too steep to hold soil. It changes no height. Relief layers and renderers ask it
/// where the bedrock is, so that sand, rocks and the look of the ground agree.
///
/// Level ground can be bedrock too: a lava floor the wind keeps swept, broken into plates, with soil lying over the
/// rest of it. The data's shape cannot say where, so `floor_cover` of the ground is given to it in patches, by noise.
///
/// The thresholds are a rule of thumb, not a measurement of any site.
class CH_PLANET_API ChExposureField {
  public:
    struct Params {
        double reach = 12.0;         ///< how far round a point its surroundings are taken (m)
        double rise_soil = 0.010;    ///< standing this much of the reach above its surroundings or less: soil
        double rise_rock = 0.035;    ///< standing this much of the reach above them or more: bedrock
        double slope_soil = 0.20;    ///< slope (rise/run) at or under which soil holds
        double slope_rock = 0.40;    ///< slope at or over which none does
        int zoom = 15;               ///< level of detail the data is read at
        double floor_cover = 0.0;    ///< share of level ground that is bedrock too, in patches; 0 for none
        double floor_patch = 60.0;   ///< how far across those patches are (m)
    };

    /// A field over a body's elevation data. Throws std::invalid_argument on inconsistent parameters.
    ChExposureField(const ChPlanetBody& body, std::shared_ptr<const ChElevationSampler> data, const Params& params);

    const Params& GetParams() const { return m_params; }

    /// Share of bedrock at a point, in [0, 1]: 0 soil, 1 bedrock. 0 where the data has no height.
    /// At the parameters' reach it is read between the nodes of a grid a reach apart, each worked out once,
    /// the first time it is needed: nine readings of the data a node, which a layer asking rock by rock could
    /// not afford. `reach` (m), if positive, replaces the parameters', for a coarse mesh to ask at a reach it
    /// can show: worked out at the point each time.
    double GetExposure(double lon_deg, double lat_deg, double reach = 0) const;

    /// The share at the grid's node nearest a point: one node where GetExposure reads four. For a layer that
    /// asks at scattered points, each of which would otherwise have four nodes worked out for it alone.
    double GetExposureNear(double lon_deg, double lat_deg) const;

  private:
    double Node(int i, int j) const;
    double At(double lon_deg, double lat_deg, double reach) const;
    mutable std::shared_mutex m_mutex;
    mutable std::unordered_map<std::int64_t, float> m_nodes;  // by longitude and latitude index
    std::shared_ptr<const ChElevationSampler> m_data;
    Params m_params;
    double m_m_per_deg;
    double m_radius;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
