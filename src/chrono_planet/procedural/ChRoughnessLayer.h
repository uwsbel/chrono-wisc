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
// Multi-octave micro-roughness below the resolution of the elevation data.
//
// =============================================================================

#ifndef CH_ROUGHNESS_LAYER_H
#define CH_ROUGHNESS_LAYER_H

#include <memory>
#include <vector>

#include "chrono_planet/ChPlanetBody.h"
#include "chrono_planet/procedural/ChReliefLayer.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Small-scale surface roughness: octaves of noise, each half the wavelength of the one before, with
/// an RMS slope that grows toward the fine end. Octaves longer than `clod_below` are shaped Perlin
/// noise; finer ones are scattered clods and pits over a gentler Perlin bed.
///
/// The default parameters describe a smooth surface (zero slope).
class CH_PLANET_API ChRoughnessLayer : public ChReliefLayer {
  public:
    struct Params {
        double coarsest_wavelength = 4.0;  ///< wavelength of the first octave (m)
        int octaves = 8;                   ///< number of octaves
        double slope_rms = 0;              ///< RMS slope (rise/run) of the first octave
        double fine_boost = 1.4;           ///< slope multiplier per octave toward the fine end
        double clod_below = 0.4;           ///< octaves with wavelength below this (m) are clods and pits
        double clod_variance_share = 0.25; ///< share of a fine octave's slope variance carried by clods
        int seed = 0;                      ///< varies the realization; 0 reproduces the reference field
    };

    /// Construct the layer for a body. Throws std::invalid_argument on inconsistent parameters.
    ChRoughnessLayer(const ChPlanetBody& body, const Params& params);
    ~ChRoughnessLayer();

    const Params& GetParams() const;

    /// RMS slope (rise/run) of the octaves resolved at the given sampling spacing (degrees).
    double GetRmsSlope(double spacing_deg) const;

    virtual double GetHeight(double lon_deg, double lat_deg, double spacing_deg) const override;
    virtual void AddToGrid(const ChGeoGrid& grid, std::vector<double>& heights) const override;

    struct Model;

  private:
    std::unique_ptr<const Model> m_model;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
