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
// Low-frequency Perlin swell on the sphere.
//
// =============================================================================

#ifndef CH_BASE_RELIEF_LAYER_H
#define CH_BASE_RELIEF_LAYER_H

#include "chrono_planet/procedural/ChReliefLayer.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Broad, smooth undulation over the whole body: Perlin noise sampled on the unit sphere, so it has
/// no seams or polar pinching. Its wavelength scales with the body (about radius * 2 / frequency).
class CH_PLANET_API ChBaseReliefLayer : public ChReliefLayer {
  public:
    struct Params {
        double frequency = 20;  ///< lattice cells per unit of the sphere's radius
        double amplitude = 0;   ///< peak height, roughly (m)
    };

    explicit ChBaseReliefLayer(const Params& params);

    const Params& GetParams() const { return m_params; }

    virtual double GetHeight(double lon_deg, double lat_deg, double spacing_deg) const override;
    virtual void AddToGrid(const ChGeoGrid& grid, std::vector<double>& heights) const override;

  private:
    Params m_params;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
