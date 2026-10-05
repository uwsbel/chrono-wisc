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
// Fields of wind-blown bedforms: ripples and transverse ridges of sand.
//
// =============================================================================

#ifndef CH_BEDFORM_LAYER_H
#define CH_BEDFORM_LAYER_H

#include <memory>

#include "chrono_planet/ChPlanetBody.h"
#include "chrono_planet/procedural/ChExposureField.h"
#include "chrono_planet/procedural/ChReliefLayer.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Wind-blown bedforms of one scale, a pure function of longitude and latitude.
/// The sand lies in separate fields, scattered over the body. Within a field the crests run across the
/// wind, one wavelength apart, and wander. Each crest climbs gently on its upwind side and drops on its
/// lee side. A field has its own wavelength, height and wind, each near the layer's, and thins to
/// nothing at its ragged edge. The relief is never negative: the sand lies on the ground.
/// Bedforms of several scales are several layers.
///
/// The default parameters describe no bedforms (no cover).
class CH_PLANET_API ChBedformLayer : public ChReliefLayer {
  public:
    struct Params {
        double wavelength = 1.0;     ///< crest to crest (m)
        double height = 0.1;         ///< trough to crest (m), at the layer's wavelength; a field's scales with its own
        double wind_azimuth = 0;     ///< where the wind blows to, degrees east of north; crests lie across it
        double azimuth_spread = 0;   ///< a field's wind differs from the layer's by up to this (degrees)
        double lee_fraction = 0.5;   ///< share of a wavelength the lee face takes, in (0, 1); 0.5 is symmetric
        double sinuosity = 0.5;      ///< how far a crest wanders along its length, in wavelengths
        double cover = 0;            ///< share of the ground under fields, about; at most 0.9
        double field_size = 100.0;   ///< typical width of a field (m)
        int seed = 0;                ///< varies the realization
        /// Where bedrock shows, if given: sand settles in the lows, so a field whose middle is on bedrock is left
        /// out, the likelier the barer the ground there. The cover asked for is then of the soil alone.
        std::shared_ptr<const ChExposureField> bedrock;
    };

    /// Construct the layer for a body. Throws std::invalid_argument on inconsistent parameters.
    ChBedformLayer(const ChPlanetBody& body, const Params& params);
    ~ChBedformLayer();

    const Params& GetParams() const;

    /// How fully a point lies in a field, in [0, 1]: 0 on bare ground, 1 well inside a field.
    /// It does not depend on the sampling spacing, so it says where the sand is at any level of detail.
    double GetCover(double lon_deg, double lat_deg) const;

    virtual double GetHeight(double lon_deg, double lat_deg, double spacing_deg) const override;

    struct Model;

  private:
    std::unique_ptr<const Model> m_model;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
