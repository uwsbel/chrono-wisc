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
// Procedural relief laid in whichever of two frames a place is nearer the
// equator of, so it is about isotropic over the whole body.
//
// =============================================================================

#ifndef CH_ISOTROPIC_RELIEF_H
#define CH_ISOTROPIC_RELIEF_H

#include <memory>
#include <vector>

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/filters/ChSurfaceFilter.h"
#include "chrono_planet/procedural/ChRockLayer.h"
#include "chrono_planet/samplers/ChElevationSampler.h"

namespace chrono {
namespace planet {

class ChPlanetSurface;

/// @addtogroup planet_module
/// @{

/// Relief that is about isotropic over the whole body.
///
/// The relief layers place their features on a lattice of longitude and latitude. Its cells are square in degrees,
/// so on the ground they narrow as the cosine of the latitude: at 85 degrees they are 12 times taller than wide.
/// There craters crowd 12 to the place of one and are cut off east and west where they outgrow their cells, bumps
/// of roughness are 12 times narrower than long, and a tile misses the features of cells too far east or west of
/// it, so they come and go as the tiles change.
///
/// This filter wraps such relief and lays it in two frames. Up to `blend_from_deg` of latitude it is the wrapped
/// relief as it is: the same heights, to the bit. Past `blend_to_deg` the relief is asked for the same place in a
/// second frame, the body's turned a quarter turn about its x axis, so that the body's poles lie on that frame's
/// equator, at 90 degrees east and west. Between the two latitudes the two are blended. A place is then never more
/// than about 45 degrees from the equator of the frame its relief is laid in, and no cell is narrower than 0.7 of its
/// height. Any relief layer, as it is, serves the whole body so.
///
/// What it costs:
/// - The wrapped relief must be additive: what it adds to a height must not depend on the height. Relief layers are.
/// - In the polar frame a grid is asked sample by sample, in parallel, where the relief's own grid pass is faster.
/// - In the blend band two independent reliefs are mixed, so features are shallower there: keep the band narrow.
/// - Given one relief, the relief about a pole is what its layers lay about the equator at 90 degrees east or west,
///   turned: the same features stand in two places. Give a second, of another seed, for the polar frame.
/// - A layer that reads the elevation data (through a ChExposureField, say) is asked in the polar frame for a place
///   that is not where the data has it. Give the polar frame a relief of its own, whose layers read the data through
///   PolarFrameSampler.
/// - Rocks are placed in the frame their relief is laid in: ask for them through QueryRocks.
class CH_PLANET_API ChIsotropicRelief : public ChSurfaceFilter {
  public:
    /// Wrap `relief`, a layer or a chain of layers. Throws std::invalid_argument unless
    /// 0 < blend_from_deg < blend_to_deg < 90.
    explicit ChIsotropicRelief(std::shared_ptr<ChSurfaceFilter> relief, double blend_from_deg = 44.0, double blend_to_deg = 46.0);
    /// Wrap two reliefs: `relief` for the body's own frame and `polar_relief` for the polar frame. The second is
    /// asked for places in the polar frame's longitude and latitude: its layers may be of another seed, and must read
    /// any elevation data through PolarFrameSampler.
    ChIsotropicRelief(std::shared_ptr<ChSurfaceFilter> relief,
                      std::shared_ptr<ChSurfaceFilter> polar_relief,
                      double blend_from_deg = 44.0,
                      double blend_to_deg = 46.0);

    /// The wrapped relief, the body's own frame's.
    std::shared_ptr<ChSurfaceFilter> GetRelief() const { return m_relief; }
    /// The polar frame's: the same, unless a second was given.
    std::shared_ptr<ChSurfaceFilter> GetPolarRelief() const { return m_polar; }

    /// Elevation data as the polar frame has it: asked for a place in the polar frame's longitude and latitude, it
    /// gives `data` at that place on the body. For the layers of a polar relief that read the data.
    static std::shared_ptr<ChElevationSampler> PolarFrameSampler(std::shared_ptr<const ChElevationSampler> data);

    /// The share of the relief taken from the polar frame at a latitude: 0 up to blend_from_deg, 1 past blend_to_deg.
    double GetPolarWeight(double lat_deg) const;

    /// A place's longitude and latitude (degrees) in the polar frame.
    static void ToPolarFrame(double lon_deg, double lat_deg, double& frame_lon_deg, double& frame_lat_deg);
    /// The body's longitude and latitude (degrees) of a place given in the polar frame.
    static void FromPolarFrame(double frame_lon_deg, double frame_lat_deg, double& lon_deg, double& lat_deg);
    /// The angle (rad) from the body's east to the polar frame's east at a place, counter-clockwise seen from above.
    /// What is turned by an angle in the polar frame's east-north-up is turned by that angle plus this in the body's.
    static double GetFrameTurn(double lon_deg, double lat_deg);

    /// Rocks with radius >= min_radius (m) whose centers lie in a longitude/latitude rectangle (degrees) of the body,
    /// half-open on the max edges, as ChRockLayer::Query gives them: each where it stands on the body, and turned as
    /// it is there. In the blend band each rock of either frame is kept or left out whole, by its own id, so that as
    /// many stand there as elsewhere. Empty if the wrapped relief has no rock layer.
    std::vector<ChRockInstance> QueryRocks(double min_lon, double min_lat, double max_lon, double max_lat, double min_radius) const;

    /// The rock layer of a surface, wrapped or not; null with none. For its parameters: ask for its rocks through
    /// QueryRocks.
    static std::shared_ptr<ChRockLayer> FindRocks(const ChPlanetSurface& surface);
    /// The rocks of a surface in a rectangle, whichever way its relief is laid: through its ChIsotropicRelief if it
    /// has one, else its rock layer's own. Empty with neither.
    static std::vector<ChRockInstance> QueryRocks(const ChPlanetSurface& surface,
                                                  double min_lon,
                                                  double min_lat,
                                                  double max_lon,
                                                  double max_lat,
                                                  double min_radius);

    virtual double Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const override;
    virtual void ApplyGrid(const ChGeoGrid& grid, std::vector<double>& heights) const override;

  private:
    std::shared_ptr<ChSurfaceFilter> m_relief;
    std::shared_ptr<ChSurfaceFilter> m_polar;
    std::shared_ptr<ChRockLayer> m_rocks;        ///< the wrapped relief's, null with none
    std::shared_ptr<ChRockLayer> m_polar_rocks;  ///< the polar relief's
    double m_blend_from, m_blend_to;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
