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
// Elevation samplers: anything that answers height above the reference sphere
// at a longitude, latitude and level of detail.
//
// =============================================================================

#ifndef CH_ELEVATION_SAMPLER_H
#define CH_ELEVATION_SAMPLER_H

#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChGeoGrid.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Base class for elevation samplers: heights in meters above the body's reference sphere.
/// A source may cover only part of the planet; where it has no data it says so, and the surface
/// consults its fallback there. Implementations must be safe to query from several threads at once.
/// The GeoTIFF stack (ChGeoTiffStack) is the usual implementation; derive from this class to plug in
/// other formats, analytic terrain, or test data.
class CH_PLANET_API ChElevationSampler {
  public:
    virtual ~ChElevationSampler();

    /// Height (m) at a longitude and latitude (degrees), for a level of detail `zoom`
    /// (0 is the coarsest quadtree level). Return nullopt where the source has no data.
    virtual std::optional<double> GetHeight(double lon_deg, double lat_deg, int zoom) const = 0;

    /// Heights over a grid (n x n, row-major), NaN where the source has no data. The grid's longitudes
    /// are not wrapped. The default calls GetHeight per sample; override it when grid evaluation is cheaper.
    virtual void GetHeightGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& heights) const;
};

/// Smooth 2D Perlin noise in longitude and latitude, defined everywhere.
/// Useful as a fallback where no DEM covers the ground, so terrain is never perfectly flat.
class CH_PLANET_API ChNoiseElevation : public ChElevationSampler {
  public:
    /// Noise of the given amplitude (m) with `frequency` lattice cells per degree.
    ChNoiseElevation(double amplitude, double frequency);

    virtual std::optional<double> GetHeight(double lon_deg, double lat_deg, int zoom) const override;

  private:
    double m_amplitude;
    double m_frequency;
};

/// Elevation from a user function of longitude and latitude (degrees), defined everywhere.
/// For example, a constant datum: ChElevationFunction([](double, double) { return -1500.0; }).
class CH_PLANET_API ChElevationFunction : public ChElevationSampler {
  public:
    using Function = std::function<double(double lon_deg, double lat_deg)>;

    explicit ChElevationFunction(Function f);

    virtual std::optional<double> GetHeight(double lon_deg, double lat_deg, int zoom) const override;

  private:
    Function m_function;
};

/// An ordered chain of samplers: at each point, the first sampler with data answers.
/// Use it to layer data sources beyond one GeoTIFF stack, for example a mission DEM served by a
/// custom sampler in front of a global stack, or an analytic sampler behind everything.
class CH_PLANET_API ChSamplerChain : public ChElevationSampler {
  public:
    ChSamplerChain() = default;
    explicit ChSamplerChain(std::vector<std::shared_ptr<ChElevationSampler>> samplers);

    /// Append a sampler, consulted after the ones already in the chain.
    void AddSampler(std::shared_ptr<ChElevationSampler> sampler);
    const std::vector<std::shared_ptr<ChElevationSampler>>& GetSamplers() const { return m_samplers; }

    virtual std::optional<double> GetHeight(double lon_deg, double lat_deg, int zoom) const override;
    virtual void GetHeightGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& heights) const override;

  private:
    std::vector<std::shared_ptr<ChElevationSampler>> m_samplers;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
