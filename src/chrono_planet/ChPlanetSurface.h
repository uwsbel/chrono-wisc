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
// Deterministic planet surface: elevation samplers, a fallback where they have
// no data, and a chain of filters, evaluated at any level of detail.
//
// =============================================================================

#ifndef CH_PLANET_SURFACE_H
#define CH_PLANET_SURFACE_H

#include <memory>
#include <vector>

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChGeoGrid.h"
#include "chrono_planet/ChLevelOfDetail.h"
#include "chrono_planet/ChPlanetBody.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/samplers/ChElevationSampler.h"
#include "chrono_planet/dem/ChGeoTiffStack.h"
#include "chrono_planet/filters/ChSurfaceFilter.h"
#include "chrono_planet/procedural/ChReliefLayer.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// The terrain of a planetary body, as one height function shared by physics and rendering.
///
/// Height at a point = the sampler's height (or the fallback's, where the sampler has no data),
///                     passed through the filter chain, first filter to last.
///
/// Samplers (ChElevationSampler) produce heights: a GeoTIFF stack, a chain of samplers, or your own.
/// Filters (ChSurfaceFilter) transform them: procedural relief layers add craters, rocks and roughness;
/// other filters scale, clamp or reshape, everywhere or within a region; your own filters plug in the
/// same way. Presets provide ready-made chains (for example moon::FilterChain()).
///
/// Everything is a pure function of longitude, latitude and level of detail, so a quadtree renderer
/// (ChPlanetQuadtree) and a physics terrain (vehicle::PlanetTerrain, vehicle::PlanetSCMTerrain) built on
/// the same surface agree exactly. Physics queries use a fixed zoom (SetZoom); at that zoom the surface
/// resolves the same relief the renderer's mesh does, so wheels and pixels ride one surface.
///
/// Configure the surface (samplers, fallback, filters, zoom) before sharing it; queries are thread-safe,
/// configuration is not.
///
/// A minimal setup for any body:
/// \code
/// ChPlanetBody body("Mars", 3389500.0, 3.721);
/// auto surface = chrono_types::make_shared<ChPlanetSurface>(body);
/// surface->AddGeoTiff({"mola_global.tif", 0, 4});
/// surface->AddGeoTiff({"hirise_site.tif", 5, 30});
/// \endcode
/// Presets for particular bodies live under chrono_planet/planets (for example planets/moon/ChMoon.h).
class CH_PLANET_API ChPlanetSurface {
  public:
    /// Construct a flat surface on the given body, sampled for physics at the given zoom.
    explicit ChPlanetSurface(const ChPlanetBody& body, int zoom = 15);
    ~ChPlanetSurface();

    ChPlanetSurface(const ChPlanetSurface&) = delete;
    ChPlanetSurface& operator=(const ChPlanetSurface&) = delete;

    /// The body this surface lies on.
    const ChPlanetBody& GetBody() const { return m_body; }

    // --- Configuration ---

    /// Set the elevation sampler (null: none, the fallback applies everywhere).
    void SetSampler(std::shared_ptr<ChElevationSampler> sampler) { m_sampler = sampler; }
    std::shared_ptr<ChElevationSampler> GetSampler() const { return m_sampler; }

    /// Load a raster into the surface's GeoTIFF stack, creating the stack as the sampler on first use.
    /// Throws std::runtime_error if the file cannot be used, and std::logic_error if a different kind
    /// of sampler was set (put a ChGeoTiffStack in a ChSamplerChain to combine them).
    void AddGeoTiff(const ChGeoTiffSource& source);

    /// The GeoTIFF stack, or null if the sampler is not one.
    std::shared_ptr<ChGeoTiffStack> GetGeoTiffStack() const;

    /// Set the sampler consulted where the main sampler has no data (null: height 0).
    /// It should cover every point; where it has no data either, the height is 0.
    void SetFallback(std::shared_ptr<ChElevationSampler> fallback) { m_fallback = fallback; }
    std::shared_ptr<ChElevationSampler> GetFallback() const { return m_fallback; }

    /// The filter chain, applied to the sampled heights. Never null; modify it in place to add,
    /// insert or remove filters.
    const std::shared_ptr<ChFilterChain>& GetFilterChain() const { return m_filters; }
    /// Replace the filter chain (null: an empty chain).
    void SetFilterChain(std::shared_ptr<ChFilterChain> chain);
    /// Append a filter to the chain.
    void AddFilter(std::shared_ptr<ChSurfaceFilter> filter) { m_filters->AddFilter(std::move(filter)); }
    /// Append a relief layer to the chain (same as AddFilter).
    void AddLayer(std::shared_ptr<ChReliefLayer> layer) { m_filters->AddFilter(std::move(layer)); }

    /// A view of this surface with extra filters after its own: it shares this surface's samplers,
    /// fallback and level of detail, and its chain runs this surface's chain (live, so later edits
    /// show through) followed by `extra`. Use it to draw what physics should not feel, typically a
    /// ChDeformationFilter holding the ruts of a soil model that samples this surface as its
    /// undisturbed ground.
    std::shared_ptr<ChPlanetSurface> CreateView(std::shared_ptr<ChSurfaceFilter> extra) const;

    /// First filter of the given type in the chain, searching nested chains, or null.
    template <class T>
    std::shared_ptr<T> FindFilter() const {
        return m_filters->Find<T>();
    }

    /// Set the zoom (quadtree level) that physics queries sample at. Default: 15.
    void SetZoom(int zoom) { m_zoom = zoom; }
    int GetZoom() const { return m_zoom; }

    /// Set the relief filter spacing (degrees) for physics queries. Zero (the default) derives it from
    /// the zoom, matching the vertex spacing of the renderer's mesh at that level.
    void SetSampleSpacing(double spacing_deg) { m_spacing = spacing_deg; }
    /// Relief filter spacing (degrees) of physics queries.
    double GetSampleSpacing() const;

    /// Set the level-of-detail schedule: the root tile size, which fixes the relief spacing at each zoom.
    /// Default: 16-degree root tiles. Must be set before a ChPlanetQuadtree is built on this surface.
    void SetLevelOfDetail(const ChLevelOfDetail& lod) { m_lod = lod; }
    const ChLevelOfDetail& GetLevelOfDetail() const { return m_lod; }

    /// Shortcut for SetLevelOfDetail(ChLevelOfDetail(size_deg)).
    void SetRootTileSize(double size_deg) { m_lod = ChLevelOfDetail(size_deg); }
    double GetRootTileSize() const { return m_lod.GetRootTileSize(); }

    /// Relief spacing (degrees) at a zoom: the vertex spacing of a tile mesh at that zoom.
    double GetSampleSpacingAtZoom(int zoom) const { return m_lod.GetVertexSpacing(zoom); }

    // --- Physics queries, at the fixed zoom ---

    /// Surface height (m above the reference sphere) at a longitude and latitude (degrees).
    double GetElevation(double lon_deg, double lat_deg) const;

    /// Fill `out` with an n x n row-major grid of heights starting at (lon0, lat0) with the given steps.
    void GetElevationGrid(double lon0,
                          double lat0,
                          double step_lon,
                          double step_lat,
                          int n,
                          std::vector<double>& out) const;

    /// A site frame on this body with its origin on the surface at the given point.
    ChSiteFrame MakeSiteFrame(double lon_deg, double lat_deg) const;

    // --- Queries at any level of detail ---

    /// Height at a point for the given zoom, with relief filtered to `spacing_deg`
    /// (zero or negative: the mesh spacing at that zoom).
    double GetElevationAtZoom(double lon_deg, double lat_deg, int zoom, double spacing_deg = 0) const;

    /// Heights over a grid for the given zoom, relief filtered to grid.spacing. The grid may
    /// cross the dateline.
    void GetElevationGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& out) const;

    /// Grid evaluation in two stages, split at the first dynamic filter of the (flattened) chain, see
    /// ChSurfaceFilter::IsDynamic. The static stage runs the sampler, the fallback and every filter before
    /// it; the dynamic stage runs the rest. Running both equals GetElevationGrid exactly, so a consumer can
    /// cache the static stage and redo only the dynamic one when a dynamic filter changes.
    void GetStaticElevationGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& out) const;
    void ApplyDynamicFilters(const ChGeoGrid& grid, std::vector<double>& heights) const;

    /// True if the filter chain holds a dynamic filter.
    bool HasDynamicFilters() const { return m_filters->IsDynamic(); }

    /// The sampler's height alone at a point and zoom, or nullopt where it has none.
    std::optional<double> GetDataElevation(double lon_deg, double lat_deg, int zoom) const;

  private:
    double Fallback(double lon_deg, double lat_deg, int zoom) const;
    // Sample the data and fill missing heights from the fallback, before any filters.
    void SampleElevationGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& heights) const;
    // The flattened chain, split at its first dynamic filter.
    void SplitFilters(std::vector<const ChSurfaceFilter*>& statics, std::vector<const ChSurfaceFilter*>& dynamics) const;
    // Apply filters to a grid, per sample if the grid crosses the dateline.
    void ApplyFilters(const std::vector<const ChSurfaceFilter*>& filters, const ChGeoGrid& grid, std::vector<double>& heights) const;

    ChPlanetBody m_body;
    std::shared_ptr<ChElevationSampler> m_sampler;
    std::shared_ptr<ChElevationSampler> m_fallback;
    std::shared_ptr<ChFilterChain> m_filters;
    ChLevelOfDetail m_lod;
    int m_zoom;
    double m_spacing;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
