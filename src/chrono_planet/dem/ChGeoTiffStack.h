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
// A stack of GeoTIFF (or any GDAL raster) elevation models, each valid over a
// range of quadtree zooms, blended into one elevation source.
//
// =============================================================================

#ifndef CH_GEOTIFF_STACK_H
#define CH_GEOTIFF_STACK_H

#include <memory>
#include <optional>
#include <string>

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChPlanetBody.h"
#include "chrono_planet/samplers/ChElevationSampler.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// One elevation raster and how to read it.
/// Any raster GDAL can open works (GeoTIFF, PDS, ISIS cubes, ...), in geographic longitude/latitude or
/// in any projection GDAL can invert. Projected rasters are transformed through the body's geographic
/// coordinate system (see ChPlanetBody::SetGeographicSRS); a raster with no coordinate system is read
/// as longitude/latitude on the body. Longitudes may run -180..180 or 0..360.
/// A projected raster whose axes run along longitude and latitude over a sampling grid (an equirectangular one) is
/// interpolated in two passes. Any other (polar stereographic, say) is transformed and interpolated sample by
/// sample, to the same heights.
///
/// A stored value v becomes a height h above the body's reference sphere as
///   h = (v * band_scale + band_offset) * scale + offset   [- body radius, if values_are_radii]
/// where band_scale and band_offset are the band's own metadata (1 and 0 if absent or ignored).
///
/// Known limitations:
/// - Each pixel's value is placed at the geotransform position of the pixel, its upper-left corner,
///   whatever the file's AREA_OR_POINT registration says. Area-registered rasters (the GeoTIFF default)
///   therefore read half a pixel north-west of their true position.
/// - A global raster is not bridged across its own wrap edge: the last pixel column before it has no data.
struct CH_PLANET_API ChGeoTiffSource {
    std::string path;               ///< file path, absolute or relative to the working directory
    int min_zoom = 0;               ///< coarsest quadtree zoom this raster serves
    int max_zoom = 30;              ///< finest quadtree zoom this raster serves
    int band = 1;                   ///< 1-based raster band holding the elevations
    double scale = 1.0;             ///< multiplier applied to the values, e.g. 1000 for kilometers
    double offset = 0.0;            ///< offset added after scaling (m)
    bool values_are_radii = false;  ///< values are distances from the body center, not heights
    bool use_band_scale_offset = true;  ///< apply the band's own scale/offset metadata first
    std::optional<double> nodata;       ///< nodata value, overriding the band's own
    double min_valid = -30000;          ///< heights below this (m) are treated as nodata
    double feather_m = 0;               ///< ground distance (m) over which an overlay fades into what is under it at
                                        ///< its edges and holes; 0: the stack's default, 500 m. At most 255 pixels
};

/// Elevation source backed by a stack of rasters, each valid over a range of quadtree zooms.
/// At a given zoom, the raster whose zoom range contains it (the narrowest, if several do) provides the
/// height; where it has holes or ends, it fades over a ground distance to the finest coarser raster
/// that covers the area. Where nothing covers a point, the source reports no data.
/// Nodata pixels are never interpolated into heights, so a base raster with holes reports no data
/// there and the surface's fallback fills them.
class CH_PLANET_API ChGeoTiffStack : public ChElevationSampler {
  public:
    /// Construct an empty stack for the given body.
    explicit ChGeoTiffStack(const ChPlanetBody& body);
    ~ChGeoTiffStack();

    ChGeoTiffStack(const ChGeoTiffStack&) = delete;
    ChGeoTiffStack& operator=(const ChGeoTiffStack&) = delete;

    /// Load a raster into the stack. Throws std::runtime_error, with the reason, if the file cannot be
    /// opened or its coordinate system cannot be transformed. Not thread-safe with queries.
    void AddSource(const ChGeoTiffSource& source);

    /// Number of rasters loaded.
    int GetNumSources() const;

    /// Seamless compositing (default: off).
    ///
    /// Off, a query's zoom picks the rasters: the one whose zoom range holds it, faded into the finest coarser one
    /// that covers the query. Two rasters of one place never hold the same heights, so along the line where a tile of
    /// one zoom meets a tile of the next, where the pick changes, the ground steps by their difference: a wall.
    ///
    /// On, the pick depends on where a point is, not on the zoom asked. The base is the whole-body raster
    /// (min_zoom 0) the zoom picks, as before. Over it every overlay (min_zoom over 0) the query meets is laid, in the
    /// order the overlays were added, coarser first, each fading into what is under it over its own feather
    /// (ChGeoTiffSource::feather_m) at its edges and holes. An overlay's zoom range is then not used. A coarse tile
    /// over a fine overlay reads that overlay at the tile's own spacing, without filtering it first.
    ///
    /// With a feather as wide as the rasters differ at an overlay's edge is steep, this gives one ground at every
    /// level of detail, for any stack of rasters. Not thread-safe with queries.
    void SetSeamless(bool seamless);
    bool IsSeamless() const;

    const ChPlanetBody& GetBody() const { return m_body; }

    virtual std::optional<double> GetHeight(double lon_deg, double lat_deg, int zoom) const override;

    virtual void GetHeightGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& heights) const override;

  private:
    ChPlanetBody m_body;
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
