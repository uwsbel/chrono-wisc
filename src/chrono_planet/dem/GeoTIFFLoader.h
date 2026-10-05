#ifndef CH_PLANET_GEOTIFFLOADER_H
#define CH_PLANET_GEOTIFFLOADER_H

// Internal: includes GDAL headers. Not installed; use ChGeoTiffStack instead.

#include <array>
#include <string>
#include <vector>

#include <ogr_spatialref.h>

namespace chrono {
namespace planet {

// One raster band, read whole into memory as doubles.
class GeoTIFFLoader {
public:
    // Loads a band (1-based). Throws std::runtime_error on failure, leaving the previous raster intact.
    void load(const std::string& filename, int band = 1);

    int width() const { return width_; }
    int height() const { return height_; }
    // Row-major samples, height() rows of width().
    const std::vector<double>& elevation() const { return elevation_; }
    std::vector<double>& elevation() { return elevation_; }
    // GDAL affine transform from (pixel, line) to dataset coordinates.
    const std::array<double, 6>& geoTransform() const { return geoTransform_; }
    // The dataset's coordinate system. Meaningful only when hasSrs().
    const OGRSpatialReference& srs() const { return srs_; }
    bool hasSrs() const { return hasSrs_; }
    // True for lon/lat rasters, including those with no coordinate system at all.
    bool isGeographic() const { return !hasSrs_ || srs_.IsGeographic(); }
    // The band's nodata sentinel, if it declares one.
    bool hasNodata() const { return hasNodata_; }
    double nodata() const { return nodata_; }
    void setNodata(double v) { hasNodata_ = true, nodata_ = v; }
    // The band's own value scale and offset (1 and 0 unless the file declares them).
    double bandScale() const { return bandScale_; }
    double bandOffset() const { return bandOffset_; }

private:
    int width_ = 0, height_ = 0;
    std::vector<double> elevation_;
    std::array<double, 6> geoTransform_{};
    OGRSpatialReference srs_;
    bool hasSrs_ = false;
    bool hasNodata_ = false;
    double nodata_ = 0.0;
    double bandScale_ = 1.0, bandOffset_ = 0.0;
};

}  // namespace planet
}  // namespace chrono

#endif  // CH_PLANET_GEOTIFFLOADER_H
