#include "chrono_planet/dem/GeoTIFFLoader.h"

#include <iostream>
#include <memory>
#include <stdexcept>

#include <gdal_priv.h>

namespace chrono {
namespace planet {

namespace {

// The dataset's coordinate system, or false when it declares none that parses.
bool datasetSRS(GDALDataset* ds, OGRSpatialReference& srs) {
    if (const char* wkt = ds->GetProjectionRef(); wkt && *wkt) {
        if (srs.SetFromUserInput(wkt) == OGRERR_NONE && srs.Validate() == OGRERR_NONE) {
            return true;
        }
        std::cerr << "[GeoTIFFLoader] WKT present but could not be parsed; falling back to GetSpatialRef().\n";
    }
#if GDAL_VERSION_MAJOR >= 3
    if (const OGRSpatialReference* live = ds->GetSpatialRef()) {
        std::unique_ptr<OGRSpatialReference> copy(live->Clone());
        if (copy && copy->Validate() == OGRERR_NONE) {
            srs = *copy;
            return true;
        }
    }
#endif
    return false;
}

}  // namespace

void GeoTIFFLoader::load(const std::string& filename, int bandIndex) {
    GDALAllRegister();
    std::unique_ptr<GDALDataset, decltype(&GDALClose)> ds(
        static_cast<GDALDataset*>(GDALOpen(filename.c_str(), GA_ReadOnly)), GDALClose);
    if (!ds) {
        throw std::runtime_error("cannot open raster file: " + filename);
    }

    GeoTIFFLoader loaded;
    loaded.width_ = ds->GetRasterXSize();
    loaded.height_ = ds->GetRasterYSize();
    if (ds->GetGeoTransform(loaded.geoTransform_.data()) != CE_None) {
        throw std::runtime_error("raster has no geotransform: " + filename);
    }

    if (bandIndex < 1 || bandIndex > ds->GetRasterCount()) {
        throw std::runtime_error("raster has no band " + std::to_string(bandIndex) + " (it has " +
                                 std::to_string(ds->GetRasterCount()) + "): " + filename);
    }
    GDALRasterBand* band = ds->GetRasterBand(bandIndex);

    loaded.elevation_.resize(static_cast<std::size_t>(loaded.width_) * loaded.height_);
    if (band->RasterIO(GF_Read, 0, 0, loaded.width_, loaded.height_, loaded.elevation_.data(), loaded.width_, loaded.height_,
                       GDT_Float64, 0, 0) != CE_None) {
        throw std::runtime_error("RasterIO failed for raster file: " + filename);
    }

    int hasNodata = 0;
    loaded.nodata_ = band->GetNoDataValue(&hasNodata);
    loaded.hasNodata_ = hasNodata != 0;
    loaded.bandScale_ = band->GetScale();
    loaded.bandOffset_ = band->GetOffset();

    loaded.hasSrs_ = datasetSRS(ds.get(), loaded.srs_);
    if (loaded.hasSrs_) {
        loaded.srs_.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    } else {
        std::cerr << "[GeoTIFFLoader] " << filename
                  << " has no coordinate system; reading it as longitude/latitude on the body\n";
    }
    *this = std::move(loaded);
}

}  // namespace planet
}  // namespace chrono
