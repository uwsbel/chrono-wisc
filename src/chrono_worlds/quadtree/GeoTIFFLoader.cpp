#include "GeoTIFFLoader.h"
#include "gdal_priv.h"
#include "cpl_conv.h"  // for CPLMalloc
#include <iostream>

GeoTIFFLoader::GeoTIFFLoader() : width(0), height(0) {
    // Register all GDAL drivers.
    GDALAllRegister();
}

GeoTIFFLoader::~GeoTIFFLoader() {
    // GDAL cleanup, if necessary, can be done here.
}

void GeoTIFFLoader::load(const std::string& filename) {
    // Open the GeoTIFF file in read-only mode.
    GDALDataset *poDataset = static_cast<GDALDataset*>(GDALOpen(filename.c_str(), GA_ReadOnly));
    if (poDataset == nullptr) {
        throw std::runtime_error("Failed to open GeoTIFF file: " + filename);
    }
    
    // Retrieve the raster dimensions.
    width  = poDataset->GetRasterXSize();
    height = poDataset->GetRasterYSize();

    // Retrieve the geotransform parameters.
    geoTransform.resize(6);
    if (poDataset->GetGeoTransform(geoTransform.data()) != CE_None) {
        GDALClose(poDataset);
        throw std::runtime_error("Failed to get geotransform from GeoTIFF file: " + filename);
    }

    // For this example, we assume that the elevation data is in band 1.
    GDALRasterBand* poBand = poDataset->GetRasterBand(1);
    if (poBand == nullptr) {
        GDALClose(poDataset);
        throw std::runtime_error("Failed to get raster band from GeoTIFF file: " + filename);
    }
    
    // Resize the elevation data vector to hold all pixels.
    elevationData.resize(width * height);
    memoryUsage = static_cast<int>(elevationData.size() * sizeof(float) +
                                   geoTransform.size() * sizeof(double));
    // Read the raster data into the elevationData vector.
    // We assume the data type is Float32; adjust GDT_Float32 if your data type is different.
    CPLErr err = poBand->RasterIO(GF_Read,
                                  0, 0, width, height,
                                  elevationData.data(),
                                  width, height,
                                  GDT_Float32,
                                  0, 0);
    if (err != CE_None) {
        GDALClose(poDataset);
        throw std::runtime_error("RasterIO failed for GeoTIFF file: " + filename);
    }
    
    
    // Close the dataset.
    GDALClose(poDataset);
}

int GeoTIFFLoader::getWidth() const {
    return width;
}

int GeoTIFFLoader::getHeight() const {
    return height;
}

int GeoTIFFLoader::getMemoryUsage() const {
    return memoryUsage;
}

const std::vector<float>& GeoTIFFLoader::getElevationData() const {
    return elevationData;
}

const std::vector<double>& GeoTIFFLoader::getGeoTransform() const {
    return geoTransform;
}
