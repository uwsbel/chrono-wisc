#ifndef GEOTIFF_LOADER_H
#define GEOTIFF_LOADER_H

#include <string>
#include <vector>
#include <stdexcept>

// A simple class for loading elevation data from a GeoTIFF file.
class GeoTIFFLoader {
public:
    GeoTIFFLoader();
    ~GeoTIFFLoader();

    // Loads the GeoTIFF file. Throws std::runtime_error on failure.
    void load(const std::string& filename);

    // Returns the number of columns in the raster.
    int getWidth() const;

    // Returns the number of rows in the raster.
    int getHeight() const;

    // Returns the elevation data as a flat vector in row-major order.
    // Each element corresponds to the elevation value at that pixel.
    const std::vector<float>& getElevationData() const;

    // Returns the geotransform parameters as a vector of 6 doubles.
    // The geotransform array is defined as:
    // [ originX, pixelWidth, rotationX, originY, rotationY, pixelHeight ]
    const std::vector<double>& getGeoTransform() const;

    int getMemoryUsage() const;

private:
    int width;
    int height;
    std::vector<float> elevationData;
    std::vector<double> geoTransform; // 6 parameters
    int memoryUsage;
};

#endif // GEOTIFF_LOADER_H
