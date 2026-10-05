#include "chrono_planet/lod/CartesianCoordinates.h"
#include "chrono_planet/ChPlanetSurface.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace chrono {
namespace planet {

std::array<Cartesian::Boundary, 4> CoordinateTraits<Cartesian>::getChildBounds(const Boundary& b) {
    const double hw = b.halfWidthM * 0.5, hh = b.halfHeightM * 0.5;
    return {{{b.centerX + hw, b.centerY + hh, hw, hh, b.site},
             {b.centerX - hw, b.centerY + hh, hw, hh, b.site},
             {b.centerX - hw, b.centerY - hh, hw, hh, b.site},
             {b.centerX + hw, b.centerY - hh, hw, hh, b.site}}};
}

double CoordinateTraits<Cartesian>::distanceToBounds(const Boundary& b, const util::Vec3& camera,
                                                     double minHeight, double maxHeight, double) {
    const util::Vec3 nearest{std::clamp(camera.x, b.centerX - b.halfWidthM, b.centerX + b.halfWidthM),
                             std::clamp(camera.y, b.centerY - b.halfHeightM, b.centerY + b.halfHeightM),
                             std::clamp(camera.z, std::min(minHeight, maxHeight), std::max(minHeight, maxHeight))};
    return util::length(camera - nearest);
}

std::vector<double> CoordinateTraits<Cartesian>::cartesianGrid(const Boundary& b, int divisions,
    const ChPlanetSurface& surface, int zoom, double spacing, std::vector<double>* staticHeights) {
    if (divisions <= 0 || !(b.halfWidthM > 0) || !(b.halfHeightM > 0))
        throw std::invalid_argument("Cartesian grid requires positive dimensions");
    if (b.site.GetRadius() != surface.GetBody().GetRadius())
        throw std::invalid_argument("Cartesian site and surface must use the same body radius");
    const int side = divisions + 1;
    const double stepX = 2 * b.halfWidthM / divisions, stepY = 2 * b.halfHeightM / divisions;
    const size_t count = static_cast<size_t>(side) * side;
    const bool cached = staticHeights && staticHeights->size() == count;
    std::vector<double> newStaticHeights;
    if (staticHeights && !cached)
        newStaticHeights.resize(count);
    std::vector<double> positions(count * 3);
    // A Cartesian row has constant latitude, but longitude spacing changes between rows.
    // One-point geographic grids reuse the surface's static/dynamic filter policy exactly.
    for (int row = 0; row < side; ++row) {
        const double y = b.centerY - b.halfHeightM + row * stepY;
        for (int column = 0; column < side; ++column) {
            const double x = b.centerX - b.halfWidthM + column * stepX;
            double lon, lat;
            b.site.ToLonLat(x, y, lon, lat);
            if (!std::isfinite(lon) || std::abs(lat) >= 89.9)
                throw std::invalid_argument("Cartesian site grid must stay outside the polar caps");
            const double reliefSpacing = spacing > 0 ? spacing : stepX / util::metresPerDegLon(b.site.GetRadius(), lat);
            const ChGeoGrid point{lon, lat, reliefSpacing, reliefSpacing, 1, reliefSpacing};
            const size_t index = static_cast<size_t>(row) * side + column;
            double height;
            if (staticHeights) {
                std::vector<double> sample;
                if (cached)
                    sample = {(*staticHeights)[index]};
                else {
                    surface.GetStaticElevationGrid(point, zoom, sample);
                    newStaticHeights[index] = sample[0];
                }
                surface.ApplyDynamicFilters(point, sample);
                height = sample[0];
            } else {
                height = surface.GetElevationAtZoom(lon, lat, zoom, reliefSpacing);
            }
            positions[index * 3] = x;
            positions[index * 3 + 1] = y;
            positions[index * 3 + 2] = height - b.site.GetOriginElevation();
        }
    }
    if (staticHeights && !cached)
        *staticHeights = std::move(newStaticHeights);
    return positions;
}

}  // namespace planet
}  // namespace chrono
