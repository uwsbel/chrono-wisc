#ifndef CH_PLANET_CARTESIAN_COORDINATES_H
#define CH_PLANET_CARTESIAN_COORDINATES_H

#include "chrono_planet/lod/CoordinateSystems.h"
#include "chrono_planet/core/SphereMath.h"
#include "chrono_planet/lod/TileMetadata.h"
#include <array>
#include <vector>

namespace chrono {
namespace planet {
class ChPlanetSurface;

// Local east/north/up meters. The site frame travels with the bounds, including worker jobs.
template <>
class CoordinateTraits<Cartesian> {
  public:
    using Boundary = Cartesian::Boundary;
    using Position = Cartesian::Position;
    static int childCount(const Boundary&) { return 4; }
    static std::array<Boundary, 4> getChildBounds(const Boundary& bounds);
    static double distanceToBounds(const Boundary& bounds, const util::Vec3& camera, double minHeight, double maxHeight, double);
    static std::vector<double> cartesianGrid(const Boundary& bounds, int divisions, const ChPlanetSurface& surface,
                                           int zoom, double spacing = 0, std::vector<double>* staticHeights = nullptr);
    static double elevationOf(double, double, double z, double) { return z; }
    static util::Vec3 skirtVertex(const util::Vec3& point, double depth) { return {point.x, point.y, point.z - depth}; }
    static util::EnuFrame frameAt(const util::Vec3&) { return {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; }
    static Boundary expanded(const Boundary& bounds, double factor) {
        auto result = bounds;
        result.halfWidthM *= factor;
        result.halfHeightM *= factor;
        return result;
    }
    static void textureBounds(const Boundary&, double* origin, double* size) {
        origin[0] = origin[1] = 0;
        size[0] = size[1] = 1;
    }
};

}  // namespace planet
}  // namespace chrono
#endif
