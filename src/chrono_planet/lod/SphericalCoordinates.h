#ifndef CH_PLANET_COORDINATE_TRAITS_SPHERICAL_H
#define CH_PLANET_COORDINATE_TRAITS_SPHERICAL_H

#include "chrono_planet/ChApiPlanet.h"

#include <algorithm>
#include <array>
#include <memory>
#include <utility>
#include <vector>

#include "chrono_planet/lod/CoordinateSystems.h"
#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/core/SphereMath.h"
#include "chrono_planet/lod/TileMetadata.h"

namespace chrono {
namespace planet {

class ChPlanetSurface;

// Traits for the lon/lat sphere, covering tile bounds, relief sampling and cartesian grids.
template <>
class CoordinateTraits<Spherical> {
public:
    using Boundary = Spherical::Boundary;
    using Position = Spherical::Position;

    static util::EnuFrame frameAt(const util::Vec3& point) { return util::enuAlong(point); }
    static Boundary expanded(const Boundary& bounds, double factor) {
        auto result = bounds;
        result.halfWidthDeg *= factor;
        result.halfHeightDeg *= factor;
        return result;
    }
    static void textureBounds(const Boundary& b, double* origin, double* size) {
        origin[0] = (b.centerLonDeg - b.halfWidthDeg + 180.0) / 360.0;
        origin[1] = (b.centerLatDeg - b.halfHeightDeg + 90.0) / 180.0;
        size[0] = 2.0 * b.halfWidthDeg / 360.0;
        size[1] = 2.0 * b.halfHeightDeg / 180.0;
    }

    // How many tiles a tile splits into. Four, as a quadtree's, where it is about as wide as tall on the ground.
    // Toward a pole a tile of equal degrees each way grows thin, its width shrinking as the cosine of its latitude:
    // there it splits in two, north and south, keeping its width, until its children are about square again. So a
    // tile's cells stay within a factor of 1.4 of square at any latitude, where a plain quadtree's are 12 times taller
    // than wide at 85 degrees. A tile's level still counts the halvings of its height.
    static int childCount(const Boundary& b);
    // NE, NW, SW, SE; or north, south where childCount is two (the last two are then unused).
    static std::array<Boundary, 4> getChildBounds(const Boundary& b);

    // Chord distance from a camera position (meters) to the nearest point of the tile's elevation shell
    // on a sphere of the given radius.
    [[nodiscard]] static double distanceToBounds(const Boundary& b, const util::Vec3& cameraM,
                                                 double minElev, double maxElev, double sphereRadiusM);

    // Heights in meters above the reference sphere, using the same grid and cache as cartesianGrid.
    // reliefSpacingDeg <= 0 uses the grid's longitude spacing for relief filtering.
    [[nodiscard]] static std::vector<double> elevationGrid(const Boundary& b, int divisions, const ChPlanetSurface& surface,
                                                          int zoomLevel, double reliefSpacingDeg = 0.0,
                                                          std::vector<double>* staticHeights = nullptr);

    // Cartesian positions of a (divisions+1)^2 grid over b, row-major south to north, on the surface at zoomLevel.
    // reliefSpacingDeg <= 0 filters the relief to the grid's own spacing.
    // staticHeights, if given, caches the surface's static stage: an empty vector is filled with it, a filled
    // one is used instead of recomputing it, so only the dynamic filters run again.
    [[nodiscard]] static std::vector<double> cartesianGrid(const Boundary& b, int divisions, const ChPlanetSurface& surface,
                                                           int zoomLevel, double reliefSpacingDeg = 0.0,
                                                           std::vector<double>* staticHeights = nullptr);

    // Root tile grid indices (TileKey x, y) of the tile containing pos.
    [[nodiscard]] static TileKey computeTileIndices(const Position& pos, double tileSizeDegrees);
    // (lon, lat) of a root tile's center, degrees.
    [[nodiscard]] static util::LonLat tileCenterPosition(const TileKey& key, double tileSizeDegrees);

    static double wrapLongitude(double lon) { return util::wrapLongitude(lon); }
    // Height of a cartesian point above a sphere of the given radius, meters.
    static double elevationOf(double x, double y, double z, double sphereRadiusM) {
        return util::elevationOf(x, y, z, sphereRadiusM);
    }

    // Lowers a border vertex by depth meters toward the planet center.
    static util::Vec3 skirtVertex(const util::Vec3& p, double depth) {
        const double r = util::length(p);
        if (r < 1e-6) {
            return p;
        }
        const double s = (r - depth) / r;
        return {p.x * s, p.y * s, p.z * s};
    }

    // Clips a boundary to +-90 latitude and wraps its center longitude.
    static Boundary clampAtPoles(Boundary b) {
        b.centerLatDeg = std::clamp(b.centerLatDeg, -90.0, 90.0);
        if (b.centerLatDeg + b.halfHeightDeg > 90.0) {
            b.halfHeightDeg = 90.0 - b.centerLatDeg;
        }
        if (b.centerLatDeg - b.halfHeightDeg < -90.0) {
            b.halfHeightDeg = b.centerLatDeg + 90.0;
        }
        b.centerLonDeg = wrapLongitude(b.centerLonDeg);
        return b;
    }
};

}  // namespace planet
}  // namespace chrono

#endif   // CH_PLANET_COORDINATE_TRAITS_SPHERICAL_H
