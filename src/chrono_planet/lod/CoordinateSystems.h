#ifndef CH_PLANET_COORDINATESYSTEMS_H
#define CH_PLANET_COORDINATESYSTEMS_H

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChSiteFrame.h"

namespace chrono {
namespace planet {

// The lon/lat coordinate system, with position and tile-bounds types in degrees.
struct CH_PLANET_API Spherical {
    // A point on the sphere, in degrees.
    struct Position {
        double lon, lat;
    };

    // Geographic center and half extents, all in degrees.
    struct Boundary {
        double centerLonDeg, centerLatDeg, halfWidthDeg, halfHeightDeg;
    };
};

// Local Cartesian coordinates in meters, sampling the planet through a site frame.
struct CH_PLANET_API Cartesian {
    struct Position { double x, y; };
    struct Boundary {
        double centerX, centerY, halfWidthM, halfHeightM;
        ChSiteFrame site{1.0, 0.0, 0.0, 0.0};  // supply the surface's site before sampling
    };
};

// Geometry operations for one coordinate system, specialized in that system's own header.
template <typename CoordSystem>
struct CoordinateTraits;

}  // namespace planet
}  // namespace chrono

#endif   // CH_PLANET_COORDINATESYSTEMS_H
