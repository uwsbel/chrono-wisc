#ifndef CH_PLANET_TILEMETADATA_H
#define CH_PLANET_TILEMETADATA_H

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/lod/ChTileMesh.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace chrono {
namespace planet {

// Identifies a root tile by its grid indices.
struct CH_PLANET_API TileKey {
    int x;
    int y;
    bool operator==(const TileKey& other) const { return x == other.x && y == other.y; }
};


// Per-node LOD state, stored as the payload of each QuadTree node.
struct CH_PLANET_API TileMetadata {
    // Elevation range of the node's mesh (m above the sphere), for the LOD distance test.
    double maxElevation = 0.0;
    double minElevation = 0.0;
    bool morphFromParent = false;              // new leaves start their renderer-owned split animation
};

}  // namespace planet
}  // namespace chrono

namespace std {
// Hash for TileKey, so it can key the quadtree's root tile map.
template <>
struct hash<::chrono::planet::TileKey> {
    std::size_t operator()(const ::chrono::planet::TileKey& key) const {
        return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1);
    }
};
}  // namespace std

#endif   // CH_PLANET_TILEMETADATA_H
