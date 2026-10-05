#ifndef CH_PLANET_ROCKMESHES_H
#define CH_PLANET_ROCKMESHES_H

#include "chrono_planet/ChApiPlanet.h"

#include <array>
#include <cstdint>
#include <vector>

namespace chrono {
namespace planet {

// One rock mesh, scaled so its longest semi-axis is 1.
struct CH_PLANET_API RockMesh {
    static constexpr int kLods = 3;
    static constexpr int kVertexStride = 6;   // xyz position followed by xyz normal

    std::vector<float> vertices;          // interleaved xyz position, xyz normal
    std::vector<std::uint32_t> indices;   // absolute into vertices

    struct Lod {
        std::uint32_t first = 0, count = 0;   // range in indices
    };
    std::array<Lod, kLods> lod;   // 0 = finest
    std::vector<float> support;   // LOD-1 vertex positions, for extentsZ
};

// The library of kCount procedural rocks, built once on first use.
class CH_PLANET_API RockMeshes {
public:
    static constexpr int kCount = 12;
    // Mesh for an id. Ids wrap modulo kCount.
    [[nodiscard]] static const RockMesh& get(std::uint16_t meshId);
    struct VerticalExtents {
        double bottomExtent;   // reach below the origin, in unit-rock coordinates
        double totalHeight;    // bottom-to-top height in the same units
    };
    // Extents after rotation by a unit quaternion (x, y, z, w).
    [[nodiscard]] static VerticalExtents extentsZ(std::uint16_t meshId, const std::array<double, 4>& orientationXyzw);
};

}  // namespace planet
}  // namespace chrono

#endif   // CH_PLANET_ROCKMESHES_H
