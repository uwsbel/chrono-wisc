#ifndef CH_PLANET_ROCKSDF_H
#define CH_PLANET_ROCKSDF_H

#include "chrono_planet/ChApiPlanet.h"

#include <cstdint>
#include <vector>

namespace chrono {
namespace planet {

// Signed distance field of one RockMeshes rock in its unit frame, sphere-traced for rock cast shadows.
// kN^3 samples over [-kExtent, kExtent]^3, x fastest, in rock units, negative inside.
struct CH_PLANET_API RockSdfGrid {
    static constexpr int kN = 48;
    static constexpr float kExtent = 1.15f;   // pads the unit sphere so edge fetches never clamp
    std::vector<float> d;                     // kN * kN * kN
};

// Baked signed distance grids for every mesh in RockMeshes.
class CH_PLANET_API RockSdf {
public:
    // Grid for a mesh id. The whole library is baked on first use.
    [[nodiscard]] static const RockSdfGrid& get(std::uint16_t meshId);
};

}  // namespace planet
}  // namespace chrono

#endif   // CH_PLANET_ROCKSDF_H
