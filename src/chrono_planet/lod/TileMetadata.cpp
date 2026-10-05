#include "chrono_planet/lod/TileMetadata.h"

#include <atomic>

namespace chrono {
namespace planet {

std::uint64_t ChTileMesh::nextId() {
    static std::atomic<std::uint64_t> n{1};
    return n++;
}

}  // namespace planet
}  // namespace chrono
