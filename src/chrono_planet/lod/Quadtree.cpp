#include "chrono_planet/lod/Quadtree.h"

#include "chrono_planet/lod/SphericalCoordinates.h"
#include "chrono_planet/lod/CartesianCoordinates.h"
#include "chrono_planet/lod/TileMetadata.h"

namespace chrono {
namespace planet {

template <typename T, typename CoordSystem>
QuadTree<T, CoordSystem>::QuadTree(Boundary boundary, int level, QuadTree* parent)
    : boundary_(boundary), level_(level), parent_(parent) {
    if (parent) {
        onInit = parent->onInit;
        onDestroy = parent->onDestroy;
        onSplit = parent->onSplit;
        onMerge = parent->onMerge;
    }
    if (onInit) {
        onInit(this);
    }
}

template <typename T, typename CoordSystem>
QuadTree<T, CoordSystem>::~QuadTree() {
    if (onDestroy) {
        onDestroy(this);
    }
}

template <typename T, typename CoordSystem>
void QuadTree<T, CoordSystem>::subdivide() {
    if (divided_) {
        return;
    }
    const auto bounds = Traits::getChildBounds(boundary_);
    const int count = Traits::childCount(boundary_);
    for (int k = 0; k < count; ++k) {
        children_[k] = std::make_unique<QuadTree>(bounds[k], level_ + 1, this);
    }
    divided_ = true;
    if (onSplit) {
        onSplit(this);
    }
}

template <typename T, typename CoordSystem>
void QuadTree<T, CoordSystem>::merge() {
    if (!divided_) {
        return;
    }
    if (onMerge) {
        onMerge(this);
    }
    for (std::unique_ptr<QuadTree>& child : children_) {
        child.reset();
    }
    divided_ = false;
}

template class QuadTree<TileMetadata, Spherical>;

template class QuadTree<TileMetadata, Cartesian>;

}  // namespace planet
}  // namespace chrono
