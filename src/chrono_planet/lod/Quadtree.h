#ifndef CH_PLANET_QUADTREE_H
#define CH_PLANET_QUADTREE_H

#include "chrono_planet/ChApiPlanet.h"

#include <array>
#include <functional>
#include <memory>
#include "chrono_planet/lod/CoordinateSystems.h"

namespace chrono {
namespace planet {

// A node of the LOD tree that owns its four children. Callbacks let the owner attach and release per-node data.
// Instantiated for TileMetadata with Spherical and Cartesian coordinates.
template <typename T, typename CoordSystem>
class QuadTree {
public:
    using Boundary = typename CoordSystem::Boundary;
    using Traits = CoordinateTraits<CoordSystem>;
    using Callback = std::function<void(QuadTree*)>;

    Callback onInit;      // node constructed (runs before its children exist)
    Callback onDestroy;   // node about to be destroyed
    Callback onSplit;     // children just created
    Callback onMerge;     // children about to be deleted

    // A child inherits its parent's callbacks. onInit fires at the end of construction.
    explicit QuadTree(Boundary boundary, int level = 0, QuadTree* parent = nullptr);
    // Fires onDestroy, then releases the children.
    ~QuadTree();
    QuadTree(const QuadTree&) = delete;
    QuadTree& operator=(const QuadTree&) = delete;

    Boundary getBoundary() const { return boundary_; }
    QuadTree* getParent() { return parent_; }
    const QuadTree* getParent() const { return parent_; }
    int getLevel() const { return level_; }
    bool isDivided() const { return divided_; }
    // The per-node payload the owner keeps on this node.
    const T* getType() const { return &data_; }
    T* getType() { return &data_; }

    // Borrowed child nodes in Traits::getChildBounds order, NE, NW, SW, SE. All null unless divided; the last two
    // null where the node split in two (Traits::childCount).
    std::array<QuadTree*, 4> children() {
        return {children_[0].get(), children_[1].get(), children_[2].get(), children_[3].get()};
    }
    std::array<const QuadTree*, 4> children() const {
        return {children_[0].get(), children_[1].get(), children_[2].get(), children_[3].get()};
    }

    // Creates the children, four or two (no-op if already divided), then fires onSplit.
    void subdivide();
    // Fires onMerge, then deletes the children (no-op if not divided).
    void merge();

private:
    Boundary boundary_;
    T data_{};
    int level_;
    QuadTree* parent_;   // borrowed, outlives this node
    bool divided_ = false;
    std::array<std::unique_ptr<QuadTree>, 4> children_;
};

}  // namespace planet
}  // namespace chrono

#endif   // CH_PLANET_QUADTREE_H
