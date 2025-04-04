#include "QuadtreeWorld.h"
#include <algorithm>

//---------------------------------------------------------------------
// Constructor
//---------------------------------------------------------------------
QuadtreeWorld::QuadtreeWorld(float tileSize, int viewRangeInTiles, float splitThreshold, float mergeThreshold)
    : tileSize(tileSize),
      viewRangeInTiles(viewRangeInTiles),
      splitThreshold(splitThreshold),
      mergeThreshold(mergeThreshold) {
    // Load the terrain data.
    loader.load("../resources/apollo.TIFF");
}

//---------------------------------------------------------------------
// Destructor: Cleans up all allocated tiles.
//---------------------------------------------------------------------
QuadtreeWorld::~QuadtreeWorld() {
    for (auto& pair : tiles) {
        delete pair.second;
    }
}

//---------------------------------------------------------------------
// update
//
// Updates the world based on the camera position. Creates new tiles
// if needed, removes tiles that are out of view, and updates the LOD
// for each active tile.
//---------------------------------------------------------------------
#include <cmath> // For std::fabs and std::ceil

void QuadtreeWorld::update(float cameraX, float cameraY, float cameraZ) {
    // *** Update view range based on the camera's height ***
    // Define a base height that corresponds to the initial view range.
    // The absolute value of cameraZ is used so that negative heights are handled correctly.
    const float baseHeight = -1686.0f; // Adjust this as needed.
    
    // Optional: clamp the view range to sensible minimum and maximum values.
    const int minViewRange = 10;
    const int maxViewRange = 30;
    
    // Compute a dynamic view range using the absolute height of the camera.
    // If the camera's height (absolute value) equals baseHeight, then the dynamic view range equals viewRangeInTiles.
    // For greater heights, the view range increases proportionally.
    float dZ = cameraZ - baseHeight;
    int dynamicViewRange = 15;
    // dynamicViewRange = std::max(minViewRange, std::min(maxViewRange, dynamicViewRange));

    // Determine the grid index of the tile that contains the camera.
    int centerTileX = static_cast<int>(std::floor(cameraX / tileSize));
    int centerTileY = static_cast<int>(std::floor(cameraY / tileSize));

    // Create a temporary map representing the tiles that should be active.
    std::unordered_map<TileKey, bool> neededTiles;

    // Loop over a square region around the camera using the dynamic view range.
    for (int dy = -dynamicViewRange; dy <= dynamicViewRange; ++dy) {
        for (int dx = -dynamicViewRange; dx <= dynamicViewRange; ++dx) {
            int tileX = centerTileX + dx;
            int tileY = centerTileY + dy;
            TileKey key{ tileX, tileY };
            neededTiles[key] = true;

            // If this tile does not exist yet, create and initialize it.
            if (tiles.find(key) == tiles.end()) {
                // Compute the center position of the tile.
                float centerPosX = tileX * tileSize + tileSize * 0.5f;
                float centerPosY = tileY * tileSize + tileSize * 0.5f;
                float halfSize = tileSize * 0.5f;
                tiles[key] = new QuadtreeTile(centerPosX, centerPosY, halfSize, halfSize, &loader);
            }
        }
    }

    // Remove tiles that are no longer within the view range.
    for (auto it = tiles.begin(); it != tiles.end(); ) {
        if (neededTiles.find(it->first) == neededTiles.end()) {
            delete it->second;
            it = tiles.erase(it);
            // dirtyTiles.erase(it->second);
        } else {
            ++it;
        }
    }
    // Update each active tile's level-of-detail based on the camera's position.
    for (auto& pair : tiles) {
        int subdivisions = 0;
        pair.second->updateLOD(cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold, subdivisions);
    }

    // Update ticks for each tile.
    for (auto& pair : tiles) {
        pair.second->tick();
    }
}


//---------------------------------------------------------------------
// getTotalTiles
//
// Returns the total number of active tiles.
//---------------------------------------------------------------------
int QuadtreeWorld::getTotalTiles() const {
    return static_cast<int>(tiles.size());
}

//---------------------------------------------------------------------
// getMemoryUsage
//
// Aggregates the memory usage of all active tiles in bytes.
//---------------------------------------------------------------------
size_t QuadtreeWorld::getMemoryUsage() const {
    size_t totalMemory = 0;
    for (const auto& pair : tiles) {
        totalMemory += pair.second->getMemoryUsage();
    }
    totalMemory += loader.getMemoryUsage();
    return totalMemory;
}

//---------------------------------------------------------------------
// getAllMeshes
//
// Aggregates and returns all the meshes from each tile's buckets.
//---------------------------------------------------------------------
std::unordered_map<QuadTree<TileMetadata>*, Mesh> QuadtreeWorld::getAllMeshes() {
    std::unordered_map<QuadTree<TileMetadata>*, Mesh> allMeshes;
    for (auto& pair : tiles) {
        auto tileMeshes = pair.second->getMeshes();
        allMeshes.insert(tileMeshes.begin(), tileMeshes.end());
    }
    return allMeshes;
}

void QuadtreeWorld::deformVertex(float x, float y, float dz) {
    // Compute the grid indices for the tile that should cover (x,y).
    int tileX = static_cast<int>(std::floor(x / tileSize));
    int tileY = static_cast<int>(std::floor(y / tileSize));
    TileKey key{ tileX, tileY };

    auto it = tiles.find(key);
    if (it != tiles.end()) {

        it->second->deformVertex(x, y, dz);
        // auto _it = std::find(dirtyTiles.begin(),dirtyTiles.end(), it->second);
        // dirtyTiles.insert(it->second);

        // std::cout << "Deforming" << std::endl;
    } else {

        return;
    }
}

float QuadtreeWorld::getElevation(float x, float y) {
    // Compute the grid indices for the tile that should cover (x,y).
    int tileX = static_cast<int>(std::floor(x / tileSize));
    int tileY = static_cast<int>(std::floor(y / tileSize));
    TileKey key{ tileX, tileY };
    
    // Look up the tile in the active tiles map.
    auto it = tiles.find(key);
    if (it != tiles.end()) {
        // Forward the deformation to the tile.
        return it->second->getElevation(x,y);
        
    } else {
        return -1;
    }
}