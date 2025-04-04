#ifndef QUADTREE_TILE_H
#define QUADTREE_TILE_H

#include "Quadtree.h"
#include <cmath>      // For std::sqrt, std::floor
#include <unordered_map>
#include <vector>
#include <iostream>
#include "Perlin.h"
#include "GeoTIFFLoader.h"  // Include the GeoTIFF loader
#include <unordered_set>

//----------------------------------------------------------
// Mesh structure and QuadtreeTile declaration
//----------------------------------------------------------

struct Mesh {
    std::vector<float> vertices;  // position data, 3 floats per vertex
    std::vector<float> normals;   // normal data, 3 floats per vertex
    std::vector<float> texCoords; // texture coords, 2 floats per vertex
    std::vector<float> coarseNormals; // texture coords, 2 floats per vertex
    std::vector<unsigned int> indices;
};

struct vec3 {

    vec3(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    float x = 0;
    float y = 0;
    float z = 0;
};

struct TileMetadata {
    int ticksSinceSplit = 0; // -1 if expired/never happened
    int ticksSinceMerge = 0; // -1 if expired/never happened
    std::vector<vec3> dirtyVertices; // Deformations at highest LOD level
    bool dirtyVerticesTransferred = false;
};

class QuadtreeTile {
public:
    /**
     * Constructor.
     *
     * @param x Center x coordinate of the tile.
     * @param y Center y coordinate of the tile.
     * @param width Half-width of the tile.
     * @param height Half-height of the tile.
     * @param geoLoader Optional pointer to a GeoTIFFLoader.
     *
     * If geoLoader is provided, the elevation values will be taken from the GeoTIFF.
     * Otherwise, Perlin noise is used as a fallback.
     */
    QuadtreeTile(float x, float y, float width, float height, GeoTIFFLoader* geoLoader = nullptr);
    
    // Destructor: cleans up the allocated QuadTree.
    ~QuadtreeTile();

    void updateLOD(float cameraX, float cameraY, float cameraZ,
                   float splitThreshold, float mergeThreshold, int& subdivisions);

    void tick();
    void tickLeaves(QuadTree<TileMetadata>* node);

    void deformVertex(float x, float y, float dz);

    QuadTree<TileMetadata>* getTree() const;
    std::unordered_map<QuadTree<TileMetadata>*, Mesh> getMeshes();
    size_t getMemoryUsage() const;

    /**
     * Retrieves the elevation at a given (x, y) coordinate.
     *
     * If a GeoTIFFLoader is provided, it converts world coordinates (x, y) to pixel
     * coordinates using the geotransform and returns the elevation from the raster data.
     * Otherwise, it falls back to generating elevation using Perlin noise.
     */
    float getElevation(float x, float y);
    float computeBaseElevation(float x, float y);

    //std::unordered_set<QuadTree<TileMetadata>*> getDirtyTiles() { return dirtyTiles; }

private:
    // Recursive function to update level-of-detail.
    void updateLODRec(QuadTree<TileMetadata>* node,
                      float cameraX, float cameraY, float cameraZ,
                      float splitThreshold, float mergeThreshold,
                      int& subdivisions);

    // Called when a new bucket (node) is created.
    void onNewBucket(QuadTree<TileMetadata>* node);
    void onSplit(QuadTree<TileMetadata>* parent);
    void onMerge(QuadTree<TileMetadata>* node);

    // Called when a bucket (node) is unloaded.
    void onUnloadBucket(QuadTree<TileMetadata>* node);

    void updateMesh(QuadTree<TileMetadata>* node);


    QuadTree<TileMetadata>* findLeafNode(QuadTree<TileMetadata>* node, float x, float y);

    /**
     * Generates a triangular mesh for the tile.
     *
     * The elevation for each vertex is computed using either the GeoTIFF data
     * (if available) or Perlin noise.
     */
    Mesh generateTriangularMesh(float centerX, float centerY, float halfWidth, float halfHeight, int level);

    // Cross product helper.
    static inline void cross(const float* a, const float* b, float* result);

    // Normalize helper.
    static inline void normalize(float* v);

    void calculateNormals(Mesh& mesh);

    // Pointer to the underlying QuadTree.

    QuadTree<TileMetadata>* tree;
    // Mapping from a quadtree node to its mesh.
    std::unordered_map<QuadTree<TileMetadata>*, Mesh> bucketMeshes;
    // Optional pointer to a GeoTIFFLoader for elevation data.
    GeoTIFFLoader* geoTIFFLoader;

    std::unordered_set<QuadTree<TileMetadata>*> dirtyTiles;
};

#endif // QUADTREE_TILE_H
