#include "QuadtreeTile.h"
#include <cmath>
#include <cstdlib>
#include <algorithm> 

// ------------------------------
// Constructor & Destructor
// ------------------------------
QuadtreeTile::QuadtreeTile(float x, float y, float width, float height, GeoTIFFLoader* geoLoader)
    : geoTIFFLoader(geoLoader)
{
    // Create the quadtree (assumes QuadTree is now non-templated).
    tree = new QuadTree<TileMetadata>(x, y, width, height);
    
    // Set up callbacks.
    tree->nodeInitCallback = [this](QuadTree<TileMetadata>* node) {
        this->onNewBucket(node);
    };
    tree->nodeDestroyCallback = [this](QuadTree<TileMetadata>* node) {
        this->onUnloadBucket(node);
    };
    tree->nodeSplitCallback = [this](QuadTree<TileMetadata>* parent) {
        this->onUnloadBucket(parent);
        this->onSplit(parent);
    };
    tree->nodeMergeCallback = [this](QuadTree<TileMetadata>* node) {
        this->onNewBucket(node);
        this->onMerge(node);
    };
}

QuadtreeTile::~QuadtreeTile() {
    delete tree;
}

// ------------------------------
// Public Methods
// ------------------------------
void QuadtreeTile::updateLOD(float cameraX, float cameraY, float cameraZ,
                             float splitThreshold, float mergeThreshold, int& subdivisions)
{
    updateLODRec(tree, cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold, subdivisions);
}

QuadTree<TileMetadata>* QuadtreeTile::getTree() const {
    return tree;
}

std::unordered_map<QuadTree<TileMetadata>*, Mesh> QuadtreeTile::getMeshes() {
    return bucketMeshes;
}

void QuadtreeTile::tick() {
    tickLeaves(getTree());
}

void QuadtreeTile::tickLeaves(QuadTree<TileMetadata>* node) {
    if (!node) return;  

    if (node->isDivided()) {
        tickLeaves(node->getNortheastNonConst());
        tickLeaves(node->getNorthwestNonConst());
        tickLeaves(node->getSoutheastNonConst());
        tickLeaves(node->getSouthwestNonConst());
    } else {
        auto* data = node->getType();
        int expirationTicks = 100;

        // If a split is active, cancel the merge and update only the split counter.
        if (data->ticksSinceSplit != -1) {
            data->ticksSinceMerge = -1;  // Ensure merge is canceled.
            data->ticksSinceSplit++;
            if (data->ticksSinceSplit > expirationTicks)
                data->ticksSinceSplit = -1;
        }
        // Otherwise, if no split is active, update the merge counter if it is active.
        else if (data->ticksSinceMerge != -1) {
            data->ticksSinceMerge++;
            if (data->ticksSinceMerge > expirationTicks)
                data->ticksSinceMerge = -1;
        }
    }
}

size_t QuadtreeTile::getMemoryUsage() const {
    size_t totalMemory = 0;
    for (const auto& bucketMesh : bucketMeshes) {
        const Mesh& mesh = bucketMesh.second;
        totalMemory += mesh.vertices.capacity() * sizeof(float);
        totalMemory += mesh.indices.capacity() * sizeof(int);
        totalMemory += mesh.normals.capacity() * sizeof(float);
        totalMemory += mesh.texCoords.capacity() * sizeof(int);
        totalMemory += mesh.coarseNormals.capacity() * sizeof(int);
        
    }
    return totalMemory;
}

// ------------------------------
// Private Methods
// ------------------------------
void QuadtreeTile::updateLODRec(QuadTree<TileMetadata>* node,
                                float cameraX, float cameraY, float cameraZ,
                                float splitThreshold, float mergeThreshold,
                                int& subdivisions)
{
    // Retrieve the node's boundary.
    QuadTree<TileMetadata>::QuadBoundary boundary = node->getBoundary();

    float left   = boundary.x - boundary.width;
    float right  = boundary.x + boundary.width;
    float top    = boundary.y - boundary.height;
    float bottom = boundary.y + boundary.height;

    // Compute horizontal (dx) and vertical (dy) distances from the camera to the boundary.
    float dx = 0.0f;
    if (cameraX < left)
        dx = left - cameraX;
    else if (cameraX > right)
        dx = cameraX - right;

    float dy = 0.0f;
    if (cameraY < top)
        dy = top - cameraY;
    else if (cameraY > bottom)
        dy = cameraY - bottom;
    
    float dz = getElevation(boundary.x, boundary.y) - cameraZ;  // Assuming terrain is at z=0

    float distance = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));

    int level = node->getLevel();

    float effectiveSplitThreshold = splitThreshold / (level + 1);
    float effectiveMergeThreshold = mergeThreshold / (level + 1);

    if (level == 0) {
        effectiveSplitThreshold = 1200.f;
        effectiveMergeThreshold = 1200.f;

    }
    if (level == 1) {
        effectiveSplitThreshold = 20.f;
        effectiveMergeThreshold = 20.f;

    }
    if (level == 2) {
        effectiveSplitThreshold = 10.f;
        effectiveMergeThreshold = 10.f;
    }
    if (level == 3) {
        effectiveSplitThreshold = 2.f;
        effectiveMergeThreshold = 2.f;
    }

    if (level == 4) {
        effectiveSplitThreshold = 0.5f;
        effectiveMergeThreshold = 0.5f;
    }
    if (level == 5) {
        effectiveSplitThreshold = 0.1f;
        effectiveMergeThreshold = 0.1f;

    }


    if (distance < effectiveSplitThreshold && level < 5) {  
        if (!node->isDivided()) {
            node->subdivide();
            subdivisions++; 
        }
        if (node->isDivided()) {
            updateLODRec(node->getNortheastNonConst(), cameraX, cameraY, cameraZ,
                           splitThreshold, mergeThreshold, subdivisions);
            updateLODRec(node->getNorthwestNonConst(), cameraX, cameraY, cameraZ,
                           splitThreshold, mergeThreshold, subdivisions);
            updateLODRec(node->getSoutheastNonConst(), cameraX, cameraY, cameraZ,
                           splitThreshold, mergeThreshold, subdivisions);
            updateLODRec(node->getSouthwestNonConst(), cameraX, cameraY, cameraZ,
                           splitThreshold, mergeThreshold, subdivisions);
        }
    }
    else if (distance > effectiveMergeThreshold) {
        if (node->isDivided()) {
            node->merge();
        }
    }
}

void QuadtreeTile::onNewBucket(QuadTree<TileMetadata>* node) {
    updateMesh(node);
}


void QuadtreeTile::onSplit(QuadTree<TileMetadata>* parent) {
    if (!parent) return;
    
    TileMetadata* parentMeta = parent->getType();
    if (!parentMeta->dirtyVerticesTransferred) {
        // Transfer dirty vertices to children using half-open bounds:
        auto transferToChild = [parentMeta](QuadTree<TileMetadata>* child) {
            if (!child) return;
            TileMetadata* childMeta = child->getType();
            childMeta->dirtyVertices.clear();
            QuadTree<TileMetadata>::QuadBoundary bounds = child->getBoundary();
            float left   = bounds.x - bounds.width;
            float right  = bounds.x + bounds.width;
            float top    = bounds.y - bounds.height;
            float bottom = bounds.y + bounds.height;
            
            // Use a half-open interval: [left, right) and [top, bottom)
            for (const auto& dp : parentMeta->dirtyVertices) {
                if (dp.x >= left && dp.x < right &&
                    dp.y >= top  && dp.y < bottom) {
                    childMeta->dirtyVertices.push_back(dp);
                }
            }
        };

        // Transfer for each of the parent's four children.
        transferToChild(parent->getNortheastNonConst());
        transferToChild(parent->getNorthwestNonConst());
        transferToChild(parent->getSoutheastNonConst());
        transferToChild(parent->getSouthwestNonConst());

        // Mark that we've transferred the parent's dirty vertices.
        parentMeta->dirtyVerticesTransferred = true;
        // Clear parent's dirty vertices so they won't be transferred again.
        parentMeta->dirtyVertices.clear();
    }
 
    parent->getType()->ticksSinceSplit = 0; // Reset split timer.
}


// #include <cmath>      // for std::fabs
// #include <algorithm>  // for std::sort and std::unique

void QuadtreeTile::onMerge(QuadTree<TileMetadata>* node) {
    TileMetadata* parentMeta = node->getType();
    parentMeta->dirtyVertices.clear();

    // Merge each child's dirty vertices into the parent's list.
    auto mergeChildDirtyVertices = [parentMeta](QuadTree<TileMetadata>* child) {
        if (!child) return;
        TileMetadata* childMeta = child->getType();
        parentMeta->dirtyVertices.insert(
            parentMeta->dirtyVertices.end(),
            childMeta->dirtyVertices.begin(),
            childMeta->dirtyVertices.end()
        );
        childMeta->dirtyVertices.clear();
    };

    mergeChildDirtyVertices(node->getNortheastNonConst());
    mergeChildDirtyVertices(node->getNorthwestNonConst());
    mergeChildDirtyVertices(node->getSoutheastNonConst());
    mergeChildDirtyVertices(node->getSouthwestNonConst());

    // Deduplicate the merged dirty vertices.
    // Assumes that 'vec3' has members .x, .y, and .z.
    std::vector<vec3>& dv = parentMeta->dirtyVertices;
    std::sort(dv.begin(), dv.end(), [](const vec3& a, const vec3& b) {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    });
    dv.erase(std::unique(dv.begin(), dv.end(), [](const vec3& a, const vec3& b) {
        // Consider two vertices the same if they are nearly equal.
        const float epsilon = 1e-6f;
        return (std::fabs(a.x - b.x) < epsilon &&
                std::fabs(a.y - b.y) < epsilon &&
                std::fabs(a.z - b.z) < epsilon);
    }), dv.end());

    // Reset the merge tick counter.
    node->getType()->ticksSinceMerge = 0;
}


void QuadtreeTile::onUnloadBucket(QuadTree<TileMetadata>* node) {
    // Remove the mesh associated with this node (its children are being unloaded).
    auto it = bucketMeshes.find(node);
    if (it != bucketMeshes.end()) {
        bucketMeshes.erase(it);
    }

}


void QuadtreeTile::deformVertex(float x, float y, float dz)
{
    // Locate the leaf node covering (x,y).
    QuadTree<TileMetadata>* leaf = findLeafNode(tree, x, y);
    if (!leaf) {
        // (x,y) is outside the terrain bounds.
        return;
    }
    
    // Retrieve the tile’s metadata.
    TileMetadata* metadata = leaf->getType();
    
    // Instead of initializing a full grid from the mesh,
    // we simply check whether there is already a deformed point near (x,y).
    const int level = leaf->getLevel();
    const float baseThreshold = 0.05f; // adjust this base value as needed
    bool updated = false;
    for (auto& dp : metadata->dirtyVertices) {
        float dx = dp.x - x;
        float dy = dp.y - y;
        
        if (std::sqrt(dx * dx + dy * dy) < baseThreshold) {
            dp.z += dz;
            updated = true;
            break;
        }
    }
    
    if (!updated) {
        // No existing dirty point is nearby, so add a new one.
        vec3 newPoint{x,y,computeBaseElevation(x,y)+dz};
        metadata->dirtyVertices.push_back(newPoint);
    }

    updateMesh(leaf);   
}

float QuadtreeTile::computeBaseElevation(float x, float y) {
    if (geoTIFFLoader) {
        const std::vector<double>& gt = geoTIFFLoader->getGeoTransform();
        // Assuming the geotransform is of the form:
        // [ originX, pixelWidth, rotationX, originY, rotationY, pixelHeight ]
        double originX = gt[0];
        double pixelWidth = gt[1];
        // For consistency with your original code:
        double originY = 0.0;
        double pixelHeight = gt[5];
        
        double col_f = (x - originX) / pixelWidth;
        double row_f = (y - originY) / pixelHeight;
        int col0 = static_cast<int>(std::floor(col_f));
        int row0 = static_cast<int>(std::floor(row_f));
        int col1 = col0 + 1;
        int row1 = row0 + 1;
        
        int width = geoTIFFLoader->getWidth();
        int height = geoTIFFLoader->getHeight();
        const std::vector<float>& elevationData = geoTIFFLoader->getElevationData();
        float interpolatedValue = 0.0f;
        
        if (col0 >= 0 && row0 >= 0 && col1 < width && row1 < height) {
            float v00 = elevationData[row0 * width + col0]; // top-left
            float v10 = elevationData[row0 * width + col1]; // top-right
            float v01 = elevationData[row1 * width + col0]; // bottom-left
            float v11 = elevationData[row1 * width + col1]; // bottom-right
            double tx = col_f - col0;
            double ty = row_f - row0;
            interpolatedValue = static_cast<float>(
                (1 - tx) * (1 - ty) * v00 +
                tx       * (1 - ty) * v10 +
                (1 - tx) * ty       * v01 +
                tx       * ty       * v11
            );
        }
        
        // Optionally add some Perlin noise
        float alpha = 0.7f;
        float frequency = 0.1f;
        float amplitude = 0.25f;
        float upscalePerlin = Perlin::noise(x * frequency, y * frequency) * amplitude;
        
        return interpolatedValue + (alpha * upscalePerlin);
    } else {
        float frequency = 0.1f;
        float amplitude = 1.0f;
        return Perlin::noise(x * frequency, y * frequency) * amplitude;
    }
}


float QuadtreeTile::getElevation(float x, float y) {
    float baseElevation = computeBaseElevation(x, y);

    QuadTree<TileMetadata>* leafNode = findLeafNode(tree, x, y);
    if (!leafNode) return baseElevation; // Return base elevation if no leaf exists.

    const TileMetadata* metadata = leafNode->getType();
    if (metadata->dirtyVertices.empty()) return baseElevation;  // No deformation.

    float weightedSum = 0.0f, totalWeight = 0.0f;
    // const float epsilon = 1e-5f;  // Prevent division by zero.
    const float influenceRadius = 1e-1f;

    for (const auto& dp : metadata->dirtyVertices) {
        float dx = x - dp.x;
        float dy = y - dp.y;
        float distance = std::sqrt(dx * dx + dy * dy);  

        if(distance < influenceRadius) {
            float weight = 1.0f/(distance);
            weightedSum += (dp.z)*weight;
            totalWeight+=weight;
        }
    }

    return (totalWeight > 0.0f) ? weightedSum / totalWeight : baseElevation;
}


QuadTree<TileMetadata>* QuadtreeTile::findLeafNode(QuadTree<TileMetadata>* node, float x, float y) {
    if (!node)
        return nullptr;

    // Get the node’s boundary.
    QuadTree<TileMetadata>::QuadBoundary boundary = node->getBoundary();
    float left   = boundary.x - boundary.width;
    float right  = boundary.x + boundary.width;
    float top    = boundary.y - boundary.height;
    float bottom = boundary.y + boundary.height;

    // If (x,y) lies outside this node, return nullptr.
    if (x < left || x > right || y < top || y > bottom)
        return nullptr;

    // If not divided, this is the leaf.
    if (!node->isDivided())
        return node;

    // Otherwise, search the children.
    QuadTree<TileMetadata>* found = findLeafNode(node->getNortheastNonConst(), x, y);
    if (found) return found;
    found = findLeafNode(node->getNorthwestNonConst(), x, y);
    if (found) return found;
    found = findLeafNode(node->getSoutheastNonConst(), x, y);
    if (found) return found;
    return findLeafNode(node->getSouthwestNonConst(), x, y);
}

void QuadtreeTile::updateMesh(QuadTree<TileMetadata>* node) {
    int level = node->getLevel();
    QuadTree<TileMetadata>::QuadBoundary childBounds = node->getBoundary();

    // Generate the mesh for the child.
    Mesh mesh = generateTriangularMesh(childBounds.x, childBounds.y, childBounds.width, childBounds.height, level);
    bucketMeshes[node] = mesh;
}

Mesh QuadtreeTile::generateTriangularMesh(float centerX, float centerY,
                                            float halfWidth, float halfHeight,
                                            int level)
{
    Mesh mesh;

    int divisions = 1 << (level + 1); // 2^(level + 1)
    int numVerticesPerSide = divisions + 1;

    float stepX = (2.0f * halfWidth) / divisions;
    float stepY = (2.0f * halfHeight) / divisions;

    float startX = centerX - halfWidth;
    float startY = centerY - halfHeight;

    // -------------------------------------------------------
    // 1) Generate the fine mesh vertices and texture coordinates.
    // -------------------------------------------------------
    for (int j = 0; j < numVerticesPerSide; ++j)
    {
        for (int i = 0; i < numVerticesPerSide; ++i)
        {
            float x = startX + i * stepX;
            float y = startY + j * stepY;
            float z = getElevation(x, y);

            // Push back position (x, y, z)
            mesh.vertices.push_back(x);
            mesh.vertices.push_back(y);
            mesh.vertices.push_back(z);

            // Compute and push back texture coordinates (u, v) in [0,1].
            float u = static_cast<float>(i) / divisions; 
            float v = static_cast<float>(j) / divisions;
            mesh.texCoords.push_back(u);
            mesh.texCoords.push_back(v);
        }
    }

    // -------------------------------------------------------
    // 2) Generate indices for a triangular mesh.
    // -------------------------------------------------------
    for (int j = 0; j < divisions; ++j)
    {
        for (int i = 0; i < divisions; ++i)
        {
            int topLeft     =  j      * numVerticesPerSide + i;
            int topRight    =  topLeft + 1;
            int bottomLeft  = (j + 1) * numVerticesPerSide + i;
            int bottomRight =  bottomLeft + 1;

            // First triangle
            mesh.indices.push_back(topLeft);
            mesh.indices.push_back(bottomLeft);
            mesh.indices.push_back(topRight);

            // Second triangle
            mesh.indices.push_back(topRight);
            mesh.indices.push_back(bottomLeft);
            mesh.indices.push_back(bottomRight);
        }
    }


    // -------------------------------------------------------
    // 4) Compute vertex normals based on the positions and indices.
    // -------------------------------------------------------
    calculateNormals(mesh);
    // calculateCoarseNormals(mesh);

    return mesh;
}


// ------------------------------
// Inline Helper Functions
// ------------------------------
void QuadtreeTile::cross(const float* a, const float* b, float* result) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

void QuadtreeTile::normalize(float* v) {
    float length = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (length > 1e-8f) {
        v[0] /= length;
        v[1] /= length;
        v[2] /= length;
    }
}

void QuadtreeTile::calculateNormals(Mesh& mesh)
{
    const size_t vertexCount = mesh.vertices.size() / 3;
    mesh.normals.resize(mesh.vertices.size(), 0.0f); // 3 floats per vertex, initialized to 0

    // Accumulate face normals.
    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        int i0 = mesh.indices[i];
        int i1 = mesh.indices[i + 1];
        int i2 = mesh.indices[i + 2];

        float v0[3] = {
            mesh.vertices[3 * i0 + 0],
            mesh.vertices[3 * i0 + 1],
            mesh.vertices[3 * i0 + 2]
        };
        float v1[3] = {
            mesh.vertices[3 * i1 + 0],
            mesh.vertices[3 * i1 + 1],
            mesh.vertices[3 * i1 + 2]
        };
        float v2[3] = {
            mesh.vertices[3 * i2 + 0],
            mesh.vertices[3 * i2 + 1],
            mesh.vertices[3 * i2 + 2]
        };

        float e1[3] = { v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2] };
        float e2[3] = { v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2] };

        float faceNormal[3];
        cross(e1, e2, faceNormal);

        mesh.normals[3 * i0 + 0] += faceNormal[0];
        mesh.normals[3 * i0 + 1] += faceNormal[1];
        mesh.normals[3 * i0 + 2] += faceNormal[2];

        mesh.normals[3 * i1 + 0] += faceNormal[0];
        mesh.normals[3 * i1 + 1] += faceNormal[1];
        mesh.normals[3 * i1 + 2] += faceNormal[2];

        mesh.normals[3 * i2 + 0] += faceNormal[0];
        mesh.normals[3 * i2 + 1] += faceNormal[1];
        mesh.normals[3 * i2 + 2] += faceNormal[2];
    }

    // Normalize each accumulated normal.
    for (size_t i = 0; i < vertexCount; i++) {
        float* normal = &mesh.normals[3 * i];
        normalize(normal);
    }
}
