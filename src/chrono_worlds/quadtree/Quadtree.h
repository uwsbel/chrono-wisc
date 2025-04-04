#ifndef QUADTREE_H
#define QUADTREE_H

#include <iostream>
#include <cmath>
#include <functional>
#include <vector>
#include <stack>

using std::cout;
using std::endl;

template<typename T>
class QuadTree {
public:
    using NodeInitCallback = std::function<void(QuadTree<T>*)>;
    using NodeDestroyCallback = std::function<void(QuadTree<T>*)>;
    using NodeSplitCallback = std::function<void(QuadTree<T>*)>;
    using NodeMergeCallback = std::function<void(QuadTree<T>*)>;

    NodeInitCallback nodeInitCallback;
    NodeDestroyCallback nodeDestroyCallback;
    NodeSplitCallback nodeSplitCallback;
    NodeMergeCallback nodeMergeCallback;
    
    QuadTree(float x, float y, float width, float height, T type = T{}, int level = 0, 
             NodeInitCallback nodeInitCallback = nullptr,
             NodeDestroyCallback nodeDestroyCallback = nullptr, 
             NodeSplitCallback nodeSplitCallback = nullptr,
             NodeMergeCallback nodeMergeCallback = nullptr,
             QuadTree<T>* parent = nullptr)
        : boundary{ x, y, width, height },
          type(type),
          divided(false),
          level(level),
          nodeInitCallback(nodeInitCallback),
          nodeDestroyCallback(nodeDestroyCallback),
          nodeSplitCallback(nodeSplitCallback),
          nodeMergeCallback(nodeMergeCallback),
          m_parent(parent),
          cacheValid(false) // Initialize cache as invalid
    {
        if (nodeInitCallback) {
            nodeInitCallback(this);
        }

        northeast = nullptr;
        northwest = nullptr;
        southeast = nullptr;
        southwest = nullptr;
    }

    ~QuadTree() {
        if(nodeDestroyCallback) {
            nodeDestroyCallback(this);
        }
        delete northeast;
        delete northwest;
        delete southeast;
        delete southwest;
    }

    struct QuadBoundary {
        float x, y;
        float width;
        float height;
    };

    QuadBoundary getBoundary() const { return boundary; }
    bool isDivided() const { return divided; }

    T* getType() { return &type; }
    void setType(T newType) { type = newType; }

    void setScale(int targetDepth) {
        adjustScale(targetDepth, 0);
    }

    void adjustScale(int targetDepth, int currentDepth) {
        if (currentDepth < targetDepth) {
            if (!divided) {
                subdivide();
            }
            int nextDepth = currentDepth + 1;
            northeast->adjustScale(targetDepth, nextDepth);
            northwest->adjustScale(targetDepth, nextDepth);
            southeast->adjustScale(targetDepth, nextDepth);
            southwest->adjustScale(targetDepth, nextDepth);
        } else if (currentDepth >= targetDepth) {
            if (divided) {
                merge();
            }
        }
    }

    void subdivide() {
        if (divided) return;
        
        float x = boundary.x;
        float y = boundary.y;
        float w = boundary.width / 2.0f;
        float h = boundary.height / 2.0f;

        int childLevel = level + 1;
        northeast = new QuadTree(x + w, y - h, w, h, type, childLevel, nodeInitCallback, nodeDestroyCallback, nodeSplitCallback, nodeMergeCallback, this);
        northwest = new QuadTree(x - w, y - h, w, h, type, childLevel, nodeInitCallback, nodeDestroyCallback, nodeSplitCallback, nodeMergeCallback, this);
        southeast = new QuadTree(x + w, y + h, w, h, type, childLevel, nodeInitCallback, nodeDestroyCallback, nodeSplitCallback, nodeMergeCallback, this);
        southwest = new QuadTree(x - w, y + h, w, h, type, childLevel, nodeInitCallback, nodeDestroyCallback, nodeSplitCallback, nodeMergeCallback, this);        
        divided = true;

        clearCache(); // Invalidate cache on structure change

        if(nodeSplitCallback) {
            nodeSplitCallback(this);
        }
    }
    
    void merge() {
        if (!divided) return;

        if(nodeMergeCallback) {
            nodeMergeCallback(this);
        }

        delete northeast;
        delete northwest;
        delete southeast;
        delete southwest;
        
        northeast = northwest = southeast = southwest = nullptr;
        divided = false;

        clearCache(); // Invalidate cache on structure change
    }

    int getLevel() const { return level; }

    int getNumChildren() const {
        if (!divided)
            return 0;
        return 4 + northeast->getNumChildren() +
                   northwest->getNumChildren() +
                   southeast->getNumChildren() +
                   southwest->getNumChildren();
    }

    void collectLeaves(std::vector<QuadTree<T>*>& leaves) {
        std::stack<QuadTree<T>*> stack;
        stack.push(this);
        while(!stack.empty()) {
            QuadTree<T>* node = stack.top();
            stack.pop();
            if(!node->divided) {
                leaves.push_back(node);
            } else {
                if(node->northeast) stack.push(node->northeast);
                if(node->northwest) stack.push(node->northwest);
                if(node->southeast) stack.push(node->southeast);
                if(node->southwest) stack.push(node->southwest);
            }
        }
        
    }

    std::vector<QuadTree<T>*> getLeaves() {
        // if (cacheValid) {
        //     return leafCache; // Return cached result if valid
        // }
        // std::cout <<"Invalid" << std::endl;

        leafCache.clear();
        collectLeaves(leafCache);
        cacheValid = true; // Mark cache as valid
        return leafCache;
    }

    bool getCacheValid() { return cacheValid; }

    QuadTree* getNortheastNonConst() { return northeast; }
    QuadTree* getNorthwestNonConst() { return northwest; }
    QuadTree* getSoutheastNonConst() { return southeast; }
    QuadTree* getSouthwestNonConst() { return southwest; }
    QuadTree* getNortheast() const { return northeast; }
    QuadTree* getNorthwest() const { return northwest; }
    QuadTree* getSoutheast() const { return southeast; }
    QuadTree* getSouthwest() const { return southwest; }

private:
    QuadBoundary boundary;
    T type;
    bool divided = false;
    int level;
    QuadTree* northeast;
    QuadTree* northwest;
    QuadTree* southeast;
    QuadTree* southwest;
    QuadTree* m_parent;
    
    std::vector<QuadTree<T>*> leafCache; // Cache for leaf nodes
    bool cacheValid; // Flag to track cache validity

    void clearCache() {
        cacheValid = false; // Invalidate cache when the tree structure changes
    }
};

#endif // QUADTREE_H
