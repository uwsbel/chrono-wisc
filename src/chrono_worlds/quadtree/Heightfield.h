// Heightfield.h
#ifndef HEIGHTFIELD_H
#define HEIGHTFIELD_H

#include <vector>
#include <glad/glad.h>
#include <glm/glm.hpp>

class Heightfield
{
public:
    // Constructor
    Heightfield(int gridSize, float gridScale, float heightScale);
    
    // Destructor
    ~Heightfield();
    
    // Generate heightfield data
    void generateHeightfield();
    
    // Setup OpenGL buffers
    void setupMesh();
    
    // Render the heightfield
    void Draw() const;

private:
    // Heightfield parameters
    int GRID_SIZE;
    float GRID_SCALE;
    float HEIGHT_SCALE;

    // Data
    std::vector<float> heights;
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    // OpenGL buffers
    unsigned int VAO, VBO, EBO;
};

#endif
