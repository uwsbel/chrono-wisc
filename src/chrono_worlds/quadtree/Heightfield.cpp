// Heightfield.cpp
#include "Heightfield.h"
#include <cmath>
#include <iostream>

Heightfield::Heightfield(int gridSize, float gridScale, float heightScale)
    : GRID_SIZE(gridSize), GRID_SCALE(gridScale), HEIGHT_SCALE(heightScale),
      VAO(0), VBO(0), EBO(0)
{
    generateHeightfield();
    setupMesh();
}

Heightfield::~Heightfield()
{
    // Cleanup OpenGL buffers
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}

void Heightfield::generateHeightfield()
{
    heights.resize(GRID_SIZE * GRID_SIZE);
    for(int z = 0; z < GRID_SIZE; ++z){
        for(int x = 0; x < GRID_SIZE; ++x){
            float fx = static_cast<float>(x) / static_cast<float>(GRID_SIZE - 1);
            float fz = static_cast<float>(z) / static_cast<float>(GRID_SIZE - 1);
            // Example height function: sine wave (you can replace this with any height function)
            heights[z * GRID_SIZE + x] = 0.0; //std::sin(fx * 10.0f) * std::cos(fz * 10.0f) * HEIGHT_SCALE;
        }
    }

    // Generate vertices
    for(int z = 0; z < GRID_SIZE; ++z){
        for(int x = 0; x < GRID_SIZE; ++x){
            float xpos = (x - GRID_SIZE / 2) * GRID_SCALE;
            float zpos = (z - GRID_SIZE / 2) * GRID_SCALE;
            float ypos = heights[z * GRID_SIZE + x];
            vertices.push_back(xpos);
            vertices.push_back(ypos);
            vertices.push_back(zpos);
        }
    }

    // Generate indices for triangle strips
    for(int z = 0; z < GRID_SIZE -1; ++z){
        for(int x = 0; x < GRID_SIZE -1; ++x){
            int topLeft = z * GRID_SIZE + x;
            int topRight = topLeft + 1;
            int bottomLeft = (z +1)* GRID_SIZE + x;
            int bottomRight = bottomLeft +1;

            // First triangle
            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);

            // Second triangle
            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }
}

void Heightfield::setupMesh()
{
    // Generate OpenGL buffers and arrays
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    // Bind Vertex Array Object
    glBindVertexArray(VAO);

    // Vertex Buffer
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), 
                 vertices.data(), GL_STATIC_DRAW);

    // Element Buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), 
                 indices.data(), GL_STATIC_DRAW);

    // Vertex Attributes
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 
                          3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Unbind VAO (optional)
    glBindVertexArray(0);
}

void Heightfield::Draw() const
{
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), 
                   GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}
