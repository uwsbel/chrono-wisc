#include "chrono_planet/core/Perlin.h"

#include <cmath>

namespace chrono {
namespace planet {

namespace Perlin {
namespace {

// The reference permutation from Perlin (2002), Improving Noise.
const int kPermutation[256] = {
    151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225,
    140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6, 148,
    247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32,
    57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175,
    74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122,
    60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54,
    65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169,
    200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64,
    52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212,
    207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213,
    119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9,
    129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104,
    218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241,
    81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157,
    184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93,
    222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180};

// Low 3 bits of the hash pick one of 8 gradient directions.
float grad(int hash, float x, float y) {
    const int gradientIndex = hash & 7;
    const float primaryComponent = gradientIndex < 4 ? x : y;
    const float secondaryComponent = gradientIndex < 4 ? y : x;
    return ((gradientIndex & 1) ? -primaryComponent : primaryComponent) + ((gradientIndex & 2) ? -2.0f * secondaryComponent : 2.0f * secondaryComponent);
}

// The same eight directions as constants, bit-identical to grad(), so a run of points shares four pairs.
struct Gradient {
    float x, y;
};
Gradient gradOf(int hash) {
    const int gradientIndex = hash & 7;
    const float primarySign = (gradientIndex & 1) ? -1.0f : 1.0f, secondaryWeight = (gradientIndex & 2) ? -2.0f : 2.0f;
    return gradientIndex < 4 ? Gradient{primarySign, secondaryWeight} : Gradient{secondaryWeight, primarySign};
}

}   // namespace

float noise(float x, float y) {
    const int* permutation = kPermutation;
    const int cellX = static_cast<int>(std::floor(x)) & 255;
    const int cellY = static_cast<int>(std::floor(y)) & 255;
    x -= std::floor(x);
    y -= std::floor(y);
    const float fadeX = util::smootherstep01(x), fadeY = util::smootherstep01(y);
    const int leftHash = permutation[cellX] + cellY, rightHash = permutation[(cellX + 1) & 255] + cellY;
    using util::lerp;
    return lerp(lerp(grad(permutation[leftHash & 255], x, y), grad(permutation[rightHash & 255], x - 1, y), fadeX),
                lerp(grad(permutation[(leftHash + 1) & 255], x, y - 1), grad(permutation[(rightHash + 1) & 255], x - 1, y - 1), fadeX), fadeY);
}

float onSphere(const util::Vec3& unitDirection, float coordinateScale) {
    const float x = static_cast<float>(unitDirection.x * coordinateScale), y = static_cast<float>(unitDirection.y * coordinateScale),
                z = static_cast<float>(unitDirection.z * coordinateScale);
    return (noise(x, y) + noise(y, z) + noise(z, x)) * (1.0f / 3.0f);
}

void writeNoiseRow(const float* x, const float* y, int sampleCount, float* noiseValues) {
    const int* permutation = kPermutation;
    int i = 0;
    while (i < sampleCount) {
        // The run is every following point still in this cell.
        const float floorX = std::floor(x[i]), floorY = std::floor(y[i]);
        int end = i + 1;
        while (end < sampleCount && std::floor(x[end]) == floorX && std::floor(y[end]) == floorY) {
            ++end;
        }
        const int cellX = static_cast<int>(floorX) & 255, cellY = static_cast<int>(floorY) & 255;
        const int leftHash = permutation[cellX] + cellY, rightHash = permutation[(cellX + 1) & 255] + cellY;
        const auto [g00x, g00y] = gradOf(permutation[leftHash & 255]);
        const auto [g10x, g10y] = gradOf(permutation[rightHash & 255]);
        const auto [g01x, g01y] = gradOf(permutation[(leftHash + 1) & 255]);
        const auto [g11x, g11y] = gradOf(permutation[(rightHash + 1) & 255]);
        using util::lerp;
        for (int k = i; k < end; ++k) {
            const float fractionX = x[k] - floorX, fractionY = y[k] - floorY;
            const float fadeX = util::smootherstep01(fractionX), fadeY = util::smootherstep01(fractionY);
            const float n00 = g00x * fractionX + g00y * fractionY;
            const float n10 = g10x * (fractionX - 1) + g10y * fractionY;
            const float n01 = g01x * fractionX + g01y * (fractionY - 1);
            const float n11 = g11x * (fractionX - 1) + g11y * (fractionY - 1);
            noiseValues[k] = lerp(lerp(n00, n10, fadeX), lerp(n01, n11, fadeX), fadeY);
        }
        i = end;
    }
}

void writeSphereNoiseRow(const util::Vec3* unitDirections, int sampleCount, float coordinateScale, float* noiseValues, std::vector<float>& scratchValues) {
    scratchValues.resize(static_cast<size_t>(sampleCount) * 5);
    float* latticeX = scratchValues.data();
    float* latticeY = latticeX + sampleCount;
    float* latticeZ = latticeY + sampleCount;
    float* noiseYZ = latticeZ + sampleCount;
    float* noiseZX = noiseYZ + sampleCount;
    for (int i = 0; i < sampleCount; ++i) {
        latticeX[i] = static_cast<float>(unitDirections[i].x * coordinateScale);
        latticeY[i] = static_cast<float>(unitDirections[i].y * coordinateScale);
        latticeZ[i] = static_cast<float>(unitDirections[i].z * coordinateScale);
    }
    writeNoiseRow(latticeX, latticeY, sampleCount, noiseValues);
    writeNoiseRow(latticeY, latticeZ, sampleCount, noiseYZ);
    writeNoiseRow(latticeZ, latticeX, sampleCount, noiseZX);
    for (int i = 0; i < sampleCount; ++i) {   // same order of additions as onSphere()
        noiseValues[i] = (noiseValues[i] + noiseYZ[i] + noiseZX[i]) * (1.0f / 3.0f);
    }
}

}   // namespace Perlin

}  // namespace planet
}  // namespace chrono
