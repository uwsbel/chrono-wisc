// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: bgwitt
// =============================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/core/Parallel.h"
#include "chrono_planet/procedural/ChRoughnessLayer.h"
#include "chrono_planet/procedural/FieldGrid.h"
#include "chrono_planet/core/FieldHash.h"

namespace chrono {
namespace planet {

using field::fastFloor;
using field::hash01;
using field::hash32;

namespace {

// amplitude = slope * wavelength * kSlopeToAmp, measured for shaped Perlin (n|n|).
constexpr double kSlopeToAmp = 3.99;
// The same measurement for clodNoise at unit lattice spacing.
constexpr double kSlopeToAmpCap = 1.154;
// The same measurement for plain, unshaped Perlin, the bed under the clods.
constexpr double kSlopeToAmpBed = 1.593;

// Samples per wavelength for an octave to be fully present (kGateFull) or gone (kGateZero).
constexpr double kGateFull = 2.5;
constexpr double kGateZero = 1.25;

struct Octave {
    double wavelengthDeg, invWavelengthDeg;
    double amplitudeM;      // shaped Perlin (coarse) or the clods (fine)
    double bedAmplitudeM;   // fine only, the continuous bed under the clods
    bool usesClodCaps;
};

// The quintic fade of Perlin (2002), Improving Noise, C2 at the lattice.
inline double fade(double t) { return util::smootherstep01(t); }
using util::lerp;

// Unit gradients in the table, a power of two so a hash masks to an index.
constexpr int kGrads = 16;
constexpr int kGradMask = kGrads - 1;
constexpr float kGradsF = kGrads;
// The gradients, picked by hashed index.
struct GradTable {
    double x[kGrads], y[kGrads];
};
inline const GradTable& grads() {
    static const GradTable g = []() {
        GradTable t{};
        for (int i = 0; i < kGrads; ++i) {
            const double a = i * (util::kTwoPi / kGrads);
            t.x[i] = std::cos(a);
            t.y[i] = std::sin(a);
        }
        return t;
    }();
    return g;
}

// 2D gradient (Perlin) noise, zero at every lattice point.
inline double gradientNoise(double x, double y, int salt) {
    const int ix = fastFloor(x), iy = fastFloor(y);
    const double fx = x - ix, fy = y - iy;
    const GradTable& g = grads();
    auto corner = [&](int cx, int cy, double dx, double dy) {
        const int k = static_cast<int>(hash01(cx, cy, salt) * kGradsF) & kGradMask;
        return g.x[k] * dx + g.y[k] * dy;
    };
    const double u = fade(fx), v = fade(fy);
    const double n0 = lerp(corner(ix, iy, fx, fy), corner(ix + 1, iy, fx - 1.0, fy), u);
    const double n1 = lerp(corner(ix, iy + 1, fx, fy - 1.0), corner(ix + 1, iy + 1, fx - 1.0, fy - 1.0), u);
    return lerp(n0, n1, v);
}

// Lattice rotation for octave octaveIndex, a golden angle apart, so residual anisotropy does not stack.
constexpr double kGoldenAngleRad = 2.39996322972865332;
struct Rotation {
    double cosine, sine;
};
inline Rotation octaveRotation(int octaveIndex) {
    const double a = octaveIndex * kGoldenAngleRad;
    return {std::cos(a), std::sin(a)};
}

// Shapes Perlin into clods. n|n| flattens near zero and keeps the extrema.
inline double shaped(double noiseValue) { return noiseValue * std::abs(noiseValue); }

// Hash byte thresholds, the share of clod cells left empty and the share of caps that are pits instead.
constexpr unsigned kEmptyByte = 77u, kPitByte = 77u;
// Clod cap radius and amplitude ranges in lattice cells, jittered per cap.
constexpr double kCapRadMin = 0.25, kCapRadSpan = 0.35;
constexpr double kCapAmpMin = 0.4, kCapAmpSpan = 0.6;
// A cap's farthest reach past its own cell.
constexpr double kCapReach = kCapRadMin + kCapRadSpan;

// One clod cell's cap, in lattice units: its jittered center, radius and signed amplitude (negative for a pit).
struct ClodCap {
    double centerX, centerY, radiusCells, amplitude;
};
// The cap of cell (cx, cy), or nullopt for an empty cell.
inline std::optional<ClodCap> clodCapOf(int cx, int cy, int salt) {
    const unsigned placementHash = hash32(cx, cy, salt);
    if (((placementHash >> 24) & 255u) < kEmptyByte) {
        return std::nullopt;
    }
    const unsigned shapeHash = hash32(cx, cy, salt + 7919);
    const double jitterX = (placementHash & 255u) * (1.0 / 256.0);
    const double jitterY = ((placementHash >> 8) & 255u) * (1.0 / 256.0);
    const double radiusCells = kCapRadMin + kCapRadSpan * (((placementHash >> 16) & 255u) * (1.0 / 256.0));
    const double amplitude = kCapAmpMin + kCapAmpSpan * ((shapeHash & 255u) * (1.0 / 256.0));
    const bool pit = ((shapeHash >> 8) & 255u) < kPitByte;
    return ClodCap{cx + jitterX, cy + jitterY, radiusCells, pit ? -amplitude : amplitude};
}

// Clod noise, jittered C1 caps and pits in most cells. Mirrored by the fragment shader's capnoised().
inline double clodNoise(double x, double y, int salt) {
    double noiseValue = 0.0;
    field::forEachCellNear(x, y, kCapReach, [&](int cx, int cy) {
        const auto cap = clodCapOf(cx, cy, salt);
        if (!cap) {
            return;
        }
        const double dx = x - cap->centerX, dy = y - cap->centerY;
        const double distanceSquaredCells = dx * dx + dy * dy, radiusSquaredCells = cap->radiusCells * cap->radiusCells;
        if (distanceSquaredCells < radiusSquaredCells) {
            const double u = 1.0 - distanceSquaredCells / radiusSquaredCells;
            noiseValue += cap->amplitude * u * u;
        }
    });
    return noiseValue;
}

// Grid evaluation of the same field as heightAt(), which it must match to rounding.

// Under this many samples per lattice cell, run bookkeeping costs more than hashing every sample.
constexpr double kDenseRunSamples = 8.0;

// gradientNoise() over a row with every sample hashed, in straight-line passes over scratchValues that vectorize.
template <bool Shaped>
inline void addDensePerlinRow(double latticeOriginX, double latticeStepX, double latticeY, Rotation rotation, int salt, int sampleCount, double amplitudeM,
                              double* row, std::vector<double>& scratchValues, std::vector<int>& scratchIndices) {
    const auto [rotationCos, rotationSin] = rotation;
    const GradTable& g = grads();
    scratchValues.resize(static_cast<size_t>(sampleCount) * 10);
    scratchIndices.resize(static_cast<size_t>(sampleCount) * 6);
    double* fx = scratchValues.data();
    double* fy = fx + sampleCount;
    double* cornerGradients = fy + sampleCount;   // g00x g00y g10x g10y g01x g01y g11x g11y, sampleCount each
    int* ix = scratchIndices.data();
    int* iy = ix + sampleCount;
    int* k = iy + sampleCount;   // k00 k10 k01 k11, sampleCount each
    for (int i = 0; i < sampleCount; ++i) {
        const double x = latticeOriginX + i * latticeStepX;
        const double rotatedX = rotationCos * x - rotationSin * latticeY, rotatedY = rotationSin * x + rotationCos * latticeY;
        ix[i] = fastFloor(rotatedX);
        iy[i] = fastFloor(rotatedY);
        fx[i] = rotatedX - ix[i];
        fy[i] = rotatedY - iy[i];
    }
    for (int q = 0; q < 4; ++q) {
        const int ox = q & 1, oy = q >> 1;
        int* kq = k + q * sampleCount;
        for (int i = 0; i < sampleCount; ++i) {
            kq[i] = static_cast<int>(hash01(ix[i] + ox, iy[i] + oy, salt) * kGradsF) & kGradMask;
        }
    }
    for (int q = 0; q < 4; ++q) {
        const int* kq = k + q * sampleCount;
        double* gx = cornerGradients + 2 * q * sampleCount;
        double* gy = gx + sampleCount;
        for (int i = 0; i < sampleCount; ++i) {
            gx[i] = g.x[kq[i]];
            gy[i] = g.y[kq[i]];
        }
    }
    const double* g00x = cornerGradients;
    const double* g00y = g00x + sampleCount;
    const double* g10x = g00y + sampleCount;
    const double* g10y = g10x + sampleCount;
    const double* g01x = g10y + sampleCount;
    const double* g01y = g01x + sampleCount;
    const double* g11x = g01y + sampleCount;
    const double* g11y = g11x + sampleCount;
    for (int i = 0; i < sampleCount; ++i) {
        const double u = fade(fx[i]), v = fade(fy[i]);
        const double n00 = g00x[i] * fx[i] + g00y[i] * fy[i];
        const double n10 = g10x[i] * (fx[i] - 1.0) + g10y[i] * fy[i];
        const double n01 = g01x[i] * fx[i] + g01y[i] * (fy[i] - 1.0);
        const double n11 = g11x[i] * (fx[i] - 1.0) + g11y[i] * (fy[i] - 1.0);
        const double noiseValue = lerp(lerp(n00, n10, u), lerp(n01, n11, u), v);
        row[i] += amplitudeM * (Shaped ? noiseValue * std::abs(noiseValue) : noiseValue);
    }
}

// One row of Perlin in lattice-aligned runs, corner gradients hashed once per run.
template <bool Shaped>
inline void addPerlinRow(double latticeOriginX, double latticeStepX, double latticeY, Rotation rotation, int salt, int sampleCount, double amplitudeM,
                         double* row, std::vector<double>& scratchValues, std::vector<int>& scratchIndices) {
    const auto [rotationCos, rotationSin] = rotation;
    const GradTable& g = grads();
    const double latticeAdvanceX = rotationCos * latticeStepX, latticeAdvanceY = rotationSin * latticeStepX;   // lattice advance per sample
    if (std::max(std::abs(latticeAdvanceX), std::abs(latticeAdvanceY)) * kDenseRunSamples > 1.0) {
        addDensePerlinRow<Shaped>(latticeOriginX, latticeStepX, latticeY, {rotationCos, rotationSin}, salt, sampleCount, amplitudeM, row, scratchValues, scratchIndices);
        return;
    }
    int i = 0;
    while (i < sampleCount) {
        const double xs = latticeOriginX + i * latticeStepX;
        const double rotatedX = rotationCos * xs - rotationSin * latticeY, rotatedY = rotationSin * xs + rotationCos * latticeY;
        const int ix = fastFloor(rotatedX), iy = fastFloor(rotatedY);
        // Samples until rotatedX or rotatedY leaves the cell, bounded as a double so a near-zero advance cannot overflow.
        double len = sampleCount - i;
        if (latticeAdvanceX > 0.0) {
            len = std::min(len, std::ceil((ix + 1 - rotatedX) / latticeAdvanceX));
        } else if (latticeAdvanceX < 0.0) {
            len = std::min(len, std::floor((ix - rotatedX) / latticeAdvanceX) + 1.0);
        }
        if (latticeAdvanceY > 0.0) {
            len = std::min(len, std::ceil((iy + 1 - rotatedY) / latticeAdvanceY));
        } else if (latticeAdvanceY < 0.0) {
            len = std::min(len, std::floor((iy - rotatedY) / latticeAdvanceY) + 1.0);
        }
        const int end = i + std::max(1, static_cast<int>(len));
        const int k00 = static_cast<int>(hash01(ix, iy, salt) * kGradsF) & kGradMask;
        const int k10 = static_cast<int>(hash01(ix + 1, iy, salt) * kGradsF) & kGradMask;
        const int k01 = static_cast<int>(hash01(ix, iy + 1, salt) * kGradsF) & kGradMask;
        const int k11 = static_cast<int>(hash01(ix + 1, iy + 1, salt) * kGradsF) & kGradMask;
        const double g00x = g.x[k00], g00y = g.y[k00], g10x = g.x[k10], g10y = g.y[k10];
        const double g01x = g.x[k01], g01y = g.y[k01], g11x = g.x[k11], g11y = g.y[k11];
        for (; i < end; ++i) {
            const double x = latticeOriginX + i * latticeStepX;
            const double fx = (rotationCos * x - rotationSin * latticeY) - ix, fy = (rotationSin * x + rotationCos * latticeY) - iy;
            const double u = fade(fx), v = fade(fy);
            const double n00 = g00x * fx + g00y * fy;
            const double n10 = g10x * (fx - 1.0) + g10y * fy;
            const double n01 = g01x * fx + g01y * (fy - 1.0);
            const double n11 = g11x * (fx - 1.0) + g11y * (fy - 1.0);
            const double noiseValue = lerp(lerp(n00, n10, u), lerp(n01, n11, u), v);
            row[i] += amplitudeM * (Shaped ? noiseValue * std::abs(noiseValue) : noiseValue);
        }
    }
}

// Lattice cells a grid can see, its rotated corners plus one cell of reach.
struct CellRange {
    int minX, maxX, minY, maxY;   // inclusive
    double count() const { return double(maxX - minX + 1) * double(maxY - minY + 1); }
};
inline CellRange latticeCells(double originLonDeg, double originLatDeg, double lon1, double lat1, double invWavelengthDeg, Rotation rotation) {
    const auto [rotationCos, rotationSin] = rotation;
    double minX = 1e300, maxX = -1e300, minY = 1e300, maxY = -1e300;
    for (int k = 0; k < 4; ++k) {
        const double x = ((k & 1) ? lon1 : originLonDeg) * invWavelengthDeg, y = ((k & 2) ? lat1 : originLatDeg) * invWavelengthDeg;
        const double rotatedX = rotationCos * x - rotationSin * y, rotatedY = rotationSin * x + rotationCos * y;
        minX = std::min(minX, rotatedX);
        maxX = std::max(maxX, rotatedX);
        minY = std::min(minY, rotatedY);
        maxY = std::max(maxY, rotatedY);
    }
    return {fastFloor(minX) - 1, fastFloor(maxX) + 1, fastFloor(minY) - 1, fastFloor(maxY) + 1};
}

// clodNoise() over the grid by scatter, each cap adding itself to the samples inside its disc.
inline void addClodCaps(const ChGeoGrid& grid, field::RowRange rows, const Octave& octaveInfo, const CellRange& cells,
                        Rotation rotation, int salt, double amplitudeM, std::vector<double>& heightsM) {
    const auto [rotationCos, rotationSin] = rotation;
    for (int cy = cells.minY; cy <= cells.maxY; ++cy) {
        for (int cx = cells.minX; cx <= cells.maxX; ++cx) {
            const auto cap = clodCapOf(cx, cy, salt);
            if (!cap) {
                continue;
            }
            // Center back in lon/lat, un-rotated then scaled by the wavelength.
            const double centerLonDeg = (rotationCos * cap->centerX + rotationSin * cap->centerY) * octaveInfo.wavelengthDeg;
            const double centerLatDeg = (-rotationSin * cap->centerX + rotationCos * cap->centerY) * octaveInfo.wavelengthDeg;
            const double radiusDeg = cap->radiusCells * octaveInfo.wavelengthDeg, radiusSquaredDeg2 = radiusDeg * radiusDeg, invRadiusSquaredDeg2 = 1.0 / radiusSquaredDeg2;
            const double signedAmplitudeM = cap->amplitude * amplitudeM;
            const auto offsetLatDeg = [&](int j) { return grid.lat0 + j * grid.step_lat - centerLatDeg; };
            const auto halfWidthDeg = [&](int j) {
                const double halfWidthSquaredDeg2 = radiusSquaredDeg2 - offsetLatDeg(j) * offsetLatDeg(j);
                return halfWidthSquaredDeg2 > 0.0 ? std::sqrt(halfWidthSquaredDeg2) : 0.0;
            };
            field::scatterFootprint(grid, rows, centerLonDeg, centerLatDeg, radiusDeg, halfWidthDeg, [&](int j, int i0, int i1) {
                // Locals, since through `grid` they could alias the row and the loop stops vectorizing.
                const double rowOffsetLatDeg = offsetLatDeg(j), lon0 = grid.lon0, stepLon = grid.step_lon;
                double* row = &heightsM[static_cast<size_t>(j) * grid.n];
                for (int i = i0; i <= i1; ++i) {
                    const double offsetLonDeg = lon0 + i * stepLon - centerLonDeg;
                    const double normalizedDistanceSquared = (offsetLonDeg * offsetLonDeg + rowOffsetLatDeg * rowOffsetLatDeg) * invRadiusSquaredDeg2;
                    const double capWeight = 1.0 - normalizedDistanceSquared;
                    row[i] += (normalizedDistanceSquared < 1.0) ? signedAmplitudeM * capWeight * capWeight : 0.0;
                }
            });
        }
    }
}

}  // namespace

// Parameters with the derived per-octave table.
struct ChRoughnessLayer::Model {
    Params params;
    std::vector<Octave> octaves;

    int count() const { return static_cast<int>(octaves.size()); }
    // Resolvability fade of octave octaveIndex at the given sampling, in [0, 1].
    double octaveFade(int octaveIndex, double invSpacingDeg) const {
        return field::resolutionFade(octaves[octaveIndex].wavelengthDeg * invSpacingDeg, kGateZero, kGateFull);
    }
};

namespace {

void addGridRows(const ChRoughnessLayer::Model& m, const ChGeoGrid& grid, field::RowRange rows, std::vector<double>& heightsM) {
    const auto [originLonDeg, originLatDeg, stepLonDeg, stepLatDeg, sampleCount, sampleSpacingDeg] = grid;
    const auto [rowBegin, rowEnd] = rows;
    if (rowBegin >= rowEnd) {
        return;
    }
    const double invSampleSpacingDeg = 1.0 / std::max(sampleSpacingDeg, 1e-9);
    const double lon1 = originLonDeg + (sampleCount - 1) * stepLonDeg, lat1 = originLatDeg + (sampleCount - 1) * stepLatDeg;
    const double latB0 = originLatDeg + rowBegin * stepLatDeg, latB1 = originLatDeg + (rowEnd - 1) * stepLatDeg;
    const double samples = static_cast<double>(sampleCount) * sampleCount;
    std::vector<double> scratchValues;
    std::vector<int> scratchIndices;
    for (int octaveIndex = 0; octaveIndex < m.count(); ++octaveIndex) {
        const double resolutionWeight = m.octaveFade(octaveIndex, invSampleSpacingDeg);
        if (resolutionWeight <= 0.0) {
            break;
        }
        const Octave& octaveInfo = m.octaves[octaveIndex];
        const double amplitudeM = octaveInfo.amplitudeM * resolutionWeight, bedAmplitudeM = octaveInfo.bedAmplitudeM * resolutionWeight;
        const int salt = field::seeded(octaveIndex * 977 + 31, m.params.seed);
        const auto [rotationCos, rotationSin] = octaveRotation(octaveIndex);
        const double latticeOriginX = originLonDeg * octaveInfo.invWavelengthDeg, latticeStepX = stepLonDeg * octaveInfo.invWavelengthDeg;

        if (octaveInfo.usesClodCaps) {
            // Scatter caps while a cell spans a grid step or more, otherwise gather per sample.
            const CellRange cells = latticeCells(originLonDeg, originLatDeg, lon1, lat1, octaveInfo.invWavelengthDeg, {rotationCos, rotationSin});
            const bool scatter = cells.count() <= 2.0 * samples;
            if (scatter) {
                const CellRange bandCells = latticeCells(originLonDeg, latB0, lon1, latB1, octaveInfo.invWavelengthDeg, {rotationCos, rotationSin});
                addClodCaps(grid, rows, octaveInfo, bandCells, {rotationCos, rotationSin}, salt, amplitudeM, heightsM);
            }
            for (int j = rowBegin; j < rowEnd; ++j) {
                const double latticeY = (originLatDeg + j * stepLatDeg) * octaveInfo.invWavelengthDeg;
                double* row = &heightsM[static_cast<size_t>(j) * sampleCount];
                if (!scatter) {
                    for (int i = 0; i < sampleCount; ++i) {
                        const double x = latticeOriginX + i * latticeStepX;
                        row[i] += amplitudeM * clodNoise(rotationCos * x - rotationSin * latticeY, rotationSin * x + rotationCos * latticeY, salt);
                    }
                }
                // Its own salt, so the bed does not correlate with the clods on it.
                addPerlinRow<false>(latticeOriginX, latticeStepX, latticeY, {rotationCos, rotationSin}, salt + 5003, sampleCount, bedAmplitudeM, row, scratchValues, scratchIndices);
            }
        } else {
            for (int j = rowBegin; j < rowEnd; ++j) {
                const double latticeY = (originLatDeg + j * stepLatDeg) * octaveInfo.invWavelengthDeg;
                addPerlinRow<true>(latticeOriginX, latticeStepX, latticeY, {rotationCos, rotationSin}, salt, sampleCount, amplitudeM, &heightsM[static_cast<size_t>(j) * sampleCount], scratchValues, scratchIndices);
            }
        }
    }
}
}  // namespace

// -----------------------------------------------------------------------------

ChRoughnessLayer::ChRoughnessLayer(const ChPlanetBody& body, const Params& p) {
    if (p.octaves < 0 || !(p.coarsest_wavelength > 0))
        throw std::invalid_argument("ChRoughnessLayer: octaves must be non-negative and coarsest_wavelength positive");
    if (!(p.clod_variance_share >= 0 && p.clod_variance_share <= 1))
        throw std::invalid_argument("ChRoughnessLayer: clod_variance_share must lie in [0, 1]");
    auto model = std::make_unique<Model>();
    model->params = p;
    const double kKmPerDeg = field::kmPerDeg(body.GetRadius());
    // A smooth surface has nothing to walk.
    const int count = p.slope_rms > 0 ? p.octaves : 0;
    model->octaves.resize(count);
    for (int octaveIndex = 0; octaveIndex < count; ++octaveIndex) {
        Octave& o = model->octaves[octaveIndex];
        const double wavelengthM = p.coarsest_wavelength / std::exp2(static_cast<double>(octaveIndex));
        o.wavelengthDeg = (wavelengthM / 1000.0) / kKmPerDeg;
        o.invWavelengthDeg = 1.0 / o.wavelengthDeg;
        o.usesClodCaps = wavelengthM < p.clod_below;
        const double slope = p.slope_rms * std::pow(p.fine_boost, octaveIndex);
        if (o.usesClodCaps) {
            o.amplitudeM = std::sqrt(p.clod_variance_share) * slope * wavelengthM * kSlopeToAmpCap;
            o.bedAmplitudeM = std::sqrt(1.0 - p.clod_variance_share) * slope * wavelengthM * kSlopeToAmpBed;
        } else {
            o.amplitudeM = slope * wavelengthM * kSlopeToAmp;
            o.bedAmplitudeM = 0.0;
        }
    }
    m_model = std::move(model);
}

ChRoughnessLayer::~ChRoughnessLayer() = default;

const ChRoughnessLayer::Params& ChRoughnessLayer::GetParams() const {
    return m_model->params;
}

double ChRoughnessLayer::GetHeight(double lonDeg, double latDeg, double sampleSpacingDeg) const {
    const Model& m = *m_model;
    const double invSampleSpacingDeg = 1.0 / std::max(sampleSpacingDeg, 1e-9);
    double reliefM = 0.0;
    for (int octaveIndex = 0; octaveIndex < m.count(); ++octaveIndex) {
        const double resolutionWeight = m.octaveFade(octaveIndex, invSampleSpacingDeg);
        if (resolutionWeight <= 0.0) {
            break;   // octaves get finer, so the rest are finer still
        }
        const Octave& octaveInfo = m.octaves[octaveIndex];
        const auto [rotationCos, rotationSin] = octaveRotation(octaveIndex);
        const double x = lonDeg * octaveInfo.invWavelengthDeg, y = latDeg * octaveInfo.invWavelengthDeg;
        const double rotatedX = rotationCos * x - rotationSin * y, rotatedY = rotationSin * x + rotationCos * y;
        const int salt = field::seeded(octaveIndex * 977 + 31, m.params.seed);
        reliefM += octaveInfo.amplitudeM * resolutionWeight * (octaveInfo.usesClodCaps ? clodNoise(rotatedX, rotatedY, salt) : shaped(gradientNoise(rotatedX, rotatedY, salt)));
        // Own salt, so the bed does not correlate with the clods on it.
        if (octaveInfo.bedAmplitudeM > 0.0) {
            reliefM += octaveInfo.bedAmplitudeM * resolutionWeight * gradientNoise(rotatedX, rotatedY, salt + 5003);
        }
    }
    return reliefM;
}

void ChRoughnessLayer::AddToGrid(const ChGeoGrid& grid, std::vector<double>& heights) const {
    const Model& m = *m_model;
    if (m.octaves.empty()) {
        return;
    }
    field::forEachRowBand(grid.n, [&](field::RowRange rows) { addGridRows(m, grid, rows, heights); });
}

double ChRoughnessLayer::GetRmsSlope(double sampleSpacingDeg) const {
    const Model& m = *m_model;
    const double invSampleSpacingDeg = 1.0 / std::max(sampleSpacingDeg, 1e-9);
    double slopeVariance = 0.0;
    for (int octaveIndex = 0; octaveIndex < m.count(); ++octaveIndex) {
        const double resolutionWeight = m.octaveFade(octaveIndex, invSampleSpacingDeg);
        if (resolutionWeight <= 0.0) {
            break;
        }
        const double octaveSlope = m.params.slope_rms * std::pow(m.params.fine_boost, octaveIndex) * resolutionWeight;
        slopeVariance += octaveSlope * octaveSlope;
    }
    return std::sqrt(slopeVariance);
}

}  // namespace planet
}  // namespace chrono
