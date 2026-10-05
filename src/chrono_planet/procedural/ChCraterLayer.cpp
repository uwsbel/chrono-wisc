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
#include <utility>
#include <vector>

#include "chrono_planet/core/Parallel.h"
#include "chrono_planet/core/SphereMath.h"
#include "chrono_planet/procedural/ChCraterLayer.h"
#include "chrono_planet/procedural/FieldGrid.h"
#include "chrono_planet/core/FieldHash.h"

namespace chrono {
namespace planet {

using field::bump;
using field::fastFloor;
using field::hash01;
using field::seeded;

namespace {

// Rim bump support either side of the crest, in rim sigmas.
constexpr double kRimWidth = 2.2;
// Rim half-width cap in radiusFraction. Past kReachQ the profile is identically zero.
constexpr double kRimSigmaMax = 0.3;
constexpr double kReachQ = 1.0 + kRimWidth * kRimSigmaMax;
// Ejecta apron edge (in radiusFraction), also a crater's farthest reach.
constexpr double kEjectaQ = 2.0;
// Central peak diameter and height as fractions of the crater's own, after Pike.
constexpr double kPeakDiamFrac = 0.22;
constexpr double kPeakHeightFrac = 0.22;
// Jitter-grid cell size in crater diameters, and a crater's farthest reach in cells.
constexpr double kCellDiams = 2.5;
constexpr double kReachCells = kEjectaQ / (2.0 * kCellDiams);
// Octave fade bounds in vertices across a crater, gone below the low one and full above the high.
constexpr double kFadeLoVerts = 2.0, kFadeHiVerts = 4.0;

// Crater profile at radiusFraction = distance / rim radius. Floor term in [-1,0], rim term in [0,1]. Branch-free so the footprint loop vectorizes.
struct CraterProfile {
    double bowl, rim;
};
inline CraterProfile craterProfile(double radiusFraction, double flatness, double degradation, double rimWidthFraction) {
    const double parabolicBowl = -(1.0 - radiusFraction * radiusFraction);
    constexpr double wallStart = 0.55, invWall = 1.0 / (1.0 - wallStart);
    const double wallFraction = (radiusFraction - wallStart) * invWall;
    const double flatBottomBowl = (radiusFraction < wallStart) ? -1.0 : -(1.0 - wallFraction * wallFraction);
    double freshBowl = parabolicBowl * (1.0 - flatness) + flatBottomBowl * flatness;
    freshBowl = freshBowl * std::abs(freshBowl);   // ease toward the crest
    // Degraded craters lose the sharp wall/rim junction.
    const double degradedBowl = -bump(radiusFraction);
    const double bowl = (radiusFraction < 1.0) ? freshBowl * (1.0 - degradation) + degradedBowl * degradation : 0.0;
    const double rimBump = bump((radiusFraction - 1.0) / (kRimWidth * rimWidthFraction));   // crest at the rim radius
    // Ejecta apron, flat inside the rim and gone at kEjectaQ.
    const double ejectaDistanceFraction = kEjectaQ - radiusFraction;
    const double ejectaEnvelope = (radiusFraction > 1.0) ? ejectaDistanceFraction * ejectaDistanceFraction * (1.0 - 0.6 * degradation * (1.0 - ejectaDistanceFraction)) : 1.0;
    return {bowl, rimBump * ejectaEnvelope};
}

struct OctaveInfo {
    double diameterKm, density;
    double diameterDeg, cellSizeDeg, invCellSizeDeg;
};

}  // namespace

// Parameters with their derived per-octave tables.
struct ChCraterLayer::Model {
    Params params;
    double kmPerDeg;
    std::vector<OctaveInfo> octaves;

    int count() const { return static_cast<int>(octaves.size()); }
    const OctaveInfo& octave(int octaveIndex) const { return octaves[octaveIndex]; }
    // Octave fade, continuous so neighboring LODs agree.
    double octaveFade(int octaveIndex, double invSpacingDeg) const {
        return field::resolutionFade(octaves[octaveIndex].diameterDeg * invSpacingDeg, kFadeLoVerts, kFadeHiVerts);
    }
    float hash(int gx, int gy, int salt) const { return hash01(gx, gy, seeded(salt, params.seed)); }
};

namespace {

using Model = ChCraterLayer::Model;

struct CraterInstance {
    double centerLonDeg, centerLatDeg;   // center (deg)
    double nominalRadiusKm;              // nominal radius (km)
    double age, preservationFactor;      // fresh to subdued, and the depth/rim multiplier
    double depthKm, rimKm, flatness;
    bool isComplex;
    double outlineAmplitude, streakAmplitude, invPeakRadiusFraction;
    double cosineHarmonics[6], sineHarmonics[6];   // cos/sin coefficients: outline harmonics 2/3/5, then streak harmonics 5/7/8
    double baseRimWidthFraction;
    double reachKm;   // relief is exactly zero beyond this radius

    // Returns nullopt if the cell has no crater.
    [[nodiscard]] static std::optional<CraterInstance> make(int gx, int gy, int octaveIndex, const Model& m) {
        CraterInstance crater;
        const OctaveInfo& info = m.octave(octaveIndex);
        if (m.hash(gx, gy, octaveIndex * 13 + 1) > info.density) {
            return std::nullopt;
        }
        const ChCraterLayer::Params& p = m.params;
        const double cellSizeDeg = info.cellSizeDeg, classDiameterKm = info.diameterKm;
        // Center anywhere in the cell, or the lattice shows as dimple rows.
        crater.centerLonDeg = (gx + m.hash(gx, gy, octaveIndex * 13 + 2)) * cellSizeDeg;
        crater.centerLatDeg = (gy + m.hash(gx, gy, octaveIndex * 13 + 3)) * cellSizeDeg;
        crater.nominalRadiusKm = 0.5 * classDiameterKm * (0.55 + 0.45 * m.hash(gx, gy, octaveIndex * 13 + 4));
        const double ageSample = m.hash(gx, gy, octaveIndex * 13 + 5);
        crater.age = 1.0 - (1.0 - ageSample) * (1.0 - ageSample);   // skewed old
        crater.preservationFactor = 1.0 - 0.85 * crater.age;
        // Depth and rim height from power laws in diameter, after Pike (1977).
        const double diameterKm = 2.0 * crater.nominalRadiusKm;
        crater.isComplex = diameterKm >= p.complex_diameter_km;
        // A small crater's share of a simple crater's depth and rim: 1 from shallow_below_km up
        double shallow = 1.0;
        if (p.shallow_below_km > 0 && diameterKm < p.shallow_below_km) {
            const double t = std::min(1.0, std::log(p.shallow_below_km / diameterKm) / std::log(p.shallow_below_km / p.shallow_at_km));
            shallow = 1.0 + t * (p.shallow_depth_ratio / p.simple_depth_ratio - 1.0);
        }
        crater.depthKm = (crater.isComplex ? p.complex_depth_coef * std::pow(diameterKm, p.complex_depth_exp) : p.simple_depth_ratio * shallow * diameterKm) * crater.preservationFactor;
        crater.rimKm = (crater.isComplex ? p.complex_rim_coef * std::pow(diameterKm, p.complex_rim_exp) : p.simple_rim_ratio * shallow * diameterKm) * crater.preservationFactor;
        crater.flatness = crater.isComplex ? std::min(1.0, (diameterKm - p.complex_diameter_km) / p.floor_flattening_km) : 0.0;
        crater.outlineAmplitude = 0.04 + 0.08 * crater.age;                              // lumpy outline, old craters more so
        crater.streakAmplitude = (crater.age < 0.8) ? 0.45 * (1.0 - crater.age) : 0.0;   // ejecta rays, fresh only
        const double peakRadiusFraction = kPeakDiamFrac * (1.0 + 0.3 * (m.hash(gx, gy, octaveIndex * 13 + 12) - 0.5));
        crater.invPeakRadiusFraction = 1.0 / (peakRadiusFraction * 1.3);
        const double harmonicWeights[3] = {0.50, 0.32, 0.18};
        double outlineMax = 1.0;
        for (int k = 0; k < 3; ++k) {
            crater.cosineHarmonics[k] = 1.4 * harmonicWeights[k] * (m.hash(gx, gy, octaveIndex * 13 + 6 + 2 * k) - 0.5);
            crater.sineHarmonics[k] = 1.4 * harmonicWeights[k] * (m.hash(gx, gy, octaveIndex * 13 + 7 + 2 * k) - 0.5);
            crater.cosineHarmonics[3 + k] = 1.4 * harmonicWeights[k] * (m.hash(gx, gy, octaveIndex * 13 + 14 + 2 * k) - 0.5);
            crater.sineHarmonics[3 + k] = 1.4 * harmonicWeights[k] * (m.hash(gx, gy, octaveIndex * 13 + 15 + 2 * k) - 0.5);
            // a cos + b sin <= sqrt(a^2 + b^2), the outline's largest excursion.
            outlineMax += crater.outlineAmplitude * std::sqrt(crater.cosineHarmonics[k] * crater.cosineHarmonics[k] + crater.sineHarmonics[k] * crater.sineHarmonics[k]);
        }
        // Rim half-width, widened per sample so it never aliases.
        crater.baseRimWidthFraction = 0.08 + 0.17 * crater.age;
        crater.reachKm = std::min(kEjectaQ * crater.nominalRadiusKm, kReachQ * crater.nominalRadiusKm * outlineMax);
        return crater;
    }

    [[nodiscard]] double reliefKmAtOffset(double offsetEastKm, double offsetNorthKm, double spacingKm) const {
        const double distanceSquaredKm2 = offsetEastKm * offsetEastKm + offsetNorthKm * offsetNorthKm;
        const double distanceKm = std::sqrt(distanceSquaredKm2);
        const double invDistanceKm = 1.0 / std::max(distanceKm, 1e-9);
        const double cosAzimuth1 = offsetEastKm * invDistanceKm, sinAzimuth1 = offsetNorthKm * invDistanceKm;   // (cos, sin) azimuth
        // Multiple angles by products of lower ones, no trig per sample.
        const double cosAzimuth2 = cosAzimuth1 * cosAzimuth1 - sinAzimuth1 * sinAzimuth1, sinAzimuth2 = 2.0 * cosAzimuth1 * sinAzimuth1;
        const double cosAzimuth3 = cosAzimuth2 * cosAzimuth1 - sinAzimuth2 * sinAzimuth1, sinAzimuth3 = sinAzimuth2 * cosAzimuth1 + cosAzimuth2 * sinAzimuth1;
        const double cosAzimuth5 = cosAzimuth2 * cosAzimuth3 - sinAzimuth2 * sinAzimuth3, sinAzimuth5 = sinAzimuth2 * cosAzimuth3 + cosAzimuth2 * sinAzimuth3;
        const double outline = 1.0 + outlineAmplitude * (cosineHarmonics[0] * cosAzimuth2 + sineHarmonics[0] * sinAzimuth2 + cosineHarmonics[1] * cosAzimuth3 + sineHarmonics[1] * sinAzimuth3 + cosineHarmonics[2] * cosAzimuth5 + sineHarmonics[2] * sinAzimuth5);
        const double invOutlineRadiusKm = 1.0 / (nominalRadiusKm * outline);
        const double radiusFraction = distanceKm * invOutlineRadiusKm;
        const double rimWidthFraction = std::min(kRimSigmaMax, std::max(baseRimWidthFraction, 0.8 * spacingKm * invOutlineRadiusKm));
        const auto [bowl, rim] = craterProfile(radiusFraction, flatness, age, rimWidthFraction);
        // Ejecta rays, past the rim only. Unconditional so the footprint loop vectorizes.
        const double cosAzimuth7 = cosAzimuth2 * cosAzimuth5 - sinAzimuth2 * sinAzimuth5, sinAzimuth7 = sinAzimuth2 * cosAzimuth5 + cosAzimuth2 * sinAzimuth5;
        const double cosAzimuth8 = cosAzimuth3 * cosAzimuth5 - sinAzimuth3 * sinAzimuth5, sinAzimuth8 = sinAzimuth3 * cosAzimuth5 + cosAzimuth3 * sinAzimuth5;
        const double streak = cosineHarmonics[3] * cosAzimuth5 + sineHarmonics[3] * sinAzimuth5 + cosineHarmonics[4] * cosAzimuth7 + sineHarmonics[4] * sinAzimuth7 + cosineHarmonics[5] * cosAzimuth8 + sineHarmonics[5] * sinAzimuth8;
        const double ejectaMod = 1.0 + streakAmplitude * streak * std::min(1.0, std::max(0.0, (radiusFraction - 1.0) * 4.0));
        double reliefKm = depthKm * bowl + rimKm * rim * ejectaMod;
        // Central peak, zero for simple craters and past the peak radius.
        reliefKm += depthKm * kPeakHeightFrac * preservationFactor * flatness * bump(radiusFraction * invPeakRadiusFraction);
        return reliefKm;
    }
};

// One octave's craters over the whole grid, a vector per cell row in gx order, the order GetHeight() sums in.
struct OctaveCraters {
    double scale, cellSizeDeg, invCellSizeDeg;
    int gy0, gy1;
    std::vector<std::vector<CraterInstance>> rows;
};

// Instantiates every crater that can reach the grid once, cell rows in parallel. The bands then walk their own rows.
std::vector<OctaveCraters> instantiate(const Model& m, double originLonDeg, double originLatDeg, double lon1, double lat1, double invSampleSpacingDeg) {
    std::vector<OctaveCraters> out;
    std::vector<std::pair<int, int>> jobs;   // (octave, row)
    for (int octaveIndex = 0; octaveIndex < m.count(); ++octaveIndex) {
        const double fade = m.octaveFade(octaveIndex, invSampleSpacingDeg);
        if (fade <= 0.0) {
            break;
        }
        OctaveCraters octaveInfo;
        octaveInfo.scale = 1000.0 * fade;
        octaveInfo.cellSizeDeg = m.octave(octaveIndex).cellSizeDeg;
        octaveInfo.invCellSizeDeg = m.octave(octaveIndex).invCellSizeDeg;
        octaveInfo.gy0 = fastFloor((originLatDeg - kReachCells * octaveInfo.cellSizeDeg) * octaveInfo.invCellSizeDeg);
        octaveInfo.gy1 = fastFloor((lat1 + kReachCells * octaveInfo.cellSizeDeg) * octaveInfo.invCellSizeDeg);
        octaveInfo.rows.resize(static_cast<size_t>(octaveInfo.gy1 - octaveInfo.gy0 + 1));
        for (int gy = octaveInfo.gy0; gy <= octaveInfo.gy1; ++gy) {
            jobs.emplace_back(octaveIndex, gy);
        }
        out.push_back(std::move(octaveInfo));
    }
    const int jobCount = static_cast<int>(jobs.size());
#pragma omp parallel for num_threads(util::parallelThreads()) schedule(dynamic, 4) if (jobCount >= 64)
    for (int t = 0; t < jobCount; ++t) {
        const int octaveIndex = jobs[t].first, gy = jobs[t].second;
        OctaveCraters& octaveInfo = out[octaveIndex];
        const int gx0 = fastFloor((originLonDeg - kReachCells * octaveInfo.cellSizeDeg) * octaveInfo.invCellSizeDeg), gx1 = fastFloor((lon1 + kReachCells * octaveInfo.cellSizeDeg) * octaveInfo.invCellSizeDeg);
        std::vector<CraterInstance>& row = octaveInfo.rows[gy - octaveInfo.gy0];
        for (int gx = gx0; gx <= gx1; ++gx) {
            if (auto crater = CraterInstance::make(gx, gy, octaveIndex, m)) {
                row.push_back(*crater);
            }
        }
    }
    return out;
}

// One crater's relief over samples i0..i1 of grid row j. The crater and grid are taken by value: through a
// reference they could alias the row, and the loop stops vectorizing.
void addCraterSpan(const CraterInstance crater, const ChGeoGrid grid, int j, int i0, int i1, double offsetNorthKm,
                   double kmPerDeg, double sampleSpacingKm, double scale, std::vector<double>& heightsM) {
    double* row = &heightsM[static_cast<size_t>(j) * grid.n];
    for (int i = i0; i <= i1; ++i) {
        const double offsetEastKm = (grid.lon0 + i * grid.step_lon - crater.centerLonDeg) * kmPerDeg;
        row[i] += crater.reliefKmAtOffset(offsetEastKm, offsetNorthKm, sampleSpacingKm) * scale;
    }
}

// Crater relief over rows `rows` of the grid, craters in the whole-grid walk order, so sums match serial bit for bit.
void addCraterRows(const Model& m, const ChGeoGrid& grid, field::RowRange rows,
                   const std::vector<OctaveCraters>& craterOctaves, const double* kmPerDegLon,
                   const double* spacingKm, std::vector<double>& heightsM) {
    const double kKmPerDeg = m.kmPerDeg;
    const double latB0 = grid.lat0 + rows.begin * grid.step_lat, latB1 = grid.lat0 + (rows.end - 1) * grid.step_lat;
    for (const OctaveCraters& octaveInfo : craterOctaves) {
        const double scale = octaveInfo.scale;
        // Cell rows whose crater can touch the band.
        const int gyB0 = std::max(octaveInfo.gy0, fastFloor((latB0 - kReachCells * octaveInfo.cellSizeDeg) * octaveInfo.invCellSizeDeg));
        const int gyB1 = std::min(octaveInfo.gy1, fastFloor((latB1 + kReachCells * octaveInfo.cellSizeDeg) * octaveInfo.invCellSizeDeg));
        for (int gy = gyB0; gy <= gyB1; ++gy) {
            for (const CraterInstance& crater : octaveInfo.rows[gy - octaveInfo.gy0]) {
                // The disk where the relief is nonzero.
                const double reach2 = crater.reachKm * crater.reachKm;
                const auto northKm = [&](int j) { return (grid.lat0 + j * grid.step_lat - crater.centerLatDeg) * kKmPerDeg; };
                const auto halfWidthDeg = [&](int j) {
                    const double offsetNorthKm = northKm(j), half2 = reach2 - offsetNorthKm * offsetNorthKm;
                    return half2 > 0.0 ? std::sqrt(half2) / kmPerDegLon[j] : 0.0;
                };
                field::scatterFootprint(grid, rows, crater.centerLonDeg, crater.centerLatDeg, crater.reachKm / kKmPerDeg, halfWidthDeg,
                                        [&](int j, int i0, int i1) {
                                            addCraterSpan(crater, grid, j, i0, i1, northKm(j), kmPerDegLon[j], spacingKm[j], scale, heightsM);
                                        });
            }
        }
    }
}

}  // namespace

// -----------------------------------------------------------------------------

ChCraterLayer::ChCraterLayer(const ChPlanetBody& body, const Params& params) {
    const auto& d = params.diameters_km;
    if (d.size() != params.densities.size())
        throw std::invalid_argument("ChCraterLayer: diameters_km and densities differ in length");
    for (size_t i = 0; i < d.size(); ++i) {
        if (!(d[i] > 0) || (i > 0 && !(d[i] < d[i - 1])))
            throw std::invalid_argument("ChCraterLayer: diameters_km must be positive and strictly decreasing");
        if (!(params.densities[i] >= 0 && params.densities[i] <= 1))
            throw std::invalid_argument("ChCraterLayer: densities must lie in [0, 1]");
    }
    if (!d.empty() && !(params.floor_flattening_km > 0))
        throw std::invalid_argument("ChCraterLayer: floor_flattening_km must be positive");
    if (params.shallow_below_km > 0 &&
        !(params.shallow_at_km > 0 && params.shallow_at_km < params.shallow_below_km && params.shallow_depth_ratio > 0 && params.simple_depth_ratio > 0))
        throw std::invalid_argument("ChCraterLayer: shallow craters want 0 < shallow_at_km < shallow_below_km and positive depth ratios");

    auto model = std::make_unique<Model>();
    model->params = params;
    model->kmPerDeg = field::kmPerDeg(body.GetRadius());
    for (size_t i = 0; i < d.size(); ++i) {
        OctaveInfo o;
        o.diameterKm = d[i];
        o.density = params.densities[i];
        o.diameterDeg = d[i] / model->kmPerDeg;
        o.cellSizeDeg = o.diameterDeg * kCellDiams;
        o.invCellSizeDeg = 1.0 / o.cellSizeDeg;
        model->octaves.push_back(o);
    }
    m_model = std::move(model);
}

ChCraterLayer::~ChCraterLayer() = default;

const ChCraterLayer::Params& ChCraterLayer::GetParams() const {
    return m_model->params;
}

// Reach is at most kReachCells of a cell, so only adjacent cells can contribute.
std::vector<ChCraterInstance> ChCraterLayer::Query(double minLon, double minLat, double maxLon, double maxLat,
                                                   double minDiameter) const {
    const Model& m = *m_model;
    std::vector<ChCraterInstance> out;
    for (int octaveIndex = 0; octaveIndex < m.count(); ++octaveIndex) {
        const OctaveInfo& info = m.octave(octaveIndex);
        // Nominal radii span 0.55 to 1.0 of the class radius
        if (1000.0 * info.diameterKm < minDiameter)
            continue;
        const int gx0 = fastFloor(minLon * info.invCellSizeDeg), gx1 = fastFloor(maxLon * info.invCellSizeDeg);
        const int gy0 = fastFloor(minLat * info.invCellSizeDeg), gy1 = fastFloor(maxLat * info.invCellSizeDeg);
        for (int gy = gy0; gy <= gy1; ++gy) {
            for (int gx = gx0; gx <= gx1; ++gx) {
                const auto crater = CraterInstance::make(gx, gy, octaveIndex, m);
                if (!crater)
                    continue;
                if (crater->centerLonDeg < minLon || crater->centerLonDeg >= maxLon || crater->centerLatDeg < minLat ||
                    crater->centerLatDeg >= maxLat)
                    continue;
                ChCraterInstance c;
                c.lonDeg = crater->centerLonDeg;
                c.latDeg = crater->centerLatDeg;
                c.diameterM = 2000.0 * crater->nominalRadiusKm;
                c.depthM = 1000.0 * crater->depthKm;
                c.rimM = 1000.0 * crater->rimKm;
                c.isComplex = crater->isComplex;
                c.freshness = crater->preservationFactor;
                if (c.diameterM >= minDiameter)
                    out.push_back(c);
            }
        }
    }
    return out;
}

double ChCraterLayer::GetHeight(double lonDeg, double latDeg, double sampleSpacingDeg) const {
    const Model& m = *m_model;
    const double kKmPerDeg = m.kmPerDeg;
    const double invSampleSpacingDeg = 1.0 / std::max(sampleSpacingDeg, 1e-9);
    const double cosLat = util::cosLatClamped(latDeg);
    const double spacingKm = sampleSpacingDeg * kKmPerDeg * cosLat;
    double reliefM = 0.0;
    for (int octaveIndex = 0; octaveIndex < m.count(); ++octaveIndex) {
        const double fade = m.octaveFade(octaveIndex, invSampleSpacingDeg);
        if (fade <= 0.0) {
            break;
        }
        const double invCellSizeDeg = m.octave(octaveIndex).invCellSizeDeg;
        field::forEachCellNear(lonDeg * invCellSizeDeg, latDeg * invCellSizeDeg, kReachCells, [&](int gx, int gy) {
            const auto candidate = CraterInstance::make(gx, gy, octaveIndex, m);
            if (!candidate) {
                return;
            }
            const CraterInstance& crater = *candidate;
            const double dLon = util::wrapLongitude(lonDeg - crater.centerLonDeg);
            const double offsetEastKm = dLon * cosLat * kKmPerDeg, offsetNorthKm = (latDeg - crater.centerLatDeg) * kKmPerDeg;
            if (offsetEastKm * offsetEastKm + offsetNorthKm * offsetNorthKm < crater.reachKm * crater.reachKm) {
                reliefM += crater.reliefKmAtOffset(offsetEastKm, offsetNorthKm, spacingKm) * 1000.0 * fade;
            }
        });
    }
    return reliefM;
}

// Row bands in parallel over craters instantiated once for the whole grid.
void ChCraterLayer::AddToGrid(const ChGeoGrid& grid, std::vector<double>& heightsM) const {
    const Model& m = *m_model;
    if (m.octaves.empty()) {
        return;
    }
    const auto [originLonDeg, originLatDeg, stepLonDeg, stepLatDeg, sampleCount, sampleSpacingDeg] = grid;
    std::vector<double> kmPerDegLon(sampleCount), spacingKm(sampleCount);
    for (int j = 0; j < sampleCount; ++j) {
        const double cosLat = util::cosLatClamped(originLatDeg + j * stepLatDeg);
        kmPerDegLon[j] = cosLat * m.kmPerDeg;
        spacingKm[j] = sampleSpacingDeg * m.kmPerDeg * cosLat;
    }
    const auto craterOctaves = instantiate(m, originLonDeg, originLatDeg, originLonDeg + (sampleCount - 1) * stepLonDeg, originLatDeg + (sampleCount - 1) * stepLatDeg, 1.0 / std::max(sampleSpacingDeg, 1e-9));
    field::forEachRowBand(sampleCount, [&](field::RowRange rows) {
        addCraterRows(m, grid, rows, craterOctaves, kmPerDegLon.data(), spacingKm.data(), heightsM);
    });
}

}  // namespace planet
}  // namespace chrono
