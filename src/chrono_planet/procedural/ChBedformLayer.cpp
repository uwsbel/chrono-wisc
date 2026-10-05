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
#include <cmath>
#include <stdexcept>

#include "chrono_planet/core/FieldHash.h"
#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/core/Perlin.h"
#include "chrono_planet/procedural/ChBedformLayer.h"
#include "chrono_planet/procedural/FieldGrid.h"

namespace chrono {
namespace planet {

using field::hash01;
using field::seeded;

namespace {

// One candidate field per lattice cell. A field's radius is kRadiusMin to kRadiusMax cells and its ragged
// edge reaches kRagged farther, so a field stays within kReachCells of its cell.
constexpr double kRadiusMin = 0.40, kRadiusMax = 0.70, kRagged = 0.30;
constexpr double kReachCells = 1.0;
// Mean field area in cells^2, pi E[r^2] for a radius uniform between the two.
constexpr double kMeanAreaCells =
    util::kPi * (kRadiusMax * kRadiusMax * kRadiusMax - kRadiusMin * kRadiusMin * kRadiusMin) / (3.0 * (kRadiusMax - kRadiusMin));
// A field thins to nothing over this outer share of its radius.
constexpr double kEdgeFrac = 0.35;
// A field's wavelength, as a multiple of the layer's.
constexpr double kWavelengthMin = 0.8, kWavelengthSpan = 0.4;
// Crests wander over this many wavelengths along their length, and the edge's rags are this many radii wide.
constexpr double kWanderWavelengths = 5.0, kRagRadii = 0.5;
// Fade bounds, in samples across a wavelength.
constexpr double kFadeLoVerts = 2.0, kFadeHiVerts = 4.0;

}  // namespace

struct ChBedformLayer::Model {
    Params params;
    double mPerDeg;
    double cellM, cellDeg, invCellDeg;
    double density;  // probability a cell holds a field

    float hash(int gx, int gy, int salt) const { return hash01(gx, gy, seeded(salt, params.seed)); }

    // The fields near a point: visit(cover, relief) for each, its cover in [0, 1] and its relief (m) there.
    template <class Visit>
    void forEachField(double lonDeg, double latDeg, Visit&& visit) const {
        field::forEachCellNear(lonDeg * invCellDeg, latDeg * invCellDeg, kReachCells, [&](int gx, int gy) {
            // Cells are square in degrees, so the per-area density scales by cos(lat).
            const double cellLat = (gy + 0.5) * cellDeg;
            const double cosLat = std::max(std::cos(util::deg2rad(cellLat)), 0.02);
            if (hash(gx, gy, 301) > density * cosLat) {
                return;
            }
            // Meters east and north of the field's center, on its own tangent plane.
            const double centerLon = (gx + hash(gx, gy, 302)) * cellDeg, centerLat = (gy + hash(gx, gy, 303)) * cellDeg;
            if (params.bedrock && hash(gx, gy, 310) < params.bedrock->GetExposureNear(centerLon, centerLat)) {
                return;
            }
            double dLon = lonDeg - centerLon;
            if (dLon > 180.0) {
                dLon -= 360.0;
            } else if (dLon < -180.0) {
                dLon += 360.0;
            }
            const double x = dLon * std::cos(util::deg2rad(centerLat)) * mPerDeg, y = (latDeg - centerLat) * mPerDeg;
            // Each field reads its own patch of the noise.
            const float ox = 97.f * hash(gx, gy, 304), oy = 97.f * hash(gx, gy, 305);

            // ponytail: the lattice is square in degrees, so fields shrink toward the poles with their cells. A lattice
            // in meters if polar sites need full-size fields.
            const double radius = (kRadiusMin + (kRadiusMax - kRadiusMin) * hash(gx, gy, 306)) * cellM * cosLat;
            const double rag = Perlin::noise(static_cast<float>(x / (kRagRadii * radius)) + ox, static_cast<float>(y / (kRagRadii * radius)) + oy);
            const double inside = 1.0 - std::sqrt(x * x + y * y) / (radius * (1.0 + kRagged * rag));
            if (inside <= 0.0) {
                return;
            }
            const double cover = util::smoothstep01(std::min(1.0, inside / kEdgeFrac));

            const double scale = kWavelengthMin + kWavelengthSpan * hash(gx, gy, 307);
            const double wavelength = params.wavelength * scale;
            const double azimuth = util::deg2rad(params.wind_azimuth + params.azimuth_spread * (2.0 * hash(gx, gy, 308) - 1.0));
            const double wander = Perlin::noise(static_cast<float>(x / (kWanderWavelengths * wavelength)) + oy,
                                                static_cast<float>(y / (kWanderWavelengths * wavelength)) + ox);
            const double along = (x * std::sin(azimuth) + y * std::cos(azimuth)) / wavelength + params.sinuosity * wander + hash(gx, gy, 309);
            // Downwind across one wavelength: up the upwind face, then down the lee. C1, 0 in the troughs.
            const double t = along - std::floor(along), upwind = 1.0 - params.lee_fraction;
            const double profile = t < upwind ? 0.5 * (1.0 - std::cos(util::kPi * t / upwind))
                                              : 0.5 * (1.0 + std::cos(util::kPi * (t - upwind) / params.lee_fraction));
            visit(cover, cover * params.height * scale * profile);
        });
    }
};

ChBedformLayer::ChBedformLayer(const ChPlanetBody& body, const Params& params) {
    if (!(params.wavelength > 0) || params.height < 0 || !(params.field_size > 0))
        throw std::invalid_argument("ChBedformLayer: wavelength and field_size must be positive, height not negative");
    if (!(params.lee_fraction > 0 && params.lee_fraction < 1))
        throw std::invalid_argument("ChBedformLayer: lee_fraction must lie in (0, 1)");
    if (params.cover < 0 || params.cover > 0.9)
        throw std::invalid_argument("ChBedformLayer: cover must lie in [0, 0.9]");

    auto model = std::make_unique<Model>();
    model->params = params;
    model->mPerDeg = field::kmPerDeg(body.GetRadius()) * 1000.0;
    model->cellM = params.field_size / (kRadiusMin + kRadiusMax);
    model->cellDeg = model->cellM / model->mPerDeg;
    model->invCellDeg = 1.0 / model->cellDeg;
    // Fields overlap, so the ground they cover together is 1 - exp(-density * area).
    model->density = std::min(1.0, -std::log(1.0 - params.cover) / kMeanAreaCells);
    m_model = std::move(model);
}

ChBedformLayer::~ChBedformLayer() = default;

const ChBedformLayer::Params& ChBedformLayer::GetParams() const {
    return m_model->params;
}

double ChBedformLayer::GetCover(double lon_deg, double lat_deg) const {
    if (m_model->density <= 0.0) {
        return 0.0;
    }
    double total = 0.0;
    m_model->forEachField(lon_deg, lat_deg, [&](double cover, double) { total += cover; });
    return std::min(1.0, total);
}

double ChBedformLayer::GetHeight(double lon_deg, double lat_deg, double spacing_deg) const {
    const Model& m = *m_model;
    if (m.density <= 0.0) {
        return 0.0;
    }
    const double fade = field::resolutionFade(m.params.wavelength / (m.mPerDeg * std::max(spacing_deg, 1e-12)), kFadeLoVerts, kFadeHiVerts);
    if (fade <= 0.0) {
        return 0.0;
    }
    // Where fields overlap their sand is shared out, not piled: the relief is their mean, weighted by cover.
    double total = 0.0, relief = 0.0;
    m.forEachField(lon_deg, lat_deg, [&](double cover, double height) {
        total += cover;
        relief += height;
    });
    return fade * relief / std::max(1.0, total);
}

}  // namespace planet
}  // namespace chrono
