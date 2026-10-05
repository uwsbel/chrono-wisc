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

#include <cmath>

#include "chrono_planet/core/Parallel.h"
#include "chrono_planet/core/SphereMath.h"
#include "chrono_planet/procedural/ChBaseReliefLayer.h"
#include "chrono_planet/core/Perlin.h"

namespace chrono {
namespace planet {

ChBaseReliefLayer::ChBaseReliefLayer(const Params& params) : m_params(params) {}

double ChBaseReliefLayer::GetHeight(double lon_deg, double lat_deg, double) const {
    const float noise = Perlin::onSphere(util::dirFromLonLat(lon_deg, lat_deg), static_cast<float>(m_params.frequency));
    return m_params.amplitude * noise;
}

void ChBaseReliefLayer::AddToGrid(const ChGeoGrid& grid, std::vector<double>& heights) const {
    const int n = grid.n;
    const float frequency = static_cast<float>(m_params.frequency);
    std::vector<double> cosLon(n), sinLon(n);
    for (int i = 0; i < n; ++i) {
        const double lam = util::deg2rad(util::wrapLongitude(grid.lon0 + i * grid.step_lon));
        cosLon[i] = std::cos(lam);
        sinLon[i] = std::sin(lam);
    }
    const bool par = util::parallelGrid(static_cast<size_t>(n) * n);
    // Rows are independent. Each thread keeps its own row scratch.
#pragma omp parallel num_threads(util::parallelThreads()) if (par)
    {
        std::vector<util::Vec3> dirs(n);
        std::vector<float> noise(n), scratch;
#pragma omp for schedule(static)
        for (int j = 0; j < n; ++j) {
            const double phi = util::deg2rad(grid.lat0 + j * grid.step_lat);
            const double cp = std::cos(phi), sp = std::sin(phi);
            for (int i = 0; i < n; ++i) {
                dirs[i] = {cp * cosLon[i], cp * sinLon[i], sp};  // == dirFromLonLat(lon_i, lat_j)
            }
            Perlin::writeSphereNoiseRow(dirs.data(), n, frequency, noise.data(), scratch);
            double* row = &heights[static_cast<size_t>(j) * n];
            for (int i = 0; i < n; ++i) {
                row[i] += m_params.amplitude * noise[i];
            }
        }
    }
}

}  // namespace planet
}  // namespace chrono
