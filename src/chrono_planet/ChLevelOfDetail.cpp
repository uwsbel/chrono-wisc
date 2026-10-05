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

#include "chrono_planet/ChLevelOfDetail.h"

namespace chrono {
namespace planet {

ChLevelOfDetail::ChLevelOfDetail(double root_tile_deg) : m_root_tile_deg(root_tile_deg) {
    if (!(root_tile_deg > 0) || root_tile_deg > 180)
        throw std::invalid_argument("ChLevelOfDetail: the root tile size must lie in (0, 180] degrees");
}

int ChLevelOfDetail::GetDivisions(int zoom) {
    return std::min(1 << (zoom + 1), kMaxDivisions);
}

double ChLevelOfDetail::GetVertexSpacing(int zoom, double root_tile_deg) {
    return std::ldexp(root_tile_deg, -std::max(0, zoom)) / GetDivisions(std::max(0, zoom));
}

}  // namespace planet
}  // namespace chrono
