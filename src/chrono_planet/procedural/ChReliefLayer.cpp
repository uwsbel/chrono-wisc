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

#include "chrono_planet/procedural/ChReliefLayer.h"

namespace chrono {
namespace planet {

ChReliefLayer::~ChReliefLayer() = default;

void ChReliefLayer::AddToGrid(const ChGeoGrid& grid, std::vector<double>& heights) const {
    // Per sample, through Apply: height + GetHeight.
    ChSurfaceFilter::ApplyGrid(grid, heights);
}

}  // namespace planet
}  // namespace chrono
