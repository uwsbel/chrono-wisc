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
//
// Cutting rectangles of site x/y out of terrain meshes, where other ground,
// such as a work site modeled on its own, takes over.
//
// =============================================================================

#ifndef CH_SITE_HOLES_H
#define CH_SITE_HOLES_H

#include <vector>

#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChSiteFrame.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Remove the parts of a mesh's faces over rectangles of x/y, cutting the faces that cross a rectangle's edges
/// along them. New vertices take normals, UVs and colors interpolated across their face, and new faces the
/// material of the face they come from. Faces wholly outside every rectangle are left as they are. Returns the
/// number of faces removed or cut.
CH_PLANET_API size_t CutSiteHoles(ChTriangleMeshConnected& mesh, const std::vector<ChSiteRegion>& holes);

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
