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
// Glue between a ChDustField and the demos: wheel states from wheel bodies,
// and the dust grid as a participating medium for the Vulkan RT cameras.
//
// =============================================================================

#ifndef PLANET_DUST_H
#define PLANET_DUST_H

#include <memory>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/terrain/ChDustField.h"

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/ChSensorManager.h"

/// State of a wheel body spinning about the y axis of its reference frame, as PlanetSCMTerrain has it, for
/// ChDustField::EmitFromWheels. (The center of mass frame of a ChBodyAuxRef can be turned from it.)
inline chrono::vehicle::ChDustField::WheelState DustWheelState(const std::shared_ptr<chrono::ChBody>& body, double radius, double width) {
    const auto& frame = body->GetFrameRefToAbs();
    const chrono::ChVector3d axle = frame.GetRot().GetAxisY();
    return {frame.GetPos(), axle, frame.GetPosDt(), body->GetAngVelParent().Dot(axle), radius, width};
}

/// Hand the dust grid to the manager's cameras, scattering light as grains of the given Hapke parameters and
/// albedo tint do. Returns false if the render backend draws no participating medium (only Vulkan RT does).
inline bool SetDustVolume(chrono::sensor::ChSensorManager& manager,
                          const chrono::vehicle::ChDustField& dust,
                          float hapke_w,
                          float hapke_b,
                          float hapke_c,
                          const chrono::ChColor& tint) {
#ifdef CHRONO_HAS_VULKAN_RT
    const auto& g = dust.GetGrid();
    auto volume = std::make_shared<chrono::sensor::ChVulkanRTVolume>();
    volume->origin = chrono::ChVector3f(g.origin);
    volume->voxel = static_cast<float>(g.voxel);
    volume->nx = g.nx;
    volume->ny = g.ny;
    volume->nz = g.nz;
    volume->extinction = g.extinction;
    volume->sun_transmittance = g.sun_transmittance;
    volume->sun_visibility = g.sun_visibility;
    volume->sun_dir = chrono::ChVector3f(g.sun_dir);
    volume->albedo = hapke_w;
    volume->phase_b = hapke_b;
    volume->phase_c = hapke_c;
    volume->color = chrono::ChVector3f(tint.R, tint.G, tint.B);
    manager.vulkan_scene->SetVolume(volume);
    return true;
#else
    return false;
#endif
}

#endif
