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
// Authors: Patrick Chen
// =============================================================================
//
// Electromagnetic properties of scene surfaces at radar wavelengths.
//
// =============================================================================

#ifndef CHRADARMATERIALREGISTRY_H
#define CHRADARMATERIALREGISTRY_H

#include <map>
#include <string>
#include <vector>

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/sensors/radar/ChRadarTypes.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Table of surface responses at the radar carrier, keyed by visual material class id.
///
/// Core Chrono's ChVisualMaterial carries no electromagnetic parameters, and radar has no
/// business adding any: the registry keys off the class id that already reaches the shaders, so
/// a scene declares its radar properties by tagging its visual materials. Class ids without an
/// entry fall back to a response derived from the material's metallic and roughness channels,
/// which keeps an untagged scene usable while making a tagged one exact.
class CH_SENSOR_API ChRadarMaterialRegistry {
  public:
    ChRadarMaterialRegistry();

    /// Look up one of the named sample materials. Throws std::invalid_argument if unknown.
    /// Available: "metal", "vehicle_body", "bumper", "glass", "asphalt", "concrete",
    /// "pedestrian", "wood", "vegetation".
    static RadarMaterial GetSample(const std::string& name);

    /// Names of all sample materials.
    static std::vector<std::string> GetSampleNames();

    /// Bind a visual material class id to a radar response.
    void Assign(unsigned short class_id, const RadarMaterial& material);

    /// Bind a visual material class id to one of the named samples.
    void Assign(unsigned short class_id, const std::string& sample_name);

    /// Response used where no class id matches. Left unset, the shader derives one from the
    /// visual material's metallic and roughness channels instead.
    void SetFallback(const RadarMaterial& material) {
        m_fallback = material;
        m_fallback.assigned = 1;
    }
    const RadarMaterial& GetFallback() const { return m_fallback; }

    /// Dense table indexed by class id, ready for upload. Entries with no assignment carry
    /// assigned == 0.
    std::vector<RadarMaterial> BuildTable() const;

  private:
    std::map<unsigned short, RadarMaterial> m_assignments;
    RadarMaterial m_fallback;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
