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

#include "chrono_sensor/sensors/radar/ChRadarMaterialRegistry.h"

#include <stdexcept>

namespace chrono {
namespace sensor {

namespace {

RadarMaterial Make(float specular, float diffuse, float lobe_width, float transmission) {
    RadarMaterial m;
    m.specular_reflectivity = specular;
    m.diffuse_reflectivity = diffuse;
    m.lobe_width = lobe_width;
    m.transmission = transmission;
    m.assigned = 1;
    return m;
}

// Representative responses near 77 GHz. Specular fractions follow the normal-incidence Fresnel
// power reflection of the bulk permittivity; the split into a coherent lobe and an incoherent
// one follows the surface roughness relative to the 3.9 mm wavelength. These are a starting
// sample set for a scene, not a measurement-backed material library.
const std::map<std::string, RadarMaterial>& SampleTable() {
    static const std::map<std::string, RadarMaterial> samples = {
        // Perfect conductor: all power into a near-mirror lobe.
        {"metal", Make(0.98f, 0.02f, 0.02f, 0.0f)},
        // Painted steel panel: conductor under a thin dielectric, gently curved.
        {"vehicle_body", Make(0.85f, 0.10f, 0.06f, 0.0f)},
        // Polypropylene bumper, relative permittivity near 2.25: mostly transparent, which is
        // what makes a radar behind a bumper workable.
        {"bumper", Make(0.04f, 0.02f, 0.08f, 0.85f)},
        // Automotive glass, relative permittivity near 6.
        {"glass", Make(0.17f, 0.04f, 0.05f, 0.60f)},
        // Dry asphalt, relative permittivity near 4.5, root mean square height comparable to the
        // wavelength: the coherent lobe nearly vanishes and the rest is the clutter floor.
        {"asphalt", Make(0.02f, 0.10f, 0.35f, 0.0f)},
        {"concrete", Make(0.05f, 0.12f, 0.25f, 0.0f)},
        // Skin and clothing, high water content, strongly curved.
        {"pedestrian", Make(0.05f, 0.25f, 0.60f, 0.0f)},
        {"wood", Make(0.03f, 0.04f, 0.30f, 0.50f)},
        // Foliage: no coherent return, a weak decorrelated one, and it lets signal through.
        {"vegetation", Make(0.00f, 0.05f, 1.00f, 0.50f)}};
    return samples;
}

}  // namespace

ChRadarMaterialRegistry::ChRadarMaterialRegistry() {
    // An unassigned fallback tells the shader to derive the response from the visual material's
    // metallic and roughness channels, so an untagged scene still behaves sensibly.
    m_fallback = {};
    m_fallback.assigned = 0;
}

RadarMaterial ChRadarMaterialRegistry::GetSample(const std::string& name) {
    const auto& samples = SampleTable();
    const auto it = samples.find(name);
    if (it == samples.end())
        throw std::invalid_argument("ChRadarMaterialRegistry: no sample material named '" + name + "'");
    return it->second;
}

std::vector<std::string> ChRadarMaterialRegistry::GetSampleNames() {
    std::vector<std::string> names;
    for (const auto& s : SampleTable())
        names.push_back(s.first);
    return names;
}

void ChRadarMaterialRegistry::Assign(unsigned short class_id, const RadarMaterial& material) {
    RadarMaterial m = material;
    m.assigned = 1;
    m_assignments[class_id] = m;
}

void ChRadarMaterialRegistry::Assign(unsigned short class_id, const std::string& sample_name) {
    Assign(class_id, GetSample(sample_name));
}

std::vector<RadarMaterial> ChRadarMaterialRegistry::BuildTable() const {
    unsigned int size = 1;
    if (!m_assignments.empty())
        size = static_cast<unsigned int>(m_assignments.rbegin()->first) + 1;

    RadarMaterial unassigned = {};
    unassigned.assigned = 0;
    std::vector<RadarMaterial> table(size, unassigned);
    for (const auto& entry : m_assignments)
        table[entry.first] = entry.second;
    return table;
}

}  // namespace sensor
}  // namespace chrono
