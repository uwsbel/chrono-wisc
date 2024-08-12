// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_sensor/optix/scene/ChScene.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChScene::ChScene() {
    m_background.mode = BackgroundMode::GRADIENT;
    m_background.color_zenith = {0.4f, 0.5f, 0.6f};
    m_background.color_horizon = {0.7f, 0.8f, 0.9f};
    m_background.env_tex = "";

    m_ambient_light = ChVector3f({.2f, .2f, .2f});
    //m_pointlights = std::vector<PointLight>();
    //m_arealights = std::vector<AreaLight>();
    m_lights = std::vector<Light>();
    m_num_pointlights = 0;
    m_num_arealights = 0;

    lights_changed = true;
    background_changed = true;

    m_fog_color = ChVector3f(1.f, 1.f, 1.f);
    m_fog_scattering = 0.f;
    

    m_scene_epsilon = 1e-3f;
    m_dynamic_origin_threshold = 100.f;
    m_dynamic_origin_offset = false;
    m_sprites = std::vector <std::shared_ptr<ChBody>>();



    //m_nvdb = nullptr;
}

CH_SENSOR_API ChScene::~ChScene() {}

CH_SENSOR_API unsigned int ChScene::AddPointLight(ChVector3f pos, ChColor color, float max_range) {
    PointLight p({pos.x(), pos.y(), pos.z()}, {color.R, color.G, color.B}, max_range);
  /*  p.pos = {pos.x(), pos.y(), pos.z()};
    p.color = {color.R, color.G, color.B};
    p.max_range = max_range;*/
    //m_pointlights.push_back(p);
    m_lights.push_back(p);
    lights_changed = true,
    m_num_pointlights++;
    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API unsigned int ChScene::AddAreaLight(ChVector3f pos, ChColor color, float max_range, ChVector3f du, ChVector3f dv) {
    AreaLight a;

    a.pos = {pos.x(), pos.y(), pos.z()};
    a.du = {du.x(), du.y(), du.z()};
    a.dv = {dv.x(), dv.y(), dv.z()};

    a.color = {color.R, color.G, color.B};
    a.max_range = max_range;
    lights_changed = true;
    //m_arealights.push_back(a);
    m_num_arealights++;
    m_lights.push_back(a);

    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API unsigned int ChScene::AddPointLight(const PointLight& p) {
    //m_pointlights.push_back(p);
    m_lights.push_back(p);
    lights_changed = true;
    m_num_pointlights++;
    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API unsigned int ChScene::AddSpotLight(ChVector3f pos,
    ChVector3f to,
    ChColor color,
    float max_range,
    float total_width,
    float falloff_start) {

    Light spot;
    spot.type = LightType::SPOT_LIGHT;
    spot.pos = {pos.x(), pos.y(), pos.z()};
    spot.to = {to.x(), to.y(), to.z()};
    spot.color = {color.R, color.G, color.B};
    spot.max_range = max_range;
    spot.cos_total_width = cosf(total_width);
    spot.cos_falloff_start = cosf(falloff_start);

    lights_changed = true;
    m_lights.push_back(spot);

    return static_cast<unsigned int>(m_lights.size() - 1);
}

// TODO: Add a Modify Light function
// CH_SENSOR_API void ChScene::ModifyPointLight(unsigned int id, PointLight p) {
//     if (id <= m_pointlights.size()) {
//         m_pointlights[id] = p;
//         lights_changed = true;
//     }
// }

CH_SENSOR_API void ChScene::SetBackground(Background b) {
    m_background = b;
    background_changed = true;
}

CH_SENSOR_API void ChScene::SetSceneEpsilon(float e) {
    m_scene_epsilon = e;
    background_changed = true;
}

/// Function to set the fog color
CH_SENSOR_API void ChScene::SetFogColor(ChVector3f color) {
    m_fog_color = ChClamp(color, ChVector3f(0.f, 0.f, 0.f), ChVector3f(1.f, 1.f, 1.f));
    background_changed = true;
}

/// Function to set the fog scattering coefficient
CH_SENSOR_API void ChScene::SetFogScattering(float coefficient) {
    m_fog_scattering = ChClamp(coefficient, 0.f, 1.f);
    background_changed = true;
}

/// Function to set the fog scattering coefficient
CH_SENSOR_API void ChScene::SetFogScatteringFromDistance(float distance) {
    distance = ChClamp(distance, 1e-3f, 1e16f);
    m_fog_scattering = log(256.0) / distance;
    background_changed = true;
}


CH_SENSOR_API void ChScene::AddSprite(std::shared_ptr<ChBody> sprite) {
    m_sprites.push_back(sprite);
}


void ChScene::UpdateOriginOffset(ChVector3f sensor_pos, bool force) {
    if (force || (m_dynamic_origin_offset && (sensor_pos - m_origin_offset).Length() > m_dynamic_origin_threshold)) {
        // set the new origin offset
        m_origin_offset = sensor_pos;
        m_origin_changed = true;
    }
}


}  // namespace sensor
}  // namespace chrono
