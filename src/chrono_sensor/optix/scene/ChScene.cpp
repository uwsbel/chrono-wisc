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

    ChVector3f dir = {to.x() - pos.x(), to.y() - pos.y(), to.z() - pos.z()};
    dir = dir / dir.Length();
    //std::cout << "Dir: " << dir << std::endl;
    spot.spot_dir = {dir.x(), dir.y(), dir.z()};
    spot.color = {color.R, color.G, color.B};
    spot.max_range = max_range;
    spot.cos_total_width = cosf(total_width);
    spot.cos_falloff_start = cosf(falloff_start);

    lights_changed = true;
    m_lights.push_back(spot);

    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API unsigned int ChScene::AddSpotLight(std::shared_ptr<chrono::ChBody> parent,
    ChFramed offsetPose,
    ChColor color,
    float max_range,
    float total_width,
    float falloff_start) {

    Light spot;
    spot.type = LightType::SPOT_LIGHT;
    ChFramed global_frame = parent->GetVisualModelFrame() * offsetPose;
    ChVector3f frame_pos = global_frame.GetPos() - GetOriginOffset();
    ChVector3f local_dir(1, 0, 0);
    ChVector3f world_dir = global_frame.GetRot().Rotate(local_dir);
    spot.pos = make_float3(frame_pos.x(), frame_pos.y(), frame_pos.z());
    spot.spot_dir = make_float3(world_dir.x(), world_dir.y(), world_dir.z());

    spot.color = {color.R, color.G, color.B};
    spot.max_range = max_range;
    spot.cos_total_width = cosf(total_width);
    spot.cos_falloff_start = cosf(falloff_start);

    spot.parent_id = parent->GetIdentifier();
    lights_changed = true;
    m_lights.push_back(spot);

    unsigned int id = static_cast<unsigned int>(m_lights.size() - 1);
    m_light_frames.insert({id,offsetPose});
    m_light_parent.insert({id,parent});
    return id;
}


CH_SENSOR_API unsigned int ChScene::AddSpotLight(ChFramed offsetPose,
                                                 ChColor color,
                                                 float max_range,
                                                 float total_width,
                                                 float falloff_start) {
    Light spot;
    spot.type = LightType::SPOT_LIGHT;
    ChVector3f frame_pos = offsetPose.GetPos();
    ChVector3f local_dir(1, 0, 0);
    ChVector3f world_dir = offsetPose.GetRot().Rotate(local_dir);
    spot.pos = make_float3(frame_pos.x(), frame_pos.y(), frame_pos.z());
    spot.spot_dir = make_float3(world_dir.x(), world_dir.y(), world_dir.z());

    spot.color = {color.R, color.G, color.B};
    spot.max_range = max_range;
    spot.cos_total_width = cosf(total_width);
    spot.cos_falloff_start = cosf(falloff_start);

    lights_changed = true;
    m_lights.push_back(spot);

    return static_cast<unsigned int>(m_lights.size() - 1);
}

/// interpolate between two vectors
ChVector3f ChLerp(ChVector3f& a, ChVector3f& b, double x) {
    return a + (b - a) * x;
}

CH_SENSOR_API void ChScene::UpdateLight(unsigned int id, ChFramed newpose) {
    Light* light = &m_lights[id];
    switch (light->type) {
        case LightType::SPOT_LIGHT: {
            if (light->parent_id > 0) { // If light has a parent
                ChFramed local_frame = GetLightFrame(id);
                ChFramed parent_frame = GetLightParentFrame(id);
                ChFramed glob_frame = parent_frame * local_frame;

                ChVector3f glob_pos = glob_frame.GetPos();
                ChVector3f local_dir{1, 0, 0};
                ChVector3f world_dir = glob_frame.GetRot().Rotate(local_dir);
                ChVector3f prev_pos = ChVector3f(light->pos.x, light->pos.y, light->pos.z);
                ChVector3f prev_dir = ChVector3f(light->spot_dir.x, light->spot_dir.y, light->spot_dir.z);

                float interpolation_factor = 0.4;
                ChVector3f interp_pos = glob_pos +  interpolation_factor * (glob_pos - prev_pos);
                ChVector3f interp_dir = world_dir + interpolation_factor * (world_dir - prev_dir);

                light->pos = make_float3(interp_pos.x(), interp_pos.y(), interp_pos.z());
                light->spot_dir = make_float3(interp_dir.x(), interp_dir.y(), interp_dir.z());
            } else { // Not attached to parent just change pose
                //std::cout << "Updating Pose of Spot Light" << std::endl;
                ChVector3f glob_pos = newpose.GetPos();
                ChVector3f local_dir{1, 0, 0};
                ChVector3f world_dir = newpose.GetRot().Rotate(local_dir);
                light->pos = make_float3(glob_pos.x(), glob_pos.y(), glob_pos.z());
                light->spot_dir = make_float3(world_dir.x(), world_dir.y(), world_dir.z());
            }
            break;
        }
        case LightType::AREA_LIGHT: {
            break;
        }
        case LightType::POINT_LIGHT: {
            ChVector3f glob_pos = newpose.GetPos();
            light->pos = make_float3(glob_pos.x(), glob_pos.y(), glob_pos.z());
            break;
        }
        default:
            break;
    }
    lights_changed = true;
}

// TODO: Add a Modify Light function
CH_SENSOR_API void ChScene::ModifyPointLight(unsigned int id, PointLight p) {
    if (id <= m_lights.size()) {
        m_lights[id] = p;
        lights_changed = true;
    }
}

CH_SENSOR_API void ChScene::SetBackground(Background b) {
    m_background = b;
    background_changed = true;
}

CH_SENSOR_API void ChScene::SetSceneEpsilon(float epsilon) {
    m_scene_epsilon = epsilon;
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
