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
    m_lights = std::vector<ChOptixLight>();

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

/// Point light ///
CH_SENSOR_API unsigned int ChScene::AddPointLight(ChVector3f pos, ChColor color, float max_range, bool const_color) {
    ChOptixLight light{};   // zero-initialize everything
    light.light_type  = LightType::POINT_LIGHT;
    light.pos = {pos.x(), pos.y(), pos.z()};
    light.delta = true;
    light.specific.point.color = {color.R, color.G, color.B};
    light.specific.point.max_range = max_range;
    light.specific.point.const_color = const_color;
    
    // Calculate extended parameters
    light.specific.point.atten_scale = (max_range > 0) ? (0.01 * max_range * max_range) : 1.f;

    m_lights.push_back(light);
    lights_changed = true;

    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API void ChScene::ModifyPointLight(unsigned int id, const ChOptixLight& point_light) {
    if (id <= m_lights.size() && point_light.light_type == LightType::POINT_LIGHT) {
        m_lights[id] = point_light;
        lights_changed = true;
    }
}

/// ---- Directional light ---- ///

CH_SENSOR_API unsigned int ChScene::AddDirectionalLight(ChColor color, float elevation, float azimuth) {
    ChOptixLight light{};   // zero-initialize everything
    light.light_type  = LightType::DIRECTIONAL_LIGHT;
    light.pos = {0.f, 0.f, 0.f};
    light.delta = true;
    light.specific.directional.color = {color.R, color.G, color.B};
    light.specific.directional.elevation = elevation;
    light.specific.directional.azimuth = azimuth;

    // Calculate extended parameters
    light.specific.directional.light_dir = {cosf(elevation) * cosf(azimuth), cosf(elevation) * sinf(azimuth), sinf(elevation)};

    m_lights.push_back(light);
    lights_changed = true;

    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API void ChScene::ModifyDirectionalLight(unsigned int id, const ChOptixLight& directional_light) {
    if (id <= m_lights.size() && directional_light.light_type == LightType::DIRECTIONAL_LIGHT) {
        m_lights[id] = directional_light;
        lights_changed = true;
    }
}

/// ---- Spot light ---- ///

CH_SENSOR_API unsigned int ChScene::AddSpotLight(
    ChVector3f pos, ChColor color, float max_range, ChVector3f light_dir, float angle_falloff_start, float angle_range, bool const_color
) {
    ChOptixLight light{};   // zero-initialize everything
    light.light_type  = LightType::SPOT_LIGHT;
    light.pos = make_float3(pos.x(), pos.y(), pos.z());
    light.delta = true;
    light.specific.spot.color = make_float3(color.R, color.G, color.B);
    light.specific.spot.max_range = max_range;
    ChVector3f nrmlz_light_dir = light_dir.GetNormalized();
    light.specific.spot.light_dir = make_float3(nrmlz_light_dir.x(), nrmlz_light_dir.y(), nrmlz_light_dir.z());
    light.specific.spot.const_color = const_color;
    light.specific.spot.angle_range = angle_range;
    if (angle_falloff_start < angle_range - 1e-6f) {
        light.specific.spot.angle_falloff_start = angle_falloff_start;
        light.specific.spot.angle_atten_rate = 1.f / (angle_range - angle_falloff_start);
    }
    // ineffective value of angle_falloff_start, set to no angular attenuation
    else {
        light.specific.spot.angle_falloff_start = angle_range;
        light.specific.spot.angle_atten_rate = -1.f;
    }

    // Calculate extended parameters
    light.specific.spot.atten_scale = (max_range > 0) ? (0.01 * max_range * max_range) : 1.f;

    m_lights.push_back(light);
    lights_changed = true;

    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API void ChScene::ModifySpotLight(unsigned int id, const ChOptixLight& spot_light) {
    if (id <= m_lights.size() && spot_light.light_type == LightType::SPOT_LIGHT) {
        m_lights[id] = spot_light;
        lights_changed = true;
    }
}

/// ---- Rectangle light ---- ///

CH_SENSOR_API unsigned int ChScene::AddRectangleLight(
    ChVector3f pos, ChColor color, float max_range, ChVector3f length_vec, ChVector3f width_vec, bool const_color
) {
    ChOptixLight light{};   // zero-initialize everything
    light.light_type  = LightType::RECTANGLE_LIGHT;
    light.pos = make_float3(pos.x(), pos.y(), pos.z());
    light.delta = false;
    light.specific.rectangle.color = {color.R, color.G, color.B};
    light.specific.rectangle.max_range = max_range;
    light.specific.rectangle.length_vec = make_float3(length_vec.x(), length_vec.y(), length_vec.z());
    light.specific.rectangle.width_vec = make_float3(width_vec.x(), width_vec.y(), width_vec.z());
    light.specific.rectangle.const_color = const_color;

    // Calculate extended parameters
    light.specific.rectangle.atten_scale = (max_range > 0) ? (0.01 * max_range * max_range) : 1.f;
    ChVector3f light_dir = Vcross(length_vec, width_vec);
    light.specific.rectangle.area = light_dir.Length();
    light_dir = light_dir / light_dir.Length();
    light.specific.rectangle.light_dir = make_float3(light_dir.x(), light_dir.y(), light_dir.z());
    
    m_lights.push_back(light);
    lights_changed = true;

    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API void ChScene::ModifyRectangleLight(unsigned int id, const ChOptixLight& rectangle_light) {
    if (id <= m_lights.size() && rectangle_light.light_type == LightType::RECTANGLE_LIGHT) {
        m_lights[id] = rectangle_light;
        lights_changed = true;
    }
}

/// ---- Disk light ---- ///

CH_SENSOR_API unsigned int ChScene::AddDiskLight(
   ChVector3f pos, ChColor color, float max_range, ChVector3f light_dir, float radius, bool const_color
) {
    ChOptixLight light{};   // zero-initialize everything
    light.light_type  = LightType::DISK_LIGHT;
    light.pos = make_float3(pos.x(), pos.y(), pos.z());
    light.delta = false;
    light.specific.disk.color = {color.R, color.G, color.B};
    light.specific.disk.max_range = max_range;
    ChVector3f nrmlz_light_dir = light_dir.GetNormalized();
    light.specific.disk.light_dir = make_float3(nrmlz_light_dir.x(), nrmlz_light_dir.y(), nrmlz_light_dir.z());
    light.specific.disk.radius = radius;
    light.specific.disk.const_color = const_color;

    // Calculate extended parameters
    light.specific.disk.atten_scale = (max_range > 0) ? (0.01 * max_range * max_range) : 1.f;
    light.specific.disk.area = CH_PI * radius * radius;
    
    m_lights.push_back(light);
    lights_changed = true;

    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API void ChScene::ModifyDiskLight(unsigned int id, const ChOptixLight& disk_light) {
    if (id <= m_lights.size() && disk_light.light_type == LightType::DISK_LIGHT) {
        m_lights[id] = disk_light;
        lights_changed = true;
    }
}

//// ---- Register Your Customized Light Types Here (AddLight and ModifyLight functions) ---- ////


// CH_SENSOR_API unsigned int ChScene::AddAreaLight(
//     ChVector3f pos, ChColor color, float max_range, ChVector3f du, ChVector3f dv) {
//     AreaLight a;

//     a.pos = {pos.x(), pos.y(), pos.z()};
//     a.du = {du.x(), du.y(), du.z()};
//     a.dv = {dv.x(), dv.y(), dv.z()};

//     a.color = {color.R, color.G, color.B};
//     a.max_range = max_range;
//     lights_changed = true;
//     //m_arealights.push_back(a);
//     m_num_arealights++;
//     m_lights.push_back(a);

//     return static_cast<unsigned int>(m_lights.size() - 1);
// }


// CH_SENSOR_API unsigned int ChScene::AddPointLight(const PointLight& p) {
//     //m_pointlights.push_back(p);
//     m_lights.push_back(p);
//     lights_changed = true;
//     m_num_pointlights++;
//     return static_cast<unsigned int>(m_lights.size() - 1);
// }



// CH_SENSOR_API unsigned int ChScene::AddSpotLight(ChVector3f pos,
//     ChVector3f to,
//     ChColor color,
//     float max_range,
//     float total_width,
//     float falloff_start) {

//     Light spot;
//     spot.type = LightType::SPOT_LIGHT;
//     spot.pos = {pos.x(), pos.y(), pos.z()};

//     ChVector3f dir = {to.x() - pos.x(), to.y() - pos.y(), to.z() - pos.z()};
//     dir = dir / dir.Length();
//     //std::cout << "Dir: " << dir << std::endl;
//     spot.spot_dir = {dir.x(), dir.y(), dir.z()};
//     spot.color = {color.R, color.G, color.B};
//     spot.max_range = max_range;
//     spot.cos_total_width = cosf(total_width);
//     spot.cos_falloff_start = cosf(falloff_start);

//     lights_changed = true;
//     m_lights.push_back(spot);

//     return static_cast<unsigned int>(m_lights.size() - 1);
// }

// CH_SENSOR_API unsigned int ChScene::AddSpotLight(std::shared_ptr<chrono::ChBody> parent,
//     ChFramed offsetPose,
//     ChColor color,
//     float max_range,
//     float total_width,
//     float falloff_start) {

//     Light spot;
//     spot.type = LightType::SPOT_LIGHT;
//     ChFramed global_frame = parent->GetVisualModelFrame() * offsetPose;
//     ChVector3f frame_pos = global_frame.GetPos() - GetOriginOffset();
//     ChVector3f local_dir(1, 0, 0);
//     ChVector3f world_dir = global_frame.GetRot().Rotate(local_dir);
//     spot.pos = make_float3(frame_pos.x(), frame_pos.y(), frame_pos.z());
//     spot.spot_dir = make_float3(world_dir.x(), world_dir.y(), world_dir.z());

//     spot.color = {color.R, color.G, color.B};
//     spot.max_range = max_range;
//     spot.cos_total_width = cosf(total_width);
//     spot.cos_falloff_start = cosf(falloff_start);

//     spot.parent_id = parent->GetIdentifier();
//     lights_changed = true;
//     m_lights.push_back(spot);

//     unsigned int id = static_cast<unsigned int>(m_lights.size() - 1);
//     m_light_frames.insert({id,offsetPose});
//     m_light_parent.insert({id,parent});
//     return id;
// }


// CH_SENSOR_API unsigned int ChScene::AddSpotLight(ChFramed offsetPose,
//                                                  ChColor color,
//                                                  float max_range,
//                                                  float total_width,
//                                                  float falloff_start) {
//     Light spot;
//     spot.type = LightType::SPOT_LIGHT;
//     ChVector3f frame_pos = offsetPose.GetPos();
//     ChVector3f local_dir(1, 0, 0);
//     ChVector3f world_dir = offsetPose.GetRot().Rotate(local_dir);
//     spot.pos = make_float3(frame_pos.x(), frame_pos.y(), frame_pos.z());
//     spot.spot_dir = make_float3(world_dir.x(), world_dir.y(), world_dir.z());

//     spot.color = {color.R, color.G, color.B};
//     spot.max_range = max_range;
//     spot.cos_total_width = cosf(total_width);
//     spot.cos_falloff_start = cosf(falloff_start);

//     lights_changed = true;
//     m_lights.push_back(spot);

//     return static_cast<unsigned int>(m_lights.size() - 1);
// }

/// interpolate between two vectors
ChVector3f ChLerp(ChVector3f& a, ChVector3f& b, double x) {
    return a + (b - a) * x;
}

// CH_SENSOR_API void ChScene::UpdateLight(unsigned int id, ChFramed newpose) {
//     Light* light = &m_lights[id];
//     switch (light->type) {
//         case LightType::SPOT_LIGHT: {
//             if (light->parent_id > 0) { // If light has a parent
//                 ChFramed local_frame = GetLightFrame(id);
//                 ChFramed parent_frame = GetLightParentFrame(id);
//                 ChFramed glob_frame = parent_frame * local_frame;

//                 ChVector3f glob_pos = glob_frame.GetPos();
//                 ChVector3f local_dir{1, 0, 0};
//                 ChVector3f world_dir = glob_frame.GetRot().Rotate(local_dir);
//                 ChVector3f prev_pos = ChVector3f(light->pos.x, light->pos.y, light->pos.z);
//                 ChVector3f prev_dir = ChVector3f(light->spot_dir.x, light->spot_dir.y, light->spot_dir.z);

//                 float interpolation_factor = 0.4;
//                 ChVector3f interp_pos = glob_pos +  interpolation_factor * (glob_pos - prev_pos);
//                 ChVector3f interp_dir = world_dir + interpolation_factor * (world_dir - prev_dir);

//                 light->pos = make_float3(interp_pos.x(), interp_pos.y(), interp_pos.z());
//                 light->spot_dir = make_float3(interp_dir.x(), interp_dir.y(), interp_dir.z());
//             } else { // Not attached to parent just change pose
//                 //std::cout << "Updating Pose of Spot Light" << std::endl;
//                 ChVector3f glob_pos = newpose.GetPos();
//                 ChVector3f local_dir{1, 0, 0};
//                 ChVector3f world_dir = newpose.GetRot().Rotate(local_dir);
//                 light->pos = make_float3(glob_pos.x(), glob_pos.y(), glob_pos.z());
//                 light->spot_dir = make_float3(world_dir.x(), world_dir.y(), world_dir.z());
//             }
//             break;
//         }
//         case LightType::AREA_LIGHT: {
//             break;
//         }
//         case LightType::POINT_LIGHT: {
//             ChVector3f glob_pos = newpose.GetPos();
//             light->pos = make_float3(glob_pos.x(), glob_pos.y(), glob_pos.z());
//             break;
//         }
//         default:
//             break;
//     }
//     lights_changed = true;
// }

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
