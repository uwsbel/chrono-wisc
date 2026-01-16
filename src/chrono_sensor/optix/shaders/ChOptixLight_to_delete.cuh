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
// Authors: Nevindu M. Batagoda, Bo-Hsun Chen
// =============================================================================
//
// Device-side light class base for different types of lights in OptiX shaders.
//
// =============================================================================

#ifndef CHRONO_SENSOR_OPTIX_LIGHT_CUH
#define CHRONO_SENSOR_OPTIX_LIGHT_CUH

#include <optix.h>
#include "chrono/core/ChVector3.h"
#include "chrono/assets/ChColor.h"
// #include "chrono_sensor/optix/ChOptixDefinitions.h"





// namespace chrono {
// namespace sensor {
// namespace optix {

using namespace chrono;

class ChOptixLight {
public:
	/// Constructor of the base OptiX light class for different types of lights used for camera sensor rendering.
	/// @param light_type The type of light being created.
	/// @param delta Whether the light is a delta light source.
	///	@param pos Position of the light in the scene.
	///	@param color Color intensity of the light.
	/// @param max_range Range at which the light intensity is equal to 1% of its maximum color intensity. If set to -1, follows inverse square law.
	ChOptixLight(
		LightType light_type,
		bool delta,
		float3 pos,
		float3 color,
		float max_range
	) :
		light_type(light_type),
		delta(delta),
		pos(pos),
		color(color),
		max_range(max_range)
	{}

	~ChOptixLight() {}

	// /// @brief Check visibility between the light and the hit point, and sample the light.
	// /// @param cntxt_params context parameters
	// /// @param prd_camera per-ray data (PRD) of the camera ray
	// /// @param light_sample the light sample to be updated
	// /// @return True if the light is visible from the hit point, false otherwise
	// __device__ virtual bool CheckVisibleAndSampleLight(
	// 	const ContextParameters& cntxt_params,
	// 	const PerRayData_camera* prd_camera,
	// 	LightSample& light_sample
	// ) const {}

	/// @brief Set the position of the light.
	/// @param position The new position of the light.
	void SetLightPos(const ChVector3f& position) {
		this->pos = {position.x(), position.y(), position.z()};
	}

	/// @brief Set the orientation of the light.
	/// @param new_x_axis The new x-axis of the light's orientation.
	/// @param new_y_axis The new y-axis of the light's orientation.
	/// @param new_z_axis The new z-axis of the light's orientation.
	void SetLightRot(const ChVector3f& new_x_axis, const ChVector3f& new_y_axis, const ChVector3f& new_z_axis) {
		this->x_axis = {new_x_axis.x(), new_x_axis.y(), new_x_axis.z()};
		this->y_axis = {new_y_axis.x(), new_y_axis.y(), new_y_axis.z()};;
		this->z_axis = {new_z_axis.x(), new_z_axis.y(), new_z_axis.z()};;
	}
	/// @brief Set the color intensity of the light.
	/// @param color The new color intensity of the light.
	void SetLightColor(const ChColor& color) { this->color = {color.R, color.G, color.B}; }

	/// @brief  Set the maximum range of the light.
	/// @param new_max_range The new maximum range of the light.
	void SetLightMaxRange(const float& new_max_range) {
		this->max_range = new_max_range;
		// Calculate the attenuation scale of the light based on its maximum range.
		this->atten_scale = (new_max_range > 0) ? (0.01 * new_max_range * new_max_range) : 1.f;	
	}

private:
	LightType light_type;
	bool delta;
	float3 pos;
	float3 x_axis;
	float3 y_axis;
	float3 z_axis;
	float3 color;
	float max_range;
	float atten_scale;
};

// }  // namespace optix
// }  // namespace sensor
// }  // namespace chrono

#endif  // CHRONO_SENSOR_OPTIX_LIGHT_CUH