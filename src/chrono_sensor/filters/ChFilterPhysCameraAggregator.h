// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bo-Hsun Chen
// =============================================================================
// 
// Filter to aggregate illumination irradiance over exposure time and pixel area
// with considering aperture number, i.e., integrate irradiance [W/m^2] of each
// pixel to energy [J]
// 
// =============================================================================

#ifndef CHFILTERPHYSCAMERAAGGREGATOR_H
#define CHFILTERPHYSCAMERAAGGREGATOR_H

#include "chrono_sensor/filters/ChFilter.h"
#include <cuda.h>
#include "chrono/core/ChVector3.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// A filter that adjust the brightness of the image according to exposure time and sensitivity coefficients
class CH_SENSOR_API ChFilterPhysCameraAggregator : public ChFilter {
	public:
		/// Class constructor
		/// @param aperture_num (N) aperture number = focal_length / aperture_diameter, [1/1]
		/// @param expsr_time (t) exposure time, [sec]
		/// @param pixel_size (C) pixel size, [m]
		/// @param max_scene_light_amount (P) maximum brightness amount of a light source in the scene, [W]
		/// @param rgb_QE_vec vector of RGB quantum efficiencies, [1/1]
		/// @param aggregator_gain (G_aggregator) proportional gain, [1/1]
		/// @param name The string name of the filter
		ChFilterPhysCameraAggregator(
			float aperture_num, float expsr_time, float pixel_size, float max_scene_light_amount,
			ChVector3f rgb_QE_vec, float aggregator_gain, std::string name = "Aggregation Filter in Phys Camera"
		);

		/// Apply function
		virtual void Apply();

		/// Set control parameters in the filter function
		/// @param aperture_num (N) aperture number = focal_length / aperture_diameter, [1/1]
		/// @param expsr_time (t) exposure time, [sec]
		void SetFilterCtrlParameters(float aperture_num, float expsr_time);

		/// Set model parameters in the filter function
		/// @param pixel_size (C) pixel size, [m]
		/// @param max_scene_light_amount (P) maximum brightness amount of a light source in the scene, [W]
		/// @param rgb_QE_vec vector of RGB quantum efficiencies, [1/1]
		/// @param aggregator_gain (G_aggregator) proportional gain, [1/1]
		void SetFilterModelParameters(
			float pixel_size, float max_scene_light_amount, ChVector3f rgb_QE_vec, float aggregator_gain
		);

		/// Initializes all data needed by the filter access apply function.
		/// @param pSensor A pointer to the sensor on which the filter is attached.
		/// @param bufferInOut A buffer that is passed into the filter. This data is what will be made available for the
		/// user.
		virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

	private:
		float m_aperture_num; 								// (N) aperture number=focal_length/aperture_diameter, [1/1]
		float m_expsr_time; 								// (t) exposure time, [sec]
		float m_pixel_size;									// (C) pixel size, [m]
		float m_max_scene_light_amount;						// (P) maximum brightness amount of a light source in the scene, [W]
		float m_rgb_QEs[3];									// array of RGB quantum efficiencies
		float m_aggregator_gain;							// proportional gain of illumination amplification, [1/1]
		std::shared_ptr<SensorDeviceHalf4Buffer> m_in_out;	// input/output buffer for RGBA(Half4)
		CUstream m_cuda_stream;								// reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
