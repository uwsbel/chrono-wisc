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
// Filter to do defocus blur based on camera control parameters and depth map
// 
// =============================================================================

#ifndef CHFILTERREALCAMERADEFOCUSBLUR_H
#define CHFILTERREALCAMERADEFOCUSBLUR_H

#include "chrono_sensor/filters/ChFilter.h"
#include <cuda.h>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// A filter that adjust the brightness of the image according to exposure time and sensitivity coefficients
class CH_SENSOR_API ChFilterRealCameraDefocusBlur : public ChFilter {
	public:
		/// Class constructor
		/// @param focal_length (f) focal length, [m]
		/// @param focus_dist (U) focus distance, [m]
		/// @param aperture_num (N) F-number (or aperture number) = focal_length / aperture_diameter, [1/1]
		/// @param pixel_size (C) length of a pixel, [m]
		/// @param defocus_gain proportional gain, [1/1]
		/// @param defocus_bias defocus-blur diameter bias, [px]
		/// @param name The string name of the filter
		ChFilterRealCameraDefocusBlur(
			float focal_length, float focus_dist, float aperture_num, float pixel_size, float defocus_gain,
			float defocus_bias, std::string name = "Defocus Blur Filter in Real Camera"
		);

		/// Apply function
		virtual void Apply();

		/// Set control parameters in the filter function
		/// @param focal_length (f) focal length, [m]
		/// @param focus_dist (U) focus distance, [m]
		/// @param aperture_num (N) F-number (or aperture number) = focal_length / aperture_diameter, [1/1]
		void SetFilterCtrlParameters(float focal_length, float focus_dist, float aperture_num);

		/// Set model parameters in the filter function
		/// @param pixel_size (C) length of a pixel, [m]
		/// @param defocus_gain (defocus_gain) proportional gain, [1/1]
		/// @param defocus_bias (defocus_bias) bias, [px]
		void SetFilterModelParameters(float pixel_size, float defocus_gain, float defocus_bias);

		/// Initializes all data needed by the filter access apply function.
		/// @param pSensor A pointer to the sensor on which the filter is attached.
		/// @param bufferInOut A buffer that is passed into the filter. This data is what will be made available for the
		/// user.
		virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    float m_focal_length;										// focal length, [m]
	float m_focus_dist;											// focus distance, [m]
    float m_pixel_size;											// length of a pixel, [m]
    float m_aperture_num;										// F-number (or aperture number) = focal_length / aperture_diameter, [1/1]
    float m_defocus_gain;										// proportional gain of defocus-blur diameter, [1/1]
	float m_defocus_bias;										// bias of defocus-blur diameter, [px]
	std::shared_ptr<SensorDeviceRGBDHalf4Buffer> m_buffer_in;	///< input buffer for rgbd(half4)
    std::shared_ptr<SensorDeviceHalf4Buffer> m_buffer_out;  	///< output buffer for rgba(half4)
    CUstream m_cuda_stream;                                 	///< reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
