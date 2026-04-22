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
// Authors: Bo-Hsun Chen
// =============================================================================
//
// 
// 
// =============================================================================

#ifndef CHOPENEXRUTILS_H
#define CHOPENEXRUTILS_H

#include <cstdint>
#include <string>
#include <vector>

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/optix/ChOptixUtils.h"

namespace chrono {
namespace sensor {

/// @brief Load a float EXR image and return interleaved float data.
/// @param filename the file name of the image to be loaded
/// @return the loaded float image data, or an empty struct with 0 values if loading failed
CH_SENSOR_API FloatImageData LoadFloatImage(const std::string& filename);

/// @brief Helper function to write a float map (ex: depth map or normal map) to an EXR file
/// @param file_path The path string to the output file
/// @param width The width of the image in pixels
/// @param height The height of the image in pixels
/// @param data A pointer to the image data in float format
/// @return true if the file was successfully written, false otherwise
CH_SENSOR_API bool WriteFloatToExr(const std::string& file_path,
                                   uint16_t width,
                                   uint16_t height,
                                   uint16_t num_chs,
                                   const void* data);

}  // namespace sensor
}  // namespace chrono

#endif