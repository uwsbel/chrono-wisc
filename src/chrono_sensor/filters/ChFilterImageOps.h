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
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

#ifndef CHFILTERIMAGEOPS_H
#define CHFILTERIMAGEOPS_H

#include "chrono_sensor/filters/ChFilter.h"
#include <cuda.h>
#include <npp.h>

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

/// A filter that converts RGBA Float4 to RGBA8
class CH_SENSOR_API ChFilterImageHalf4ToRGBA8 : public ChFilter {
  public:
    /// Class constructor
    /// @param name String name of the filter.
    ChFilterImageHalf4ToRGBA8(std::string name = {});

    /// Apply function. Applies the resize operation to the image.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<SensorDeviceHalf4Buffer> m_buffer_in;   ///< holder of the output RGBA8 image
    std::shared_ptr<SensorDeviceRGBA8Buffer> m_buffer_out;  ///< holder of the intput RGBA float image
    CUstream m_cuda_stream;                                 ///< reference to the cuda stream
};

/// A filter that converts RGB channels in RGBD Half4 to RGBA Half4
class CH_SENSOR_API ChFilterRGBDHalf4ToImageHalf4 : public ChFilter {
  public:
    /// Class constructor
    /// @param name String name of the filter.
    ChFilterRGBDHalf4ToImageHalf4(std::string name = {});

    /// Apply function. Applies the resize operation to the image.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<SensorDeviceRGBDHalf4Buffer> m_buffer_in;   ///< holder of the output RGBA8 image
    std::shared_ptr<SensorDeviceHalf4Buffer> m_buffer_out;      ///< holder of the intput RGBA float image
    CUstream m_cuda_stream;                                     ///< reference to the cuda stream
};

/// A filter that converts D-channel in RGBD Half4 to R8
class CH_SENSOR_API ChFilterRGBDHalf4ToR8 : public ChFilter {
  public:
    /// Class constructor
    /// @param name String name of the filter.
    ChFilterRGBDHalf4ToR8(std::string name = {});

    /// Apply function. Applies the resize operation to the image.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

    /// Get the maximum distance from the input RGBD buffer
    /// @param buf_in input buffer
    /// @param buf_size size of buffer
    // __half GetMaxDistance(void* buf_in, int buf_size);

    /// Get the minimum distance from the input RGBD buffer
    /// @param buf_in input buffer
    /// @param buf_size size of buffer
    // __half GetMinDistance(void* buf_in, int buf_size);

  private:
    std::shared_ptr<SensorDeviceRGBDHalf4Buffer> m_buffer_in;   ///< holder of the input RGBD Half4 image
    std::shared_ptr<SensorDeviceR8Buffer> m_buffer_out;         ///< holder of the output R8 float image
    CUstream m_cuda_stream;                                     ///< reference to the cuda stream
};

/// A filter that converts RGBA Half4 to RGBA16
class CH_SENSOR_API ChFilterImageHalf4ToRGBA16 : public ChFilter {
  public:
    /// Class constructor
    /// @param name String name of the filter.
    ChFilterImageHalf4ToRGBA16(std::string name = {});

    /// Apply function. Applies the resize operation to the image.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<SensorDeviceHalf4Buffer> m_buffer_in;   ///< holder of the intput RGBA float image
    std::shared_ptr<SensorDeviceRGBA16Buffer> m_buffer_out;  ///< holder of the output RGBA16 image
    CUstream m_cuda_stream;                                 ///< reference to the cuda stream
};


/// A filter that converts Depth values to RGBA8
class CH_SENSOR_API ChFilterDepthToRGBA8 : public ChFilter {
  
 public:
    ChFilterDepthToRGBA8(std::string name = {});
    virtual void Apply();

    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<SensorDeviceDepthBuffer> m_buffer_in;   ///<
    std::shared_ptr<SensorDeviceRGBA8Buffer> m_buffer_out;  ///<
    CUstream m_cuda_stream;
};

/// A filter that converts normal map to RGBA8
class CH_SENSOR_API ChFilterNormalToRGBA8 : public ChFilter {
  
  public:
    ChFilterNormalToRGBA8(std::string name = {});
    virtual void Apply();
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<SensorDeviceNormalBuffer> m_buffer_in;   ///<
    std::shared_ptr<SensorDeviceRGBA8Buffer> m_buffer_out;  ///<
    CUstream m_cuda_stream;
};

/// A filter that, when applied to a sensor, resizes the image to the specified dimensions.
class CH_SENSOR_API ChFilterImageResize : public ChFilter {
  public:
    /// Class constructor
    /// @param w Desired width of the image.
    /// @param h Desired height of the image.
    /// @param name String name of the filter.
    ChFilterImageResize(int w, int h, std::string name = {});

    /// Apply function. Applies the resize operation to the image.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<SensorDeviceRGBA8Buffer> m_buffer_rgba8_in;   ///< holder of an input RGBA8 image
    std::shared_ptr<SensorDeviceRGBA8Buffer> m_buffer_rgba8_out;  ///< holder of an output RGBA8 image
    std::shared_ptr<SensorDeviceR8Buffer> m_buffer_r8_in;         ///< holder of an R8 input image
    std::shared_ptr<SensorDeviceR8Buffer> m_buffer_r8_out;        ///< holder of an R8 input image
    NppStreamContext m_cuda_stream;                               ///< reference to the cuda stream
    int m_w;                                                      ///< desired image width
    int m_h;                                                      ///< desired image height
};

/// A filter that, when applied to a sensor, reduces the resolution for antialiasing
class CH_SENSOR_API ChFilterImgAlias : public ChFilter {
  public:
    /// Class constructor
    /// @param factor reduction factor for antialiasing
    /// @param name String name of the filter
    ChFilterImgAlias(int factor, std::string name = {});

    /// Apply function. Applies the antialiasing reduction to the image.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<SensorDeviceRGBA8Buffer> m_buffer_rgba8_in;     ///< holder of an input RGBA8 image
    std::shared_ptr<SensorDeviceRGBA8Buffer> m_buffer_rgba8_out;    ///< holder of an output RGBA8 image
    std::shared_ptr<SensorDeviceRGBA16Buffer> m_buffer_rgba16_in;   ///< holder of an input RGBA16 image
    std::shared_ptr<SensorDeviceRGBA16Buffer> m_buffer_rgba16_out;  ///< holder of an output RGBA16 image
    std::shared_ptr<SensorDeviceR8Buffer> m_buffer_r8_in;           ///< holder of an R8 image
    std::shared_ptr<SensorDeviceR8Buffer> m_buffer_r8_out;          ///< holder of an R8 image
    std::shared_ptr<SensorDeviceFloat4Buffer> m_buffer_float4_in;   ///< holder of an R8 image
    std::shared_ptr<SensorDeviceFloat4Buffer> m_buffer_float4_out;  ///< holder of an R8 image
    CUstream m_cuda_stream;                                         ///< reference to the cuda stream
    // NppStreamContext m_cuda_stream;
    int m_factor;  ///< reduction factor for antialiasing
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
