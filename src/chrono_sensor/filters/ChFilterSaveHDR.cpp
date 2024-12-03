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
// Authors: Asher Elmquist, Tony Adriansen
// =============================================================================
//
//
// =============================================================================

#include "chrono_sensor/filters/ChFilterSaveHDR.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"

#include "chrono_thirdparty/stb/stb_image_write.h"
#include "chrono_thirdparty/filesystem/path.h"

#include <vector>
#include <sstream>

#include <cuda_runtime_api.h>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterSaveHDR::ChFilterSaveHDR(std::string data_path, std::string name) : ChFilter(name) {
    m_path = data_path;
}

namespace {

void ToHDRFormat(const char* in_buffer, size_t w, size_t h, size_t channels, float* hdr_buffer) {
    size_t len = w * h * channels;
    for (size_t i = 0; i < len; i++) {
        hdr_buffer[i] = static_cast<uint8_t>(in_buffer[i]) / 255.0f;
    }
}

void ToHDRFormat(const PixelRGBA8* in_buffer, size_t w, size_t h, float* hdr_buffer) {
    size_t num_pixels = w * h;
    for (size_t i = 0; i < num_pixels; i++) {
        size_t offset = i * 4;
        hdr_buffer[offset] = static_cast<uint8_t>(in_buffer[i].R) / 255.0f;
        hdr_buffer[offset + 1] = static_cast<uint8_t>(in_buffer[i].G) / 255.0f;
        hdr_buffer[offset + 2] = static_cast<uint8_t>(in_buffer[i].B) / 255.0f;
        hdr_buffer[offset + 3] = static_cast<uint8_t>(in_buffer[i].A) / 255.0f;
    }
}

}  // namespace

CH_SENSOR_API void ChFilterSaveHDR::Apply() {
    std::string filename = m_path + "frame_" + std::to_string(m_frame_number) + ".hdr";
    m_frame_number++;
    if (m_r8_in) {
        cudaMemcpyAsync(m_host_r8->Buffer.get(), m_r8_in->Buffer.get(), m_r8_in->Width * m_r8_in->Height * sizeof(char),
                        cudaMemcpyDeviceToHost, m_cuda_stream);
        cudaStreamSynchronize(m_cuda_stream);
        std::vector<float> hdr_buffer(m_host_r8->Width * m_host_r8->Height);
        ToHDRFormat(m_host_r8->Buffer.get(), m_host_r8->Width, m_host_r8->Height, 1, hdr_buffer.data());
        // write a grayscale png
        if (!stbi_write_hdr(filename.c_str(), m_host_r8->Width, m_host_r8->Height, 1, hdr_buffer.data())) {
            std::cerr << "Failed to write HDR R image: " << filename << "\n";
        }
    } else if (m_rgba8_in) {
        cudaMemcpyAsync(m_host_rgba8->Buffer.get(), m_rgba8_in->Buffer.get(),
                        m_rgba8_in->Width * m_rgba8_in->Height * sizeof(PixelRGBA8), cudaMemcpyDeviceToHost,
                        m_cuda_stream);
        cudaStreamSynchronize(m_cuda_stream);

        std::vector<float> hdr_buffer(m_host_rgba8->Width * m_host_rgba8->Height * 4);
        ToHDRFormat(m_host_rgba8->Buffer.get(), m_host_rgba8->Width, m_host_rgba8->Height, hdr_buffer.data());

        static_assert(sizeof(PixelRGBA8) == 4);
        // write an rgba png
        if (!stbi_write_hdr(filename.c_str(), m_host_rgba8->Width, m_host_rgba8->Height, sizeof(PixelRGBA8),
                            hdr_buffer.data())) {
            std::cerr << "Failed to write RGBA8 image: " << filename << "\n";
        }
    }
}

CH_SENSOR_API void ChFilterSaveHDR::Initialize(std::shared_ptr<ChSensor> pSensor,
                                               std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    if (auto pR8 = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut)) {
        m_r8_in = pR8;
        m_host_r8 = chrono_types::make_shared<SensorHostR8Buffer>();
        std::shared_ptr<char[]> b(cudaHostMallocHelper<char>(m_r8_in->Width * m_r8_in->Height),
                                  cudaHostFreeHelper<char>);
        m_host_r8->Buffer = std::move(b);
        m_host_r8->Width = m_r8_in->Width;
        m_host_r8->Height = m_r8_in->Height;
    } else if (auto pRGBA8 = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut)) {
        m_rgba8_in = pRGBA8;
        m_host_rgba8 = chrono_types::make_shared<SensorHostRGBA8Buffer>();
        std::shared_ptr<PixelRGBA8[]> b(cudaHostMallocHelper<PixelRGBA8>(m_rgba8_in->Width * m_rgba8_in->Height),
                                        cudaHostFreeHelper<PixelRGBA8>);
        m_host_rgba8->Buffer = std::move(b);
        m_host_rgba8->Width = m_rgba8_in->Width;
        m_host_rgba8->Height = m_rgba8_in->Height;
    } else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }

    std::vector<std::string> split_string;

    std::istringstream istring(m_path);

    std::string substring;
    while (std::getline(istring, substring, '/')) {
        split_string.push_back(substring);
    }

    std::string partial_path = "";
    for (auto s : split_string) {
        if (s != "") {
            partial_path += s + "/";
            if (!filesystem::path(partial_path).exists()) {
                if (!filesystem::create_directory(filesystem::path(partial_path))) {
                    std::cerr << "Could not create directory: " << partial_path << std::endl;
                } else {
                    std::cout << "Created directory for sensor data: " << partial_path << std::endl;
                }
            }
        }
    }

    if (m_path.back() != '/' && m_path != "")
        m_path += '/';

    // openGL buffers are bottom to top...so flip when writing png.
    stbi_flip_vertically_on_write(1);
}

}  // namespace sensor
}  // namespace chrono
