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

#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include "chrono_sensor/sensors/ChTransientSensor.h"

#include "chrono_thirdparty/stb/stb_image_write.h"
#include "chrono_thirdparty/filesystem/path.h"

#include <vector>
#include <sstream>

#include <cuda_runtime_api.h>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterSave::ChFilterSave(std::string data_path, std::string name) : ChFilter(name) {
    m_path = data_path;
}

CH_SENSOR_API void ChFilterSave::Apply() {
    std::string filename = m_path + "frame_" + std::to_string(m_frame_number) + ".png";
    m_frame_number++;

    if (m_r8_in) {
        cudaMemcpyAsync(m_host_r8->Buffer.get(), m_r8_in->Buffer.get(), m_r8_in->Width * m_r8_in->Height * sizeof(char),
                        cudaMemcpyDeviceToHost, m_cuda_stream);
        cudaStreamSynchronize(m_cuda_stream);
        // write a grayscale png
        if (!stbi_write_png(filename.c_str(), m_host_r8->Width, m_host_r8->Height, 1, m_host_r8->Buffer.get(),
                            m_host_r8->Width)) {
            std::cerr << "Failed to write R8 image: " << filename << "\n";
        }
    } else if (m_rgba8_in) {
        cudaMemcpyAsync(m_host_rgba8->Buffer.get(), m_rgba8_in->Buffer.get(),
                        m_rgba8_in->Width * m_rgba8_in->Height * sizeof(PixelRGBA8), cudaMemcpyDeviceToHost,
                        m_cuda_stream);
        cudaStreamSynchronize(m_cuda_stream);
        if (m_is_transient) {
            int w = m_host_rgba8->Width/m_num_bins;
            int h = m_host_rgba8->Height;
            for (int bin = 0; bin < m_num_bins; bin++) {
                std::string binname =
                    m_path + "frame_" + std::to_string(m_frame_number) + "_bin" + std::to_string(bin) + ".png";
                
                PixelRGBA8* bin_data = m_host_rgba8->Buffer.get() + bin*w*h;
                // to verify loop over the bin image values
                //int activePixels = 0;
                //for (int i = 0; i < w; i++) {
                //    for (int j = 0; j < h; j++) {
                //        PixelRGBA8 p = bin_data[i*h + j];
                //        if (p.R != 0 || p.G != 0 || p.B != 0) {
                //             //std::cout << "bin: " << bin << " i: " << i << " j: " << j << " r: " << (int)p.R << " g: " << (int)p.G << " b: " << (int)p.B << " a: " << (int)p.A << std::endl;
                //            activePixels++;
                //        }
                //    }
                //}
                //std::cout << "Bin " << bin << " " << "Active Pixels: " << activePixels << std::endl;
                if (!stbi_write_png(binname.c_str(),w,h, sizeof(PixelRGBA8),
                                    m_host_rgba8->Buffer.get() + bin*w*h, sizeof(PixelRGBA8) * w)) {
                    std::cerr << "Failed to write RGBA8 image: " << filename << "\n";
                }
            }
        } else {
            // write an rgba png
            if (!stbi_write_png(filename.c_str(), m_host_rgba8->Width, m_host_rgba8->Height, sizeof(PixelRGBA8),
                                m_host_rgba8->Buffer.get(), sizeof(PixelRGBA8) * m_host_rgba8->Width)) {
                std::cerr << "Failed to write RGBA8 image: " << filename << "\n";
            }
        }
    } else if (m_semantic_in) {
        cudaMemcpyAsync(m_host_semantic->Buffer.get(), m_semantic_in->Buffer.get(),
                        m_semantic_in->Width * m_semantic_in->Height * sizeof(PixelSemantic), cudaMemcpyDeviceToHost,
                        m_cuda_stream);
        cudaStreamSynchronize(m_cuda_stream);
        // write an rgba png
        if (!stbi_write_png(filename.c_str(), m_host_semantic->Width, m_host_semantic->Height, 4,
                            m_host_semantic->Buffer.get(), 4 * m_host_semantic->Width)) {
            std::cerr << "Failed to write semantic image: " << filename << "\n";
        }
    }
}

CH_SENSOR_API void ChFilterSave::Initialize(std::shared_ptr<ChSensor> pSensor,
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

        if (auto cam = std::dynamic_pointer_cast<ChTransientSensor>(pSensor)) {
            m_is_transient = true;
            m_num_bins = (unsigned int)cam->GetNumBins();
        }
        m_host_rgba8->Width = m_rgba8_in->Width;
        m_host_rgba8->Height = m_rgba8_in->Height;
    } else if (auto pSemantic = std::dynamic_pointer_cast<SensorDeviceSemanticBuffer>(bufferInOut)) {
        m_semantic_in = pSemantic;
        m_host_semantic = chrono_types::make_shared<SensorHostSemanticBuffer>();
        std::shared_ptr<PixelSemantic[]> b(
            cudaHostMallocHelper<PixelSemantic>(m_semantic_in->Width * m_semantic_in->Height),
            cudaHostFreeHelper<PixelSemantic>);
        m_host_semantic->Buffer = std::move(b);
        m_host_semantic->Width = m_semantic_in->Width;
        m_host_semantic->Height = m_semantic_in->Height;
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
