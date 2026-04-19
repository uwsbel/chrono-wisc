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
// utility functions used for optix convenience
//
// =============================================================================

#include <cstring>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>

#include <optix_stubs.h>
// #include <optix_function_table_definition.h>

#include <ImfInputFile.h>
#include <ImfFrameBuffer.h>
#include <ImfChannelList.h>
#include <ImfHeader.h>
#include <ImathBox.h>

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/optix/ChOptixUtils.h"

#ifdef USE_CUDA_NVRTC
    #include <nvrtc.h>
#endif

namespace chrono {
namespace sensor {

static std::string shader_dir = CHRONO_SENSOR_SHADER_DIR;

void SetSensorShaderDir(const std::string& path) {
    shader_dir = path;
}

const std::string& GetSensorShaderDir() {
    return shader_dir;
}

void GetShaderFromFile(OptixDeviceContext context,
                       OptixModule& module,
                       const std::string& file_name,
                       OptixModuleCompileOptions& module_compile_options,
                       OptixPipelineCompileOptions& pipeline_compile_options) {
    
#ifdef USE_CUDA_NVRTC
    // std::chrono::high_resolution_clock::time_point start_compile = std::chrono::high_resolution_clock::now();
    
    std::string cuda_file = shader_dir + "/" + file_name + ".cu";
    std::string str;
    std::ifstream f(cuda_file);
    if (f.good()) {
        std::stringstream source_buffer;
        source_buffer << f.rdbuf();
        str = source_buffer.str();
    } else {
        throw std::runtime_error("CUDA file not found for NVRTC: " + cuda_file);
    }

    // compile CUDA code with NVRTC
    nvrtcProgram nvrtc_program;
    NVRTC_ERROR_CHECK(nvrtcCreateProgram(&nvrtc_program, str.c_str(), cuda_file.c_str(), 0, NULL, NULL));

    // complete list of flags to be used for NVRTC
    std::vector<const char*> nvrtc_compiler_flag_list;

    // include directories passed from CMake
    std::vector<std::string> scoping_dir_list;  // to keep the flags from going out of scope
    const char* nvrtc_include_dirs[] = {CUDA_NVRTC_INCLUDE_LIST};
    int num_dirs = sizeof(nvrtc_include_dirs) / sizeof(nvrtc_include_dirs[0]);
    for (int i = 0; i < num_dirs - 1; i++) {
        scoping_dir_list.push_back(std::string("-I") + nvrtc_include_dirs[i]);
        nvrtc_compiler_flag_list.push_back(scoping_dir_list[i].c_str());
    }

    // compile flags passed from CMake
    const char* nvrtc_flags[] = {CUDA_NVRTC_FLAG_LIST};
    int num_flags = sizeof(nvrtc_flags) / sizeof(nvrtc_flags[0]);
    for (int i = 0; i < num_flags - 1; i++) {
        nvrtc_compiler_flag_list.push_back(nvrtc_flags[i]);
    }

    // runtime compile CU to PTX with NVRTC
    const nvrtcResult compile_result =
        nvrtcCompileProgram(nvrtc_program, (int)nvrtc_compiler_flag_list.size(), nvrtc_compiler_flag_list.data());

    std::string nvrt_compilation_log;
    size_t log_length;
    nvrtcGetProgramLogSize(nvrtc_program, &log_length);
    nvrt_compilation_log.resize(log_length);
    if (log_length > 0) {
        NVRTC_ERROR_CHECK(nvrtcGetProgramLog(nvrtc_program, &nvrt_compilation_log[0]));
    }
    if (compile_result != NVRTC_SUCCESS) {
        throw std::runtime_error(std::string("Error: ").append(__FILE__) + " at line " + std::to_string(__LINE__) +
                                 "\n" + nvrt_compilation_log);
    }

    std::string ptx;
    size_t ptx_size = 0;
    NVRTC_ERROR_CHECK(nvrtcGetPTXSize(nvrtc_program, &ptx_size));
    ptx.resize(ptx_size);
    NVRTC_ERROR_CHECK(nvrtcGetPTX(nvrtc_program, &ptx[0]));

    // std::chrono::high_resolution_clock::time_point end_compile = std::chrono::high_resolution_clock::now();

    // std::cout << "Rebuilt root acceleration structure, addr = " << m_root << std::endl;
    // std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_compile -
    // start_compile); std::cout << "NVRTC Compilation: " << file_name << " | " << wall_time.count() << std::endl;
    // wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

#else
    std::string ptx_file = shader_dir + "/" + file_name + ".ptx";
    std::string ptx;
    std::ifstream f(ptx_file);
    if (f.good()) {
        std::stringstream source_buffer;
        source_buffer << f.rdbuf();
        ptx = source_buffer.str();
    } else {
        throw std::runtime_error("PTX file not found: " + ptx_file);
    }

#endif // USE_CUDA_NVRTC
    
    char log[2048];
    size_t sizeof_log = sizeof(log);
    OPTIX_ERROR_CHECK(optixModuleCreate(context, &module_compile_options, &pipeline_compile_options, ptx.c_str(),
                                        ptx.size(), log, &sizeof_log, &module));
    
}

void optix_log_callback(unsigned int level, const char* tag, const char* message, void*) {
    std::cerr << "[" << std::setw(2) << level << "][" << std::setw(12) << tag << "]: " << message << "\n";
}

ByteImageData LoadByteImage(const std::string& filename) {
    ByteImageData img_data;
    int w;
    int h;
    int c;
    unsigned char* data = stbi_load(filename.c_str(), &w, &h, &c, 0);

    if (!data) {
        img_data.w = 0;
        img_data.h = 0;
        img_data.c = 0;
        return img_data;  // return if loading failed
    }

    img_data.data = std::vector<unsigned char>(w * h * c);
    img_data.w = w;
    img_data.h = h;
    img_data.c = c;
    memcpy(img_data.data.data(), data, sizeof(unsigned char) * img_data.data.size());

    stbi_image_free(data);

    return img_data;
}


FloatImageData LoadFloatImage(const std::string& filename) {
    FloatImageData img_data;
    img_data.w = 0;
    img_data.h = 0;
    img_data.c = 0;

    try {
        Imf::InputFile file(filename.c_str());

        const Imf::Header& header = file.header();
        const Imath::Box2i dw = header.dataWindow();

        const int img_w = dw.max.x - dw.min.x + 1;
        const int img_h = dw.max.y - dw.min.y + 1;

        // Read all EXR channels in the file, ignoring their names.
        int num_channels = 0;
        for (Imf::ChannelList::ConstIterator it = header.channels().begin(); it != header.channels().end(); ++it) {
            if (it.channel().xSampling != 1 || it.channel().ySampling != 1) {
                throw std::runtime_error("LoadFloatImage: subsampled channels are not supported.");
            }
            ++num_channels;
        }
        
        std::vector<std::string> channel_names;
        if (num_channels == 1) {
            channel_names = {"R"};
        }
        else if (num_channels == 2) {
            channel_names = {"R", "A"};
        }
        else if (num_channels == 3) {
            channel_names = {"R", "G", "B"};
        }
        else if (num_channels == 4) {
            channel_names = {"R", "G", "B", "A"};
        }
        else {
            throw std::runtime_error("LoadFloatImage: only EXR images with 1 to 4 channels are supported.");
        }

        // One planar float buffer per EXR channel.
        std::vector<std::vector<float>> planar(num_channels, std::vector<float>(img_w * img_h));

        Imf::FrameBuffer frame_buffer;

        for (int ch_idx = 0; ch_idx < num_channels; ++ch_idx) {
            char* base = reinterpret_cast<char*>(planar[ch_idx].data() - dw.min.x - dw.min.y * img_w);

            frame_buffer.insert(
                channel_names[ch_idx].c_str(),
                Imf::Slice(
                    Imf::FLOAT,            // convert to float in memory
                    base,
                    sizeof(float),         // x stride
                    sizeof(float) * img_w, // y stride
                    1,                     // x sampling
                    1,                     // y sampling
                    0.0f                   // fill value
                )
            );
        }

        file.setFrameBuffer(frame_buffer);
        file.readPixels(dw.min.y, dw.max.y);

        // Pack planar -> interleaved
        img_data.w = img_w;
        img_data.h = img_h;
        img_data.c = num_channels;
        img_data.data.resize(img_w * img_h * num_channels);
        float max_val = std::numeric_limits<float>::min();
        float min_val = std::numeric_limits<float>::max();
        for (int y = 0; y < img_h; ++y) {
            for (int x = 0; x < img_w; ++x) {
                int px_idx = y * img_w + x;
                int out = px_idx * num_channels;
                for (int ch_idx = 0; ch_idx < num_channels; ++ch_idx) {
                    float val = planar[ch_idx][px_idx];
                    if (val > max_val) max_val = val;
                    if (val < min_val) min_val = val;
                    img_data.data[out + ch_idx] = planar[ch_idx][px_idx];
                }
            }
        }
        // Debug
        printf("\nLoaded float image: %s | w=%d, h=%d, c=%d, min=%f, max=%f\n\n", filename.c_str(), img_data.w, img_data.h, img_data.c, min_val, max_val);

        return img_data;
    }
    catch (const std::exception&) {
        img_data.w = 0;
        img_data.h = 0;
        img_data.c = 0;
        img_data.data.clear();
        return img_data;
    }
}

}  // namespace sensor
}  // namespace chrono
