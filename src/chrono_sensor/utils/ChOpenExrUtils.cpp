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

#include "chrono_sensor/utils/ChOpenExrUtils.h"

#include <limits>
#include <stdexcept>
#include <iostream>

#include <OpenEXR/ImfInputFile.h>
#include <OpenEXR/ImfOutputFile.h>
#include <OpenEXR/ImfFrameBuffer.h>
#include <OpenEXR/ImfChannelList.h>
#include <OpenEXR/ImfHeader.h>
#include <Imath/ImathBox.h>

namespace chrono {
namespace sensor {

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


bool WriteFloatToExr(const std::string& file_path, uint16_t width, uint16_t height, uint16_t num_chs, const void* data) {
    if (!data) {
        std::cerr << "WriteFloatToExr: data pointer is null.\n";
        return false;
    }

    if (!(num_chs == 1 || num_chs == 3)) {
        std::cerr << "WriteFloatToExr: unsupported number of channels = " << num_chs
                  << ". Only 1 or 3 channels are supported.\n";
        return false;
    }

    try {
        const float* float_data = reinterpret_cast<const float*>(data);

        // Create EXR header
        Imf::Header header(width, height);

        // Define channels
        if (num_chs == 1) {
            header.channels().insert("Y", Imf::Channel(Imf::FLOAT));
        }
        else if (num_chs == 3) {
            header.channels().insert("R", Imf::Channel(Imf::FLOAT));
            header.channels().insert("G", Imf::Channel(Imf::FLOAT));
            header.channels().insert("B", Imf::Channel(Imf::FLOAT));
        }

        Imf::FrameBuffer frameBuffer;

        // Strides for interleaved float layout
        const uint64_t xStride = sizeof(float) * num_chs;
        const uint64_t yStride = sizeof(float) * num_chs * width;

        if (num_chs == 1) {
            frameBuffer.insert(
                "Y",
                Imf::Slice(
                    Imf::FLOAT,
                    reinterpret_cast<char*>(const_cast<float*>(float_data)),
                    xStride,
                    yStride
                )
            );
        }
        else if (num_chs == 3) {
            // R channel starts at float_data[0]
            frameBuffer.insert(
                "R",
                Imf::Slice(
                    Imf::FLOAT,
                    reinterpret_cast<char*>(const_cast<float*>(float_data)),
                    xStride,
                    yStride
                )
            );

            // G channel starts at float_data[1]
            frameBuffer.insert(
                "G",
                Imf::Slice(
                    Imf::FLOAT,
                    reinterpret_cast<char*>(const_cast<float*>(float_data + 1)),
                    xStride,
                    yStride
                )
            );

            // B channel starts at float_data[2]
            frameBuffer.insert(
                "B",
                Imf::Slice(
                    Imf::FLOAT,
                    reinterpret_cast<char*>(const_cast<float*>(float_data + 2)),
                    xStride,
                    yStride
                )
            );
        }

        Imf::OutputFile file(file_path.c_str(), header);
        file.setFrameBuffer(frameBuffer);
        file.writePixels(height);

        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to write EXR file: " << file_path << "\n"
                  << "Reason: " << e.what() << std::endl;
        return false;
    }
}

}  // namespace sensor
}  // namespace chrono
