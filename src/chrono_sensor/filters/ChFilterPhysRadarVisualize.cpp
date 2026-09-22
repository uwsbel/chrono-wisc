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
// Authors: Patrick Chen
// =============================================================================

#include "chrono_sensor/filters/ChFilterPhysRadarVisualize.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>

#include <cuda_runtime.h>

#include "chrono_sensor/filters/ChFilterVisualizeGuards.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"

#include "chrono_thirdparty/stb/stb_image_write.h"

namespace chrono {
namespace sensor {

namespace {

constexpr int k_margin = 34;   // room for the panel title and the axis labels
constexpr int k_glyph_w = 6;   // 5 columns plus one of spacing
constexpr int k_glyph_h = 7;

PixelRGBA8 Rgba(int r, int g, int b, int a = 255) {
    PixelRGBA8 p;
    p.R = (uint8_t)std::clamp(r, 0, 255);
    p.G = (uint8_t)std::clamp(g, 0, 255);
    p.B = (uint8_t)std::clamp(b, 0, 255);
    p.A = (uint8_t)a;
    return p;
}

/// Perceptually ordered sequential ramp, dark to bright, for a magnitude in [0, 1].
PixelRGBA8 MagnitudeColor(float t) {
    t = std::clamp(t, 0.f, 1.f);
    static const float stops[6][3] = {{0.03f, 0.03f, 0.15f}, {0.15f, 0.10f, 0.45f}, {0.25f, 0.35f, 0.62f},
                                      {0.23f, 0.62f, 0.55f}, {0.63f, 0.80f, 0.32f}, {0.99f, 0.95f, 0.55f}};
    const float scaled = t * 5.f;
    const int lower = std::min(4, (int)scaled);
    const float f = scaled - lower;
    return Rgba((int)(255 * (stops[lower][0] + f * (stops[lower + 1][0] - stops[lower][0]))),
                (int)(255 * (stops[lower][1] + f * (stops[lower + 1][1] - stops[lower][1]))),
                (int)(255 * (stops[lower][2] + f * (stops[lower + 1][2] - stops[lower][2]))));
}

/// Diverging ramp for a signed margin in [-1, 1]: blue below zero, grey at it, red above.
PixelRGBA8 MarginColor(float t) {
    t = std::clamp(t, -1.f, 1.f);
    if (t < 0.f) {
        const float f = -t;
        return Rgba((int)(255 * (0.55f - 0.45f * f)), (int)(255 * (0.55f - 0.30f * f)), (int)(255 * (0.55f + 0.35f * f)));
    }
    return Rgba((int)(255 * (0.55f + 0.42f * t)), (int)(255 * (0.55f - 0.42f * t)), (int)(255 * (0.55f - 0.42f * t)));
}

/// Five by seven glyphs, one byte per column, bit 0 at the top row.
const unsigned char* Glyph(char c) {
    static const unsigned char digits[10][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00}, {0x62, 0x51, 0x49, 0x49, 0x46},
        {0x22, 0x41, 0x49, 0x49, 0x36}, {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03}, {0x36, 0x49, 0x49, 0x49, 0x36},
        {0x06, 0x49, 0x49, 0x29, 0x1E}};
    static const unsigned char letters[26][5] = {
        {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36}, {0x3E, 0x41, 0x41, 0x41, 0x22},
        {0x7F, 0x41, 0x41, 0x22, 0x1C}, {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01},
        {0x3E, 0x41, 0x49, 0x49, 0x7A}, {0x7F, 0x08, 0x08, 0x08, 0x7F}, {0x00, 0x41, 0x7F, 0x41, 0x00},
        {0x20, 0x40, 0x41, 0x3F, 0x01}, {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40},
        {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F}, {0x3E, 0x41, 0x41, 0x41, 0x3E},
        {0x7F, 0x09, 0x09, 0x09, 0x06}, {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46},
        {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01}, {0x3F, 0x40, 0x40, 0x40, 0x3F},
        {0x1F, 0x20, 0x40, 0x20, 0x1F}, {0x3F, 0x40, 0x38, 0x40, 0x3F}, {0x63, 0x14, 0x08, 0x14, 0x63},
        {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43}};
    static const unsigned char space[5] = {0, 0, 0, 0, 0};
    static const unsigned char dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const unsigned char dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const unsigned char slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
    static const unsigned char colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const unsigned char plus[5] = {0x08, 0x08, 0x3E, 0x08, 0x08};

    if (c >= '0' && c <= '9')
        return digits[c - '0'];
    if (c >= 'A' && c <= 'Z')
        return letters[c - 'A'];
    if (c >= 'a' && c <= 'z')
        return letters[c - 'a'];
    switch (c) {
        case '.':
            return dot;
        case '-':
            return dash;
        case '/':
            return slash;
        case ':':
            return colon;
        case '+':
            return plus;
        default:
            return space;
    }
}

/// Simple raster canvas over an RGBA image laid out top row first.
struct Canvas {
    PixelRGBA8* pixels;
    int width;
    int height;

    void Set(int x, int y, PixelRGBA8 c) {
        if (x >= 0 && y >= 0 && x < width && y < height)
            pixels[(size_t)y * width + x] = c;
    }

    void Fill(int x0, int y0, int w, int h, PixelRGBA8 c) {
        for (int y = y0; y < y0 + h; y++)
            for (int x = x0; x < x0 + w; x++)
                Set(x, y, c);
    }

    void Frame(int x0, int y0, int w, int h, PixelRGBA8 c) {
        for (int x = x0; x < x0 + w; x++) {
            Set(x, y0, c);
            Set(x, y0 + h - 1, c);
        }
        for (int y = y0; y < y0 + h; y++) {
            Set(x0, y, c);
            Set(x0 + w - 1, y, c);
        }
    }

    void Cross(int x, int y, int radius, PixelRGBA8 c) {
        for (int d = -radius; d <= radius; d++) {
            Set(x + d, y, c);
            Set(x, y + d, c);
        }
    }

    void Disc(int x, int y, int radius, PixelRGBA8 c) {
        for (int dy = -radius; dy <= radius; dy++)
            for (int dx = -radius; dx <= radius; dx++)
                if (dx * dx + dy * dy <= radius * radius)
                    Set(x + dx, y + dy, c);
    }

    void Text(int x, int y, const std::string& s, PixelRGBA8 c) {
        int cursor = x;
        for (char ch : s) {
            const unsigned char* glyph = Glyph(ch);
            for (int col = 0; col < 5; col++)
                for (int row = 0; row < k_glyph_h; row++)
                    if (glyph[col] & (1 << row))
                        Set(cursor + col, y + row, c);
            cursor += k_glyph_w;
        }
    }
};

/// The glyph set is uppercase only, so anything rendered is folded to it.
std::string Upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](char c) { return (char)std::toupper((unsigned char)c); });
    return s;
}

std::string Format(const char* format, double value) {
    char text[64];
    std::snprintf(text, sizeof(text), format, value);
    return Upper(text);
}

}  // namespace

ChFilterPhysRadarVisualize::ChFilterPhysRadarVisualize(int w, int h, std::string name)
    : ChFilterVisualize(w, h, name) {}

ChFilterPhysRadarVisualize::~ChFilterPhysRadarVisualize() {}

void ChFilterPhysRadarVisualize::Initialize(std::shared_ptr<ChSensor> pSensor,
                                            std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_radar = std::dynamic_pointer_cast<ChPhysRadarSensor>(pSensor);
    if (!m_radar)
        InvalidFilterGraphSensorTypeMismatch(pSensor);

    m_buffer = std::dynamic_pointer_cast<SensorDevicePhysRadarFrame>(bufferInOut);
    if (!m_buffer)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    m_cuda_stream = m_radar->GetCudaStream();

    const ChRadarModelConfig& cfg = m_radar->GetConfig();
    if (m_plan_range <= 0.f)
        m_plan_range = (float)cfg.max_range;

    m_dsp = {};
    m_dsp.num_range_bins = m_buffer->NumRangeBins;
    m_dsp.num_doppler_bins = m_buffer->NumDopplerBins;
    m_dsp.num_azimuth_bins = m_buffer->NumAzimuthBins;

    const size_t map_cells = (size_t)m_buffer->NumRangeBins * m_buffer->NumDopplerBins;
    const size_t image_cells = (size_t)m_buffer->NumRangeBins * m_buffer->NumAzimuthBins;
    m_power_map.resize(map_cells);
    m_threshold_map.resize(map_cells);
    m_range_azimuth.resize(image_cells);
    m_detections.resize(m_buffer->DetectionCapacity);
    m_image.resize((size_t)m_w * m_h);
    m_range_azimuth_device =
        std::shared_ptr<float[]>(cudaMallocHelper<float>(image_cells), cudaFreeHelper<float>);

    if (!m_save_directory.empty())
        std::filesystem::create_directories(m_save_directory);

    bufferInOut = m_buffer;
}

void ChFilterPhysRadarVisualize::Compose() {
    const ChRadarModelConfig& cfg = m_radar->GetConfig();
    const int range_bins = (int)m_buffer->NumRangeBins;
    const int doppler_bins = (int)m_buffer->NumDopplerBins;
    const int azimuth_bins = (int)m_buffer->NumAzimuthBins;
    const unsigned int detections = m_buffer->NumDetections;

    Canvas canvas{m_image.data(), m_w, m_h};
    canvas.Fill(0, 0, m_w, m_h, Rgba(16, 17, 22));

    const int panel_w = m_w / 2 - 2 * k_margin;
    const int panel_h = (m_h - 24) / 2 - k_margin - 12;
    const int left = k_margin;
    const int right = m_w / 2 + k_margin;
    const int top = k_margin;
    const int bottom = (m_h - 24) / 2 + k_margin;

    const PixelRGBA8 label = Rgba(190, 195, 205);
    const PixelRGBA8 faint = Rgba(95, 100, 112);

    // Cell magnitudes carry the coherent gain of the whole interval, so their absolute value has
    // no natural scale. The noise floor does, and it is also what a detection is measured against.
    const float floor_db = 10.f * std::log10(std::max(m_buffer->NoisePowerPerCell, 1e-30f));
    float peak = m_buffer->NoisePowerPerCell;
    for (float v : m_power_map)
        peak = std::max(peak, v);
    const float peak_db = 10.f * std::log10(std::max(peak, 1e-30f));

    auto draw_map = [&](int x0, int y0, int columns, int rows, const std::function<float(int, int)>& value,
                        const std::function<PixelRGBA8(float)>& color) {
        for (int py = 0; py < panel_h; py++) {
            const int row = rows - 1 - (int)((int64_t)py * rows / panel_h);
            for (int px = 0; px < panel_w; px++) {
                const int column = (int)((int64_t)px * columns / panel_w);
                canvas.Set(x0 + px, y0 + py, color(value(column, row)));
            }
        }
        canvas.Frame(x0 - 1, y0 - 1, panel_w + 2, panel_h + 2, faint);
    };

    // Panel 1: the cube the coherent paths were deposited into.
    canvas.Text(left, top - 12, "RANGE X DOPPLER   DB OVER NOISE   PEAK " + Format("%.0f", peak_db - floor_db),
                label);
    draw_map(left, top, range_bins, doppler_bins,
             [&](int r, int d) {
                 const float power = m_power_map[(size_t)d * range_bins + r];
                 const float db = 10.f * std::log10(std::max(power, 1e-30f));
                 return (db - floor_db) / m_dynamic_range;
             },
             MagnitudeColor);
    canvas.Text(left, top + panel_h + 4, "0 M", faint);
    canvas.Text(left + panel_w - 7 * k_glyph_w, top + panel_h + 4,
                Format("%.0f", range_bins * m_buffer->RangeBinSize) + " M", faint);
    canvas.Text(left, top + 2, Format("%+.0f", m_buffer->MaxUnambiguousVelocity) + " M/S", faint);
    canvas.Text(left, top + panel_h - k_glyph_h - 2, Format("%+.0f", -m_buffer->MaxUnambiguousVelocity) + " M/S",
                faint);

    // Panel 2: how far each cell sits above or below its own detection threshold.
    canvas.Text(right, top - 12, "DETECTION MARGIN  DB ABOVE THRESHOLD", label);
    draw_map(right, top, range_bins, doppler_bins,
             [&](int r, int d) {
                 const size_t cell = (size_t)d * range_bins + r;
                 const float margin =
                     10.f * std::log10(std::max(m_power_map[cell], 1e-30f) / std::max(m_threshold_map[cell], 1e-30f));
                 return margin / 12.f;
             },
             MarginColor);

    // Panel 3: the image the beamformer forms.
    canvas.Text(left, bottom - 12,
                "RANGE X BEARING   " + Format("%+.0f", -0.5 * cfg.field_of_view_azimuth * CH_RAD_TO_DEG) +
                    " TO " + Format("%+.0f", 0.5 * cfg.field_of_view_azimuth * CH_RAD_TO_DEG) + " DEG   50 DB BELOW PEAK",
                label);
    draw_map(left, bottom, range_bins, azimuth_bins,
             [&](int r, int beam) {
                 const float power = m_range_azimuth[(size_t)beam * range_bins + r];
                 const float db = 10.f * std::log10(std::max(power, 1e-30f));
                 return (db - floor_db) / m_dynamic_range;
             },
             MagnitudeColor);

    // Panel 4: what the radar reports, in the plane.
    canvas.Fill(right, bottom, panel_w, panel_h, Rgba(22, 24, 30));
    canvas.Frame(right - 1, bottom - 1, panel_w + 2, panel_h + 2, faint);
    canvas.Text(right, bottom - 12, "PLAN VIEW   GREEN DIRECT   MAGENTA MULTIPATH   WHITE TRACK", label);

    const int origin_x = right + panel_w / 2;
    const int origin_y = bottom + panel_h - 4;
    const float scale = std::min(panel_w * 0.5f, (float)panel_h - 8) / m_plan_range;

    for (int ring = 1; ring * 20 <= (int)m_plan_range; ring++) {
        const int radius = (int)(ring * 20 * scale);
        for (int angle = 0; angle <= 180; angle += 2) {
            const float a = (float)angle * (float)CH_PI / 180.f;
            canvas.Set(origin_x + (int)(radius * std::cos(a)), origin_y - (int)(radius * std::sin(a)),
                       Rgba(45, 48, 58));
        }
    }
    // Field of view edges. The sensor frame is right handed with the lateral axis to the left, so
    // a target off to the right is drawn to the right of the boresight.
    for (int side = -1; side <= 1; side += 2) {
        const float edge = (float)(0.5 * cfg.field_of_view_azimuth);
        for (int step = 0; step < (int)(m_plan_range * scale); step++) {
            canvas.Set(origin_x + (int)(side * step * std::sin(edge)), origin_y - (int)(step * std::cos(edge)),
                       Rgba(45, 48, 58));
        }
    }

    // A detection whose dominant contributor bounced more than once was formed by an indirect
    // path. Sometimes that lands on the target anyway, through a ground bounce; sometimes it lands
    // where nothing is. Marking it is what lets one be told from the other.
    for (unsigned int i = 0; i < detections; i++) {
        const RadarDetection& d = m_detections[i];
        const int x = origin_x - (int)(d.range * std::sin(d.azimuth) * scale);
        const int y = origin_y - (int)(d.range * std::cos(d.azimuth) * scale);
        const bool multipath = (d.flags & RADAR_DET_MULTIPATH) != 0;
        canvas.Disc(x, y, 2, multipath ? Rgba(225, 90, 220) : Rgba(90, 225, 130));
    }

    for (const RadarObject& object : m_buffer->Objects) {
        const int x = origin_x - (int)(object.y * scale);
        const int y = origin_y - (int)(object.x * scale);
        canvas.Frame(x - 5, y - 5, 11, 11, Rgba(240, 240, 245));
        canvas.Text(x + 8, y - 3, Format("%.0f", (double)object.id), Rgba(240, 240, 245));
    }

    // Status line: everything needed to tell a quiet scene from a starved ray budget.
    char status[256];
    std::snprintf(status, sizeof(status), "paths %u   dropped %u   detections %u   tracks %u   t %.2f s",
                  m_buffer->NumPaths, m_buffer->DroppedPaths, detections, (unsigned int)m_buffer->Objects.size(),
                  (double)m_buffer->TimeStamp);
    canvas.Text(k_margin, m_h - 16, Upper(status), m_buffer->DroppedPaths ? Rgba(240, 160, 90) : label);
}

void ChFilterPhysRadarVisualize::Apply() {
    const size_t map_cells = m_power_map.size();

    radar_range_azimuth_map(m_buffer->AnglePower.get(), m_range_azimuth_device.get(), m_dsp, m_cuda_stream);
    cudaMemcpyAsync(m_power_map.data(), m_buffer->PowerMap.get(), map_cells * sizeof(float), cudaMemcpyDeviceToHost,
                    m_cuda_stream);
    cudaMemcpyAsync(m_threshold_map.data(), m_buffer->ThresholdMap.get(), map_cells * sizeof(float),
                    cudaMemcpyDeviceToHost, m_cuda_stream);
    cudaMemcpyAsync(m_range_azimuth.data(), m_range_azimuth_device.get(), m_range_azimuth.size() * sizeof(float),
                    cudaMemcpyDeviceToHost, m_cuda_stream);
    if (m_buffer->NumDetections > 0) {
        cudaMemcpyAsync(m_detections.data(), m_buffer->Detections.get(),
                        m_buffer->NumDetections * sizeof(RadarDetection), cudaMemcpyDeviceToHost, m_cuda_stream);
    }
    cudaStreamSynchronize(m_cuda_stream);

    Compose();

    if (!m_save_directory.empty()) {
        char filename[512];
        std::snprintf(filename, sizeof(filename), "%s/%s_%05u.png", m_save_directory.c_str(), Name().c_str(),
                      m_frame_index);
        stbi_write_png(filename, m_w, m_h, 4, m_image.data(), m_w * 4);
    }
    m_frame_index++;

#ifdef USE_SENSOR_GLFW
    GLContextGuard gl_context_guard;
    if (!m_window && !m_window_disabled)
        CreateGlfwWindow(Name());
    if (m_window_disabled || !m_window || glfwWindowShouldClose(m_window.get()))
        return;

    glfwMakeContextCurrent(m_window.get());
    int fb_w, fb_h;
    glfwGetFramebufferSize(m_window.get(), &fb_w, &fb_h);
    glViewport(0, 0, fb_w, fb_h);

    glBindTexture(GL_TEXTURE_2D, m_gl_tex_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_w, m_h, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_image.data());

    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    // The image is laid out with its first row at the top, so the vertical texture coordinate runs
    // the opposite way to the quad.
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(0.0f, 0.0f);
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(1.0f, 0.0f);
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(1.0f, 1.0f);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(0.0f, 1.0f);
    glEnd();
    glDisable(GL_TEXTURE_2D);

    glfwSwapBuffers(m_window.get());
    PollGlfwEventsPreservingHostEvents();
#endif
}

}  // namespace sensor
}  // namespace chrono
