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

#include "chrono_sensor/filters/ChFilterPhysRadarDetect.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <cuda_runtime.h>

#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

namespace {

/// Threshold multiplier of a cell averaging detector: with n training cells drawn from an
/// exponential distribution, this is the factor on their mean that yields the target false alarm
/// rate.
double CellAveragingFactor(unsigned int n, double false_alarm_rate) {
    if (n == 0)
        return 1.0;
    return n * (std::pow(false_alarm_rate, -1.0 / n) - 1.0);
}

/// Threshold multiplier of an ordered statistic detector. The false alarm rate is
/// prod_{i=0}^{k-1} (n-i)/(n-i+t), monotonically decreasing in t, so a bisection inverts it.
double OrderedStatisticFactor(unsigned int n, unsigned int k, double false_alarm_rate) {
    if (n == 0 || k == 0 || k > n)
        return 1.0;
    auto rate = [n, k](double t) {
        double p = 1.0;
        for (unsigned int i = 0; i < k; i++)
            p *= (double)(n - i) / ((double)(n - i) + t);
        return p;
    };
    double low = 0.0;
    double high = 1.0;
    while (rate(high) > false_alarm_rate && high < 1e12)
        high *= 2.0;
    for (int i = 0; i < 100; i++) {
        const double mid = 0.5 * (low + high);
        if (rate(mid) > false_alarm_rate)
            low = mid;
        else
            high = mid;
    }
    return 0.5 * (low + high);
}

}  // namespace

ChFilterPhysRadarDetect::ChFilterPhysRadarDetect(std::string name) : ChFilter(name) {}

ChFilterPhysRadarDetect::~ChFilterPhysRadarDetect() {}

void ChFilterPhysRadarDetect::Initialize(std::shared_ptr<ChSensor> pSensor,
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

    const unsigned int num_range = m_buffer->NumRangeBins;
    const unsigned int num_doppler = m_buffer->NumDopplerBins;
    const unsigned int num_channels = m_buffer->NumChannels;
    const unsigned int num_tx = cfg.GetNumTransmitters();
    const unsigned int num_rx = static_cast<unsigned int>(cfg.antenna.receivers.size());
    m_map_cells = num_range * num_doppler;

    std::vector<float3> positions(num_channels);
    for (unsigned int t = 0; t < num_tx; t++) {
        for (unsigned int r = 0; r < num_rx; r++) {
            const ChVector3d p = cfg.antenna.transmitters[t].position + cfg.antenna.receivers[r].position;
            positions[t * num_rx + r] = make_float3((float)p.x(), (float)p.y(), (float)p.z());
        }
    }
    m_channel_position = std::shared_ptr<float3[]>(cudaMallocHelper<float3>(num_channels), cudaFreeHelper<float3>);
    cudaMemcpy(m_channel_position.get(), positions.data(), num_channels * sizeof(float3), cudaMemcpyHostToDevice);

    m_steering = std::shared_ptr<float2[]>(
        cudaMallocHelper<float2>((size_t)cfg.dsp.num_azimuth_bins * num_channels), cudaFreeHelper<float2>);
    m_skirt_scratch = std::shared_ptr<float[]>(cudaMallocHelper<float>(m_map_cells), cudaFreeHelper<float>);
    m_detection_counter =
        std::shared_ptr<unsigned int[]>(cudaMallocHelper<unsigned int>(1), cudaFreeHelper<unsigned int>);

    const float azimuth_min = (float)(-0.5 * cfg.field_of_view_azimuth);
    const float azimuth_step = (float)(cfg.field_of_view_azimuth / cfg.dsp.num_azimuth_bins);
    radar_build_steering(m_channel_position.get(), num_channels, (float)cfg.GetWavelength(), azimuth_min, azimuth_step,
                         cfg.dsp.num_azimuth_bins, m_steering.get(), m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);

    const unsigned int train_cells = cfg.GetCfarTrainingCellCount();
    const unsigned int rank =
        std::max(1u, std::min(train_cells, (unsigned int)std::lround(cfg.dsp.os_rank_fraction * train_cells)));

    m_dsp = {};
    m_dsp.num_range_bins = num_range;
    m_dsp.num_doppler_bins = num_doppler;
    m_dsp.num_channels = num_channels;
    m_dsp.num_azimuth_bins = cfg.dsp.num_azimuth_bins;
    m_dsp.range_bin_size = m_buffer->RangeBinSize;
    m_dsp.doppler_bin_size = m_buffer->DopplerBinSize;
    m_dsp.azimuth_min = azimuth_min;
    m_dsp.azimuth_step = azimuth_step;
    m_dsp.min_range = (float)cfg.min_range;
    m_dsp.cfar_type = (cfg.dsp.cfar_type == ChRadarCfarType::OS) ? 1 : 0;
    m_dsp.guard_range = cfg.dsp.cfar_guard_range;
    m_dsp.guard_doppler = cfg.dsp.cfar_guard_doppler;
    m_dsp.train_range = cfg.dsp.cfar_train_range;
    m_dsp.train_doppler = cfg.dsp.cfar_train_doppler;
    m_dsp.os_rank = rank;
    m_dsp.threshold_factor = (cfg.dsp.cfar_type == ChRadarCfarType::OS)
                                 ? (float)OrderedStatisticFactor(train_cells, rank, cfg.dsp.false_alarm_rate)
                                 : (float)CellAveragingFactor(train_cells, cfg.dsp.false_alarm_rate);
    m_dsp.max_detections = cfg.dsp.max_detections;
    m_dsp.noise_floor = m_buffer->NoisePowerPerCell;

    const double transmit_power = std::pow(10.0, cfg.waveform.transmit_power_dbm / 10.0) * 1e-3;
    const double wavelength = cfg.GetWavelength();
    m_dsp.rcs_scale = (float)(std::pow(4.0 * CH_PI, 3.0) / (transmit_power * wavelength * wavelength));
    const ChRadarElementConfig& tx = cfg.antenna.transmitters.front();
    const ChRadarElementConfig& rx = cfg.antenna.receivers.front();
    m_dsp.tx_gain = (float)std::pow(10.0, tx.gain_dbi / 10.0);
    m_dsp.tx_az_exponent = (float)ChRadarPatternExponent(tx.azimuth_beamwidth);
    m_dsp.tx_el_exponent = (float)ChRadarPatternExponent(tx.elevation_beamwidth);
    m_dsp.rx_gain = (float)std::pow(10.0, rx.gain_dbi / 10.0);
    m_dsp.rx_az_exponent = (float)ChRadarPatternExponent(rx.azimuth_beamwidth);
    m_dsp.rx_el_exponent = (float)ChRadarPatternExponent(rx.elevation_beamwidth);
    m_dsp.window_peak_product = (float)(cfg.GetRangeWindowPeak() * cfg.GetDopplerWindowPeak());
    m_dsp.suppress_stationary = cfg.dsp.suppress_stationary ? 1 : 0;
    m_dsp.stationary_tolerance = std::max((float)cfg.dsp.stationary_tolerance, m_buffer->DopplerBinSize);
    m_dsp.max_unambiguous_velocity = m_buffer->MaxUnambiguousVelocity;

    m_apply_phase_noise = cfg.impairments.enable_phase_noise;
    if (m_apply_phase_noise) {
        m_phase_noise = {};
        m_phase_noise.num_range_bins = num_range;
        m_phase_noise.num_doppler_bins = num_doppler;
        // The skirt covers the whole range axis. Truncating it would leave a step where the window
        // ends, and a detector whose training cells straddle that step reports the step.
        m_phase_noise.skirt_half_width = num_range;
        m_phase_noise.range_bin_size = m_buffer->RangeBinSize;
        m_phase_noise.beat_frequency_per_bin = (float)(cfg.waveform.sample_rate / cfg.waveform.num_samples);
        m_phase_noise.level_at_reference = (float)std::pow(10.0, cfg.impairments.phase_noise_dbc_hz / 10.0);
        m_phase_noise.reference_offset = (float)cfg.impairments.phase_noise_offset;
        m_phase_noise.decade_slope = (float)cfg.impairments.phase_noise_decade_slope;
    }

    bufferInOut = m_buffer;
}

void ChFilterPhysRadarDetect::Apply() {
    radar_beamform(m_buffer->Cube.get(), m_steering.get(), m_buffer->AnglePower.get(), m_buffer->PowerMap.get(), m_dsp,
                   m_cuda_stream);

    // Impairments that behave like noise are added to the power map rather than to the cube: they
    // have no bearing of their own, and a detector that sees them in its training cells raises its
    // threshold exactly as a real one does.
    if (m_apply_phase_noise) {
        radar_add_phase_noise_skirt(m_buffer->PowerMap.get(), m_skirt_scratch.get(), m_phase_noise, m_cuda_stream);
        cudaMemcpyAsync(m_buffer->PowerMap.get(), m_skirt_scratch.get(), (size_t)m_map_cells * sizeof(float),
                        cudaMemcpyDeviceToDevice, m_cuda_stream);
    }
    if (m_buffer->QuantizationFloor > 0.f)
        radar_add_uniform_floor(m_buffer->PowerMap.get(), m_map_cells, m_buffer->QuantizationFloor, m_cuda_stream);

    radar_cfar(m_buffer->PowerMap.get(), m_buffer->ThresholdMap.get(), m_dsp, m_cuda_stream);

    m_dsp.ego_speed = (float)m_radar->GetTranslationalVelocity().Length();
    radar_extract_detections(m_buffer->PowerMap.get(), m_buffer->ThresholdMap.get(), m_buffer->AnglePower.get(),
                             m_buffer->Provenance.get(), m_buffer->Detections.get(), m_detection_counter.get(), m_dsp,
                             m_cuda_stream);

    unsigned int found = 0;
    cudaMemcpyAsync(&found, m_detection_counter.get(), sizeof(unsigned int), cudaMemcpyDeviceToHost, m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);
    m_buffer->NumDetections = std::min(found, m_buffer->DetectionCapacity);
}

}  // namespace sensor
}  // namespace chrono
