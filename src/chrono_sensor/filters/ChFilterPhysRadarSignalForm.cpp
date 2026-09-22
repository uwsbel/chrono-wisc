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

#include "chrono_sensor/filters/ChFilterPhysRadarSignalForm.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <cuda_runtime.h>

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/cuda/curand_utils.cuh"
#include "chrono_sensor/cuda/radar_impair.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

namespace {

int WindowCode(ChRadarWindowType type) {
    switch (type) {
        case ChRadarWindowType::HANN:
            return 1;
        case ChRadarWindowType::HAMMING:
            return 2;
        case ChRadarWindowType::RECTANGULAR:
        default:
            return 0;
    }
}

}  // namespace

ChFilterPhysRadarSignalForm::ChFilterPhysRadarSignalForm(std::string name) : ChFilter(name) {}

ChFilterPhysRadarSignalForm::~ChFilterPhysRadarSignalForm() {}

void ChFilterPhysRadarSignalForm::Initialize(std::shared_ptr<ChSensor> pSensor,
                                             std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_radar = std::dynamic_pointer_cast<ChPhysRadarSensor>(pSensor);
    if (!m_radar)
        InvalidFilterGraphSensorTypeMismatch(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDevicePhysRadarPathBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    m_cuda_stream = m_radar->GetCudaStream();
    const ChRadarModelConfig& cfg = m_radar->GetConfig();

    const unsigned int num_range = cfg.GetNumRangeBins();
    const unsigned int num_doppler = cfg.GetNumDopplerBins();
    const unsigned int num_channels = cfg.GetNumVirtualChannels();
    const unsigned int num_tx = cfg.GetNumTransmitters();
    const unsigned int num_rx = static_cast<unsigned int>(cfg.antenna.receivers.size());

    m_cube_cells = num_channels * num_doppler * num_range;
    m_map_cells = num_doppler * num_range;

    // Virtual array: a transmit and a receive element separated by the sum of their positions are
    // indistinguishable from one element at that sum, which is what multiplies the aperture.
    std::vector<float3> positions(num_channels);
    std::vector<float> gains(num_channels, 1.f);
    std::vector<unsigned int> slots(num_channels, 0u);
    for (unsigned int t = 0; t < num_tx; t++) {
        for (unsigned int r = 0; r < num_rx; r++) {
            const ChVector3d p = cfg.antenna.transmitters[t].position + cfg.antenna.receivers[r].position;
            const unsigned int c = t * num_rx + r;
            positions[c] = make_float3((float)p.x(), (float)p.y(), (float)p.z());
            slots[c] = (cfg.antenna.multiplex == ChRadarMultiplex::TDM) ? t : 0u;
        }
    }

    m_channel_position = std::shared_ptr<float3[]>(cudaMallocHelper<float3>(num_channels), cudaFreeHelper<float3>);
    m_channel_gain = std::shared_ptr<float[]>(cudaMallocHelper<float>(num_channels), cudaFreeHelper<float>);
    m_channel_slot =
        std::shared_ptr<unsigned int[]>(cudaMallocHelper<unsigned int>(num_channels), cudaFreeHelper<unsigned int>);
    cudaMemcpy(m_channel_position.get(), positions.data(), num_channels * sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(m_channel_gain.get(), gains.data(), num_channels * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(m_channel_slot.get(), slots.data(), num_channels * sizeof(unsigned int), cudaMemcpyHostToDevice);

    const double wavelength = cfg.GetWavelength();

    m_deposit = {};
    m_deposit.num_range_bins = num_range;
    m_deposit.num_doppler_bins = num_doppler;
    m_deposit.num_channels = num_channels;
    m_deposit.range_transform_length = cfg.waveform.num_samples;
    m_deposit.taps = 3;
    m_deposit.wavelength = (float)wavelength;
    m_deposit.range_bin_size = (float)cfg.GetRangeResolution();
    m_deposit.doppler_bin_size = (float)cfg.GetVelocityResolution();
    // The dechirped beat frequency carries the Doppler shift alongside the delay.
    m_deposit.doppler_range_coupling =
        (float)(2.0 * cfg.waveform.num_samples / (wavelength * cfg.waveform.sample_rate));
    // A transmitter that starts one slot late sees the target's slow-time phase advanced by one
    // chirp's worth. Leaving this out is what makes an uncompensated time division array report a
    // bearing error that grows with the target's speed.
    m_deposit.tdm_slot_phase = (float)(2.0 * CH_2PI * cfg.waveform.chirp_repetition_interval / wavelength);
    m_deposit.range_window = WindowCode(cfg.dsp.range_window);
    m_deposit.doppler_window = WindowCode(cfg.dsp.doppler_window);
    m_deposit.channel_position = m_channel_position.get();
    m_deposit.channel_gain = m_channel_gain.get();
    m_deposit.channel_slot = m_channel_slot.get();

    m_noise_power_per_cell = (float)cfg.GetNoisePowerPerCell();

    // Converter noise is white in time, so it spreads evenly over the cube exactly as thermal
    // noise does. Full scale is taken three standard deviations above the frame's total power,
    // which is the headroom an automatic gain control would leave.
    const double levels = std::pow(2.0, cfg.impairments.adc_bits);
    m_quantization_scale =
        cfg.impairments.enable_quantization
            ? (float)(9.0 / (12.0 * levels * levels) * cfg.GetWindowPowerProduct())
            : 0.f;

    if (cfg.impairments.enable_transmit_leakage) {
        const double transmit_power = std::pow(10.0, cfg.waveform.transmit_power_dbm / 10.0) * 1e-3;
        const double leakage_power = transmit_power * std::pow(10.0, cfg.impairments.transmit_leakage_db / 10.0);
        RadarPath leakage = {};
        leakage.length = 0.f;
        leakage.length_rate = 0.f;
        leakage.amplitude = (float)std::sqrt(leakage_power);
        leakage.phase = 0.f;
        // Broadside arrival, so the coupling sits on the array's boresight rather than steering it.
        leakage.dir_rx = make_float3(1.f, 0.f, 0.f);
        leakage.bounces = 0;
        leakage.object_id = 0;
        leakage.flags = 0;
        m_leakage_path = std::shared_ptr<RadarPath[]>(cudaMallocHelper<RadarPath>(1), cudaFreeHelper<RadarPath>);
        cudaMemcpy(m_leakage_path.get(), &leakage, sizeof(RadarPath), cudaMemcpyHostToDevice);
        m_has_leakage = true;
    }

    if (cfg.impairments.enable_thermal_noise) {
        m_num_noise_states = std::min(m_cube_cells, 65536u);
        m_noise_rng = std::shared_ptr<curandState_t[]>(cudaMallocHelper<curandState_t>(m_num_noise_states),
                                                       cudaFreeHelper<curandState_t>);
        init_cuda_rng(ChSensorManager::GetDeterministicSeed(pSensor, RngUsage::RadarThermalNoise, GetRngStreamIndex()),
                      m_noise_rng.get(), m_num_noise_states);
    }

    m_total_power = std::shared_ptr<float[]>(cudaMallocHelper<float>(1), cudaFreeHelper<float>);

    auto frame = chrono_types::make_shared<SensorDevicePhysRadarFrame>();
    frame->Width = num_range;
    frame->Height = num_doppler;
    frame->NumRangeBins = num_range;
    frame->NumDopplerBins = num_doppler;
    frame->NumChannels = num_channels;
    frame->NumAzimuthBins = cfg.dsp.num_azimuth_bins;
    frame->DetectionCapacity = cfg.dsp.max_detections;
    frame->RangeBinSize = m_deposit.range_bin_size;
    frame->DopplerBinSize = m_deposit.doppler_bin_size;
    frame->AzimuthBinSize = (float)(cfg.field_of_view_azimuth / cfg.dsp.num_azimuth_bins);
    frame->NoisePowerPerCell = m_noise_power_per_cell;
    frame->MaxUnambiguousVelocity = (float)cfg.GetMaxUnambiguousVelocity();
    frame->Wavelength = (float)wavelength;

    frame->Cube = std::shared_ptr<RadarComplex[]>(cudaMallocHelper<RadarComplex>(m_cube_cells),
                                                  cudaFreeHelper<RadarComplex>);
    frame->AnglePower = std::shared_ptr<float[]>(
        cudaMallocHelper<float>((size_t)cfg.dsp.num_azimuth_bins * m_map_cells), cudaFreeHelper<float>);
    frame->PowerMap = std::shared_ptr<float[]>(cudaMallocHelper<float>(m_map_cells), cudaFreeHelper<float>);
    frame->ThresholdMap = std::shared_ptr<float[]>(cudaMallocHelper<float>(m_map_cells), cudaFreeHelper<float>);
    frame->Provenance = std::shared_ptr<unsigned long long[]>(cudaMallocHelper<unsigned long long>(m_map_cells),
                                                              cudaFreeHelper<unsigned long long>);
    frame->Detections = std::shared_ptr<RadarDetection[]>(cudaMallocHelper<RadarDetection>(cfg.dsp.max_detections),
                                                          cudaFreeHelper<RadarDetection>);

    m_buffer_out = frame;
    bufferInOut = m_buffer_out;
}

void ChFilterPhysRadarSignalForm::Apply() {
    // The ray tracer counts what it wrote; the count has to come back before the deposit can be
    // sized. This is the one synchronization point in the chain.
    unsigned int counter[2] = {0, 0};
    cudaMemcpyAsync(counter, m_buffer_in->Counter, 2 * sizeof(unsigned int), cudaMemcpyDeviceToHost, m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);

    const unsigned int written = std::min(counter[0], m_buffer_in->Capacity);
    m_buffer_in->NumPaths = written;
    m_buffer_in->Dropped = counter[1];

    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->NumPaths = written;
    m_buffer_out->DroppedPaths = counter[1];
    m_buffer_out->NumDetections = 0;
    m_buffer_out->Objects.clear();

    radar_clear_cube(m_buffer_out->Cube.get(), m_cube_cells, m_cuda_stream);
    cudaMemsetAsync(m_buffer_out->Provenance.get(), 0, (size_t)m_map_cells * sizeof(unsigned long long), m_cuda_stream);

    radar_total_path_power(m_buffer_in->Buffer.get(), written, m_total_power.get(), m_cuda_stream);
    radar_deposit_paths(m_buffer_in->Buffer.get(), written, m_buffer_out->Cube.get(), m_buffer_out->Provenance.get(),
                        m_deposit, m_cuda_stream);
    if (m_has_leakage) {
        radar_deposit_paths(m_leakage_path.get(), 1, m_buffer_out->Cube.get(), m_buffer_out->Provenance.get(),
                            m_deposit, m_cuda_stream);
    }

    float total_power = 0.f;
    cudaMemcpyAsync(&total_power, m_total_power.get(), sizeof(float), cudaMemcpyDeviceToHost, m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);
    m_buffer_out->TotalPathPower = total_power;
    m_buffer_out->QuantizationFloor = m_quantization_scale * total_power;

    if (m_noise_rng) {
        radar_add_thermal_noise(m_buffer_out->Cube.get(), m_cube_cells, m_noise_power_per_cell, m_noise_rng.get(),
                                m_num_noise_states, m_cuda_stream);
    }

    // Clear the tracer's counter for the next launch. The launch that fills it runs earlier in
    // this sensor's stream, so zeroing it here is ordered ahead of the next one.
    cudaMemsetAsync(m_buffer_in->Counter, 0, 2 * sizeof(unsigned int), m_cuda_stream);
}

}  // namespace sensor
}  // namespace chrono
