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

#include "chrono_sensor/filters/ChFilterPhysRadarAccess.h"

#include <cmath>

#include <cuda_runtime.h>

#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace sensor {

ChFilterPhysRadarAccess::ChFilterPhysRadarAccess(ChRadarFrameContents contents, std::string name)
    : ChFilter(name), m_contents(contents) {}

ChFilterPhysRadarAccess::~ChFilterPhysRadarAccess() {}

void ChFilterPhysRadarAccess::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDevicePhysRadarFrame>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto optix_sensor = std::dynamic_pointer_cast<ChOptixSensor>(pSensor))
        m_cuda_stream = optix_sensor->GetCudaStream();

    m_sensor = pSensor;
    m_max_lag_frames =
        1 + (unsigned int)std::ceil((pSensor->GetLag() + pSensor->GetCollectionWindow()) * pSensor->GetUpdateRate());
    m_user_frame = chrono_types::make_shared<SensorHostPhysRadarFrame>();
}

std::shared_ptr<SensorHostPhysRadarFrame> ChFilterPhysRadarAccess::MakeHostFrame() const {
    auto frame = chrono_types::make_shared<SensorHostPhysRadarFrame>();
    const SensorDevicePhysRadarFrame& src = *m_buffer_in;

    frame->TimeStamp = src.TimeStamp;
    frame->Width = src.Width;
    frame->Height = src.Height;
    frame->LaunchedCount = src.LaunchedCount;
    frame->NumRangeBins = src.NumRangeBins;
    frame->NumDopplerBins = src.NumDopplerBins;
    frame->NumChannels = src.NumChannels;
    frame->NumAzimuthBins = src.NumAzimuthBins;
    frame->NumDetections = src.NumDetections;
    frame->DetectionCapacity = src.NumDetections;
    frame->RangeBinSize = src.RangeBinSize;
    frame->DopplerBinSize = src.DopplerBinSize;
    frame->AzimuthBinSize = src.AzimuthBinSize;
    frame->NoisePowerPerCell = src.NoisePowerPerCell;
    frame->MaxUnambiguousVelocity = src.MaxUnambiguousVelocity;
    frame->Wavelength = src.Wavelength;
    frame->NumPaths = src.NumPaths;
    frame->DroppedPaths = src.DroppedPaths;
    frame->TotalPathPower = src.TotalPathPower;
    frame->QuantizationFloor = src.QuantizationFloor;
    frame->Objects = src.Objects;

    const size_t map_cells = (size_t)src.NumRangeBins * src.NumDopplerBins;

    if (src.NumDetections > 0) {
        frame->Detections = std::shared_ptr<RadarDetection[]>(new RadarDetection[src.NumDetections]);
        cudaMemcpyAsync(frame->Detections.get(), src.Detections.get(), src.NumDetections * sizeof(RadarDetection),
                        cudaMemcpyDeviceToHost, m_cuda_stream);
    }

    if (m_contents != ChRadarFrameContents::DETECTIONS) {
        frame->PowerMap = std::shared_ptr<float[]>(new float[map_cells]);
        frame->ThresholdMap = std::shared_ptr<float[]>(new float[map_cells]);
        frame->Provenance = std::shared_ptr<unsigned long long[]>(new unsigned long long[map_cells]);
        cudaMemcpyAsync(frame->PowerMap.get(), src.PowerMap.get(), map_cells * sizeof(float), cudaMemcpyDeviceToHost,
                        m_cuda_stream);
        cudaMemcpyAsync(frame->ThresholdMap.get(), src.ThresholdMap.get(), map_cells * sizeof(float),
                        cudaMemcpyDeviceToHost, m_cuda_stream);
        cudaMemcpyAsync(frame->Provenance.get(), src.Provenance.get(), map_cells * sizeof(unsigned long long),
                        cudaMemcpyDeviceToHost, m_cuda_stream);
    }

    if (m_contents == ChRadarFrameContents::FULL) {
        const size_t cube_cells = map_cells * src.NumChannels;
        const size_t angle_cells = map_cells * src.NumAzimuthBins;
        frame->Cube = std::shared_ptr<RadarComplex[]>(new RadarComplex[cube_cells]);
        frame->AnglePower = std::shared_ptr<float[]>(new float[angle_cells]);
        cudaMemcpyAsync(frame->Cube.get(), src.Cube.get(), cube_cells * sizeof(RadarComplex), cudaMemcpyDeviceToHost,
                        m_cuda_stream);
        cudaMemcpyAsync(frame->AnglePower.get(), src.AnglePower.get(), angle_cells * sizeof(float),
                        cudaMemcpyDeviceToHost, m_cuda_stream);
    }

    cudaStreamSynchronize(m_cuda_stream);
    return frame;
}

void ChFilterPhysRadarAccess::Apply() {
    auto frame = MakeHostFrame();

    std::lock_guard<std::mutex> lock(m_mutex);
    m_lag_frames.push(frame);
    while (m_lag_frames.size() > m_max_lag_frames)
        m_lag_frames.pop();
}

UserPhysRadarFramePtr ChFilterPhysRadarAccess::GetBuffer() {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto sensor = m_sensor.lock();
    const float now = (float)sensor->GetParent()->GetSystem()->GetChTime();
    while (!m_lag_frames.empty() && now > m_lag_frames.front()->TimeStamp + sensor->GetLag() - 1e-7f) {
        m_user_frame = m_lag_frames.front();
        m_lag_frames.pop();
    }
    return m_user_frame;
}

}  // namespace sensor
}  // namespace chrono
