// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nevindu M. Batagoda
// =============================================================================
//
// =============================================================================

#ifndef CUDAUTILS_H
#define CUDAUTILS_H

    #include <cuda_fp16.h>

#ifdef USE_SENSOR_NVDB



#include <nanovdb/util/CreateNanoGrid.h> // converter from OpenVDB to NanoVDB (includes NanoVDB.h and GridManager.h)
#include <nanovdb/util/cuda/CudaDeviceBuffer.h>
#include <nanovdb/util/NodeManager.h>

namespace chrono {
namespace sensor {


__hostdev__ struct DustParticle {
    //__hostdev__ DustParticle() {}
    //__hostdev__  DustParticle(float3 pos, float3 vel, float t) : pos(pos), vel(vel), pos0(pos), t0(t), mass(0.1f), isActive(true) {}
    float3 pos;
    float3 vel;
    float3 pos0;
    float3 vel0;
    int lifespan;
    bool isActive;
    float t0;
};


void cuda_convert_float_buffer_to_NVDBVec3f(void* input, void* output, int n);

void computePCA(const std::vector<nanovdb::Vec3f>& neighbors, nanovdb::Vec3f& normal);

__global__ void computeNormalsKernel(const nanovdb::Coord* d_coords,
                                     nanovdb::Vec3f* d_normals,
                                     int numVoxels,
                                     int windowSize,
                                     nanovdb::NanoGrid<nanovdb::Point>* grid);


void createNanoVDBGridHandle(void* h_points_buffer,
                             int n,
                             const double voxel_size,
                             int window_size,
                             nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& handle,
                             nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& normalHandle,
                             nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& densityHandle);

nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> addVDBVolume(std::shared_ptr<openvdb::FloatGrid> openVDBgrid); 

void createDustGrid(nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& dustHandle, float* h_points,
                    int n,
                    float threshold_vel,
                    void** dust_particles_ptr,
                    int& num_dust_particles,
                    float time,
                    int& frame,
                    float z_thresh);

template <typename T>
void printGridInformation(nanovdb::NanoGrid<T>* grid,
                          nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& handle,
                          std::string grid_name);
}  // namespace sensor
}  // namespace chrono

#endif


void initializeBuffer(void* frame, int w, int h);
void initializeBuffer(void* frame, void* albedo, void* normal, int w, int h);

#endif
