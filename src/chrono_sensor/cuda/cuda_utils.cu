
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

#include <cuda.h>
#include <cuda_runtime.h>
#include <vector>
#include "cuda_utils.cuh"

#ifdef USE_SENSOR_NVDB




#include <nanovdb/NanoVDB.h>
#include <openvdb/openvdb.h>

#include <nanovdb/util/GridHandle.h>
#include <nanovdb/util/cuda/CudaPointsToGrid.cuh>

#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/ParticlesToLevelSet.h>
#include <openvdb/tools/LevelSetSphere.h> // replace with your own dependencies for generating the OpenVDB grid
//#include <nanovdb/util/CreateNanoGrid.h> // converter from OpenVDB to NanoVDB (includes NanoVDB.h and GridManager.h)
#include <nanovdb/util/cuda/CudaDeviceBuffer.h>



namespace chrono {
namespace sensor {



class MyParticleList {
  protected:
    struct MyParticle {
        openvdb::Vec3R p, v;
        openvdb::Real r;
    };
    openvdb::Real mRadiusScale;
    openvdb::Real mVelocityScale;
    std::vector<MyParticle> mParticleList;

  public:
    typedef openvdb::Vec3R PosType;

    MyParticleList(openvdb::Real rScale = 1, openvdb::Real vScale = 1) : mRadiusScale(rScale), mVelocityScale(vScale) {}
    void add(const openvdb::Vec3R& p, const openvdb::Real& r, const openvdb::Vec3R& v = openvdb::Vec3R(0, 0, 0)) {
        MyParticle pa;
        pa.p = p;
        pa.r = r;
        pa.v = v;
        mParticleList.push_back(pa);
    }
    /// @return coordinate bbox in the space of the specified transfrom
    openvdb::CoordBBox getBBox(const openvdb::GridBase& grid) {
        openvdb::CoordBBox bbox;
        openvdb::Coord &min = bbox.min(), &max = bbox.max();
        openvdb::Vec3R pos;
        openvdb::Real rad, invDx = 1 / grid.voxelSize()[0];
        for (size_t n = 0, e = this->size(); n < e; ++n) {
            this->getPosRad(n, pos, rad);
            const openvdb::Vec3d xyz = grid.worldToIndex(pos);
            const openvdb::Real r = rad * invDx;
            for (int i = 0; i < 3; ++i) {
                min[i] = openvdb::math::Min(min[i], openvdb::math::Floor(xyz[i] - r));
                max[i] = openvdb::math::Max(max[i], openvdb::math::Ceil(xyz[i] + r));
            }
        }
        return bbox;
    }
    // typedef int AttributeType;
    // The methods below are only required for the unit-tests
    openvdb::Vec3R pos(int n) const { return mParticleList[n].p; }
    openvdb::Vec3R vel(int n) const { return mVelocityScale * mParticleList[n].v; }
    openvdb::Real radius(int n) const { return mRadiusScale * mParticleList[n].r; }

    //////////////////////////////////////////////////////////////////////////////
    /// The methods below are the only ones required by tools::ParticleToLevelSet
    /// @note We return by value since the radius and velocities are modified
    /// by the scaling factors! Also these methods are all assumed to
    /// be thread-safe.

    /// Return the total number of particles in list.
    ///  Always required!
    size_t size() const { return mParticleList.size(); }

    /// Get the world space position of n'th particle.
    /// Required by ParticledToLevelSet::rasterizeSphere(*this,radius).
    void getPos(size_t n, openvdb::Vec3R& pos) const { pos = mParticleList[n].p; }

    void getPosRad(size_t n, openvdb::Vec3R& pos, openvdb::Real& rad) const {
        pos = mParticleList[n].p;
        rad = mRadiusScale * mParticleList[n].r;
    }
    void getPosRadVel(size_t n, openvdb::Vec3R& pos, openvdb::Real& rad, openvdb::Vec3R& vel) const {
        pos = mParticleList[n].p;
        rad = mRadiusScale * mParticleList[n].r;
        vel = mVelocityScale * mParticleList[n].v;
    }
    // The method below is only required for attribute transfer
    void getAtt(size_t n, openvdb::Index32& att) const { att = openvdb::Index32(n); }
};


__global__ void convertFloatBufferToNVDBVec3F(float* input, nanovdb::Vec3f* output, int n) {
   int idx = blockIdx.x * blockDim.x + threadIdx.x;
   if (idx < n) {
       output[idx] = nanovdb::Vec3f(input[6*idx], input[6*idx + 1], input[6*idx + 2]);
   }
}

void cuda_convert_float_buffer_to_NVDBVec3f(void* input, void* output, int n) {
   int blockSize = 256;
   int gridSize = (n + blockSize - 1) / blockSize;
   convertFloatBufferToNVDBVec3F<<<gridSize, blockSize>>>((float*)input, (nanovdb::Vec3f*)output, n);
   cudaError_t err = cudaGetLastError();
   if (err != cudaSuccess) {
       fprintf(stderr, "CUDA kernel launch error: %s\n", cudaGetErrorString(err));
       // Handle the error...
   }
   err = cudaDeviceSynchronize();
   if (err != cudaSuccess) {
       fprintf(stderr, "CUDA kernel execution error: %s\n", cudaGetErrorString(err));
       // Handle the error...
   }
}

nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> createNanoVDBGridHandle(void* h_points_buffer, int n) {


    // Loop over the particles and add them to MyParticleList
    MyParticleList pa;
    const openvdb::Vec3R vel(10, 5, 1);
    pa.add(openvdb::Vec3R(84.7252, 85.7946, 84.4266), 11.8569, vel);
    pa.add(openvdb::Vec3R(47.9977, 81.2169, 47.7665), 5.45313, vel);
    pa.add(openvdb::Vec3R(87.0087, 14.0351, 95.7155), 7.36483, vel);
    pa.add(openvdb::Vec3R(75.8616, 53.7373, 58.202), 14.4127, vel);
    pa.add(openvdb::Vec3R(14.9675, 32.4141, 13.5218), 4.33101, vel);
    pa.add(openvdb::Vec3R(96.9809, 9.92804, 90.2349), 12.2613, vel);
    pa.add(openvdb::Vec3R(63.4274, 3.84254, 32.5047), 12.1566, vel);
    pa.add(openvdb::Vec3R(62.351, 47.4698, 41.4369), 11.637, vel);
    pa.add(openvdb::Vec3R(62.2846, 1.35716, 66.2527), 18.9914, vel);
    pa.add(openvdb::Vec3R(44.1711, 1.99877, 45.1159), 1.11429, vel);
    //for (int i = 0; i < n; i++) {
    //    pa.add(openvdb::Vec3R(((float*)h_points_buffer)[6 * i], ((float*)h_points_buffer)[6 * i + 1], ((float*)h_points_buffer)[6 * i + 2]), 0.1, openvdb::Vec3R(((float*)h_points_buffer)[6 * i + 3], ((float*)h_points_buffer)[6 * i] + 4, ((float*)h_points_buffer)[6 * i] + 5));
    //    //pa.add(openvdb::Vec3R(1*((float*)h_points_buffer)[6 * i], 1*((float*)h_points_buffer)[6 * i + 1], 1*((float*)h_points_buffer)[6 * i + 2]), 1, vel);
    //}

    // print particles in MyParticleList
    // for (size_t n = 0, e = pa.size(); n < e; ++n) {
    //     openvdb::Vec3R pos;
    //     openvdb::Real rad;
    //     openvdb::Vec3R vel;
    //     pa.getPosRadVel(n, pos, rad, vel);
    //     printf("Particle %d: pos = (%f, %f, %f), rad = %f, vel = (%f, %f, %f)\n", (int)n, pos[0], pos[1], pos[2], rad, vel[0], vel[1], vel[2]);
    // }

    //auto bbox = pa.getBBox();
    //printf("Num Parricles: %d\n", pa.size());

    auto sdf = openvdb::createLevelSet<openvdb::FloatGrid>(0.1);
    openvdb::tools::particlesToSdf<openvdb::FloatGrid, MyParticleList>(pa, *sdf);


    //auto sdf = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(100.0f, openvdb::Vec3f(0.0f), 1.0f);
    auto handle1 = nanovdb::createNanoGrid<openvdb::FloatGrid, float, nanovdb::CudaDeviceBuffer>(*sdf);
    printf("OpenVDB to NanoVDB conversion done!\n");

    if (handle1.gridMetaData()->isLevelSet() == false) {
        throw std::runtime_error("Grid must be a level set");
    }
    
    //nanovdb::build::NodeManager<nanovdb::build::Grid<float>> mgr(*grid1);
    //nanovdb::build::sdfToLevelSet(mgr);

    //auto handle = nanovdb::createNanoGrid<nanovdb::build::Grid<float>, float, nanovdb::CudaDeviceBuffer>(*grid1);

    cudaStream_t stream;  // Create a CUDA stream to allow for asynchronous copy of pinned CUDA memory.
    cudaStreamCreate(&stream);

    handle1.deviceUpload(stream, false);  // Copy the NanoVDB grid to the GPU asynchronously
    //auto* grid1 = handle1.grid<float>();  // get a (raw) pointer to a NanoVDB grid of value type float on the CPU

    //nanovdb::DefaultReadAccessor<float> acc = grid1->tree().getAccessor();
    //nanovdb::CoordBBox bbox = acc.root().bbox();
    //nanovdb::CoordBBox IdxBbox = grid1->indexBBox();
 
    
    //printf("############### VDB GRID INFORMATION ################\n");
    //printf("Grid Size: %d\n", grid1->gridSize());
    //printf("Grid Class: %s\n", nanovdb::toStr(handle1.gridMetaData()->gridClass()));
    //printf("Grid Type: %s\n", nanovdb::toStr(handle1.gridType(0)));
    //printf("Upper Internal Nodes: %d\n", grid1->tree().nodeCount(2));
    //printf("Lower Internal Nodes: %d\n", grid1->tree().nodeCount(1));
    //printf("Leaf Nodes: %d\n", grid1->tree().nodeCount(0));
    //printf("Voxel Size: %f\n", float(grid1->voxelSize()[0]));
    //printf("Active Voxels: %d\n", grid1->activeVoxelCount());
    //printf("World BBox: %f %f %f\n", grid1->worldBBox().dim()[0], grid1->worldBBox().dim()[1], grid1->worldBBox().dim()[2]);
    //printf("World Bbox Max: %f %f %f | Min: %f %f %f\n", grid1->worldBBox().max()[0], grid1->worldBBox().max()[1],
    //       grid1->worldBBox().max()[2], grid1->worldBBox().min()[0], grid1->worldBBox().min()[1],grid1->worldBBox().min()[2]);
    //printf("BBox: min:(%d,%d,%d)| max:(%d,%d,%d)\n", bbox.min()[0], bbox.min()[1], bbox.min()[2], bbox.max()[0],bbox.max()[1],bbox.max()[2]);
    //printf("IndexBBox: min:(%d,%d,%d)| max:(%d,%d,%d)\n", IdxBbox.min()[0], IdxBbox.min()[1], IdxBbox.min()[2],
    //       bbox.max()[0],bbox.max()[1], bbox.max()[2]);
    //printf("############### END #############\n");

    cudaDeviceSynchronize();
    cudaStreamDestroy(stream);
    /*
    float* d_points_float_buffer = nullptr;
    cudaMalloc((void**)&d_points_float_buffer, sizeof(float) * 6 * n);
    cudaMemcpy(d_points_float_buffer, h_points_buffer, sizeof(float) * 6 * n, cudaMemcpyHostToDevice);


    //// Loop over the particles and add them to 
    using BuildT = nanovdb::Point;//uint32_t;
    using Vec3T = nanovdb::Vec3f;



    nanovdb::Vec3f* d_points = nullptr;
    cudaMalloc((void**)&d_points, sizeof(nanovdb::Vec3f) * n);
 
    cuda_convert_float_buffer_to_NVDBVec3f(d_points_float_buffer, d_points, n);


    const double voxelSize = 0.01;
    //printf("Creating NanoVDB Grid with %d points!\n", n);
    nanovdb::CudaPointsToGrid<BuildT> converter(voxelSize);  // unit map
    converter.setPointType(nanovdb::PointType::World32);
    //converter.setVerbose();
    auto handle = converter.getHandle(d_points, n);
    //*/
   
    //nanovdb::NanoGrid<BuildT>* grid = handle.deviceGrid<BuildT>();
    //handle.deviceDownload();
    //nanovdb::NanoGrid<BuildT>* grid_h = handle.grid<BuildT>();
    //auto* tree = grid_h->treePtr();
    /////const uint32_t maxPointsPerVoxel = converter.maxPointsPerVoxel();
    //////const uint32_t maxPointsPerLeaf = converter.maxPointsPerLeaf();
    ////// save NanoVBD grid

  
    //printf("############### VDB GRID INFORMATION ################\n");
    //printf("Grid Size: %d\n", grid_h->gridSize());
    //printf("Grid Class: %s\n", nanovdb::toStr(handle.gridMetaData()->gridClass()));
    //printf("Grid Type: %s\n", nanovdb::toStr(handle.gridType(0)));
    //printf("Point Count: %d\n", (int)grid_h->pointCount());
    //printf("Upper Internal Nodes: %d\n", grid_h->tree().nodeCount(2));
    //printf("Lower Internal Nodes: %d\n", grid_h->tree().nodeCount(1));
    //printf("Leaf Nodes: %d\n", grid_h->tree().nodeCount(0));
    //printf("Active Voxels: %d\n", grid_h->activeVoxelCount());
    //[rintf("Size of nanovdb::Point: %d", sizeof(BuildT));
  
    ////printf("maxPointsPerVoxel: %d\n maxPointsPerLeaf: %d\n", maxPointsPerVoxel, maxPointsPerLeaf);

    //float wBBoxDimZ = (float)grid_h->worldBBox().dim()[2] * 2;
    //nanovdb::Vec3<float> wBBoxCenter =
    //    nanovdb::Vec3<float>(grid_h->worldBBox().min() + grid_h->worldBBox().dim() * 0.5f);
    //nanovdb::CoordBBox treeIndexBbox = grid_h->tree().bbox();

    //std::cout << "WorldBbox Center: (" << wBBoxCenter[0] << "," << wBBoxCenter[1] << "," << wBBoxCenter[2] << ")" << std::endl;
    //std::cout << "WorldBBox Dimensions: [" << grid_h->worldBBox().dim()[0] << "," << grid_h->worldBBox().dim()[1] << "," << grid_h->worldBBox().dim()[2] << "]" << std::endl;
    //std::cout << "WolrdBBox Bounds: "
    //          << "[" << grid_h->worldBBox().min()[0] << "," << grid_h->worldBBox().min()[1] << "," << grid_h->worldBBox().min()[2]
    //           << "] -> [" << grid_h->worldBBox().max()[0] << "," << grid_h->worldBBox().max()[1] << "," << grid_h->worldBBox().max()[2]
    //           << "]" << std::endl;

    //std::cout << "Bounds: "
    //          << "[" << treeIndexBbox.min()[0] << "," << treeIndexBbox.min()[1] << "," << treeIndexBbox.min()[2]
    //          << "] -> [" << treeIndexBbox.max()[0] << "," << treeIndexBbox.max()[1] << "," << treeIndexBbox.max()[2]
    //          << "]" << std::endl;

    ////try {
    ////    nanovdb::io::writeGrid("realSlope.nvdb", handle);  // Write the NanoVDB grid to file and throw if writing fails
    ////} catch (const std::exception& e) {
    ////    std::cerr << "An exception occurred: \"" << e.what() << "\"" << std::endl;
    ////}
    //printf("############### END #############\n");




   return handle1;
}


nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> addVDBVolume(std::shared_ptr<openvdb::FloatGrid> openVDBgrid) {
    printf("############### VDB POINT GRID INFORMATION ################\n");
    printf("Voxel Size: %f %f %f\n", openVDBgrid->voxelSize()[0], openVDBgrid->voxelSize()[1], openVDBgrid->voxelSize()[2]);
    printf("Grid Class: %d\n", openVDBgrid->getGridClass());
    printf("Grid Type: %s\n", openVDBgrid->gridType().c_str());
    printf("Upper Internal Nodes: %d\n", openVDBgrid->tree().nodeCount()[2]);
    printf("Lower Internal Nodes: %d\n", openVDBgrid->tree().nodeCount()[1]);
    printf("Leaf Nodes: %d\n", openVDBgrid->tree().nodeCount()[0]);
    printf("Active Voxels: %d\n", openVDBgrid->activeVoxelCount());
    // printf("Min BBox: %f %f %f\n", minBBox[0], minBBox[1], minBBox[2]);
    // printf("Max BBox: %f %f %f\n", maxBBox[0], maxBBox[1], maxBBox[2]);
    // printf("Min BBox WorldSpace: %f %f %f\n", minBBoxWS[0], minBBoxWS[1], minBBoxWS[2]);
    // printf("Max BBox WorldSpace: %f %f %f\n", maxBBoxWS[0], maxBBoxWS[1], maxBBoxWS[2]);
    //printf("Volume Dimensions: %f %f %f\n", volDims[0], volDims[1], volDims[2]);
    printf("############### END #############\n");
    cudaDeviceSynchronize();
    cudaStream_t stream;  // Create a CUDA stream to allow for asynchronous copy of pinned CUDA memory.
    cudaStreamCreate(&stream);
    auto handle = nanovdb::createNanoGrid<openvdb::FloatGrid, float, nanovdb::CudaDeviceBuffer>(*openVDBgrid);



    //printf("Uploading Grid to device\n");
    handle.deviceUpload(stream, false); 

 
    cudaDeviceSynchronize();
    cudaStreamDestroy(stream);

    printf("OpenVDB to NanoVDB conversion done!\n");
    return handle;
}

}  // namespace sensor
}  // namespace chrono
#endif


__global__ void initializeBufferKernel(__half* buffer, int N) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < N) {
        buffer[idx] = __float2half(1e-5);
        //printf("Buffer[%d]: %f\n", idx, __half2float(buffer[idx]));
    }
}

__global__ void initializeBufferKernel(__half* frame, __half* albedo, __half* normal, int N) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < N) {
        frame[idx] = __float2half(1e-5);
        albedo[idx] = __float2half(1e-5);
        normal[idx] = __float2half(1e-5);
        // printf("Buffer[%d]: %f\n", idx, __half2float(buffer[idx]));
    }
}

void initializeBuffer(void* frame, int w, int h) {
    int threadsPerBlock = 512;
    int blocksPerGrid = (w * h * 4 + threadsPerBlock - 1) / threadsPerBlock;
    initializeBufferKernel<<<blocksPerGrid, threadsPerBlock>>>((__half*)frame, w * h * 4);
}

void initializeBuffer(void* frame, void* albedo, void* normal, int w, int h) {
    int threadsPerBlock = 512;
    int blocksPerGrid = (w * h * 4 + threadsPerBlock - 1) / threadsPerBlock;
    initializeBufferKernel<<<blocksPerGrid, threadsPerBlock>>>((__half*)frame, (__half*)albedo, (__half*)normal, w * h * 4);
}