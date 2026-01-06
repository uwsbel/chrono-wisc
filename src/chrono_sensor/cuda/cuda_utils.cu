
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
#include <curand_kernel.h>

// #include <Eigen/Dense>

#ifdef __CUDACC_VER__
    #undef __CUDACC_VER__
    #define __CUDACC_VER__ (__CUDACC_VER_MAJOR__ * 10000 + __CUDACC_VER_MINOR__ * 100 + __CUDACC_VER_BUILD__)
#endif

#define EIGEN_USE_GPU

#ifdef USE_SENSOR_NVDB




#include <nanovdb/NanoVDB.h>
#include <openvdb/openvdb.h>

#include <nanovdb/util/GridHandle.h>
#include <nanovdb/util/cuda/CudaPointsToGrid.cuh>
#include <nanovdb/util/cuda/CudaDeviceBuffer.h>
#include <nanovdb/util/IO.h>

#include <thrust/scan.h>
#include <thrust/device_ptr.h>



namespace chrono {
namespace sensor {


#define GRAVITY 1.62f
#define CUDA_PI_F 3.14159265358979323846f

#define CUDA_SAFE_CALL(call)                                                                                 \
    {                                                                                                        \
        cudaError_t err = call;                                                                              \
        if (err != cudaSuccess) {                                                                            \
            printf("CUDA Error: %s (code %d) at %s:%d\n", cudaGetErrorString(err), err, __FILE__, __LINE__); \
            return;                                                                                          \
        }                                                                                                    \
    }

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

void computePCA(const std::vector<nanovdb::Vec3f>& neighbors, nanovdb::Vec3f& normal) {
    // Compute the centroid
    using Vec3T = nanovdb::Vec3f;
    Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);
    for (const auto& point : neighbors) {
        centroid += Eigen::Vector3f(point[0], point[1], point[2]);
    }
    centroid /= float(neighbors.size());

    // Compute the covariance matrix
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& point : neighbors) {
        Eigen::Vector3f diff = Eigen::Vector3f(point[0], point[1], point[2]) - centroid;
        covariance += diff * diff.transpose();
    }
    covariance /= float(neighbors.size());

    // Perform eigen decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    Eigen::Vector3f n = solver.eigenvectors().col(0);  // Smallest eigenvalue
    n.normalize();

    normal = nanovdb::Vec3f(n.x(), n.y(), n.z());
}

__global__ void computeNormalsKernel(const nanovdb::Coord* d_coords,
                                     nanovdb::Vec3f* d_normals,
                                     int numVoxels,
                                     int windowSize,
                                     nanovdb::NanoGrid<nanovdb::Point>* grid) {
    int voxelIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (voxelIdx >= numVoxels || windowSize > 10)
        return;

    nanovdb::Coord ijk = d_coords[voxelIdx];
    nanovdb::DefaultReadAccessor<nanovdb::Point> accessor = grid->tree().getAccessor();

    // Gather neighboring points
    Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);
    int neighborCount = 0;
    nanovdb::Vec3f neighbors[9261]; // preallocate for a window size of 10
    
    for (int x = -windowSize; x <= windowSize; ++x) {
        for (int y = -windowSize; y <= windowSize; ++y) {
            for (int z = -windowSize; z <= windowSize; ++z) {
                nanovdb::Coord neighborCoord = ijk.offsetBy(x, y, z);
                if (accessor.isActive(neighborCoord)) {
                    nanovdb::Vec3f neighborPos = grid->indexToWorld(neighborCoord.asVec3s());
                    centroid += Eigen::Vector3f(neighborPos[0], neighborPos[1], neighborPos[2]);
                    neighbors[neighborCount++] = neighborPos;
                }
            }
        }
    }

    if (neighborCount < 3) {
        //normal_grid->setValue(ijk,nanovdb::Vec3f(0, 0, 1));  // Default normal for insufficient neighbors
        d_normals[voxelIdx] = nanovdb::Vec3f(0,0,1);
        return;
    }

    centroid /= neighborCount;

    // Compute covariance matrix
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (int i = 0; i < neighborCount; i++) {
        nanovdb::Vec3f point = neighbors[i];
        Eigen::Vector3f diff = Eigen::Vector3f(point[0], point[1], point[2]) - centroid;
        covariance += diff * diff.transpose();
    }
    covariance /= neighborCount;


    // Perform eigen decomposition to get the normal
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    Eigen::Vector3f normal = solver.eigenvectors().col(0);  // Smallest eigenvalue
    normal.normalize();

    //normal_grid->setValue(ijk, nanovdb::Vec3f(normal.x(), normal.y(), normal.z()));
    d_normals[voxelIdx] = nanovdb::Vec3f(normal.x(), normal.y(), normal.z());
    //if (fmaxf(fmaxf(normal.x(), normal.y()), normal.y()) <= 0)
    //    printf("normal: (%f,%f,%f)\n", normal.x(), normal.y(), normal.z());
}

__global__ void smoothNormalsKernel(const nanovdb::Coord* d_coords,
                                    nanovdb::Vec3f* d_normals,
                                    int numVoxels,
                                    int windowSize,
                                    nanovdb::NanoGrid<nanovdb::Vec3f>* normalGrid) {
    int voxelIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (voxelIdx >= numVoxels)
        return;

    nanovdb::Coord ijk = d_coords[voxelIdx];
    nanovdb::DefaultReadAccessor<nanovdb::Vec3f> accessor = normalGrid->tree().getAccessor();

    Eigen::Vector3f smoothedNormal(0.0f, 0.0f, 0.0f);
    float weightSum = 0.0f;

    for (int x = -windowSize; x <= windowSize; ++x) {
        for (int y = -windowSize; y <= windowSize; ++y) {
            for (int z = -windowSize; z <= windowSize; ++z) {
                nanovdb::Coord neighborCoord = ijk.offsetBy(x, y, z);
                if (accessor.isActive(neighborCoord)) {
                    nanovdb::Vec3f neighborNormal = accessor.getValue(neighborCoord);
                    float distance = sqrtf(x * x + y * y + z * z);
                    float weight = expf(-distance * distance / (2.0f * windowSize * windowSize));
                    smoothedNormal +=
                        Eigen::Vector3f(neighborNormal[0], neighborNormal[0], neighborNormal[0]) * weight;
                    weightSum += weight;
                }
            }
        }
    }

    smoothedNormal /= weightSum;
    smoothedNormal.normalize();

    d_normals[voxelIdx] = nanovdb::Vec3f(smoothedNormal.x(), smoothedNormal.y(), smoothedNormal.z());
}

template <typename T>
void printGridInformation(nanovdb::NanoGrid<T>* grid, nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& handle, const char* grid_name) {
    printf("############### %s VDB GRID INFORMATION ################\n", grid_name);
    printf("Grid Size: %d\n", grid->gridSize());
    printf("Grid Class: %s\n", nanovdb::toStr(handle.gridMetaData()->gridClass()));
    printf("Grid Type: %s\n", nanovdb::toStr(handle.gridType(0)));
    // printf("Point Count: %d\n", (int)grid_n->pointCount());
    printf("Upper Internal Nodes: %d\n", grid->tree().nodeCount(2));
    printf("Lower Internal Nodes: %d\n", grid->tree().nodeCount(1));
    printf("Leaf Nodes: %d\n", grid->tree().nodeCount(0));
    printf("Active Voxels: %d\n", grid->activeVoxelCount());

    nanovdb::Vec3<float>  wBBoxCenter = nanovdb::Vec3<float>(grid->worldBBox().min() + grid->worldBBox().dim() * 0.5f);
    nanovdb::CoordBBox treeIndexBbox = grid->tree().bbox();

    std::cout << "WorldBbox Center: (" << wBBoxCenter[0] << "," << wBBoxCenter[1] << "," << wBBoxCenter[2] << ")"
              << std::endl;
    std::cout << "WorldBBox Dimensions: [" << grid->worldBBox().dim()[0] << "," << grid->worldBBox().dim()[1] << ","
              << grid->worldBBox().dim()[2] << "]" << std::endl;
    std::cout << "WolrdBBox Bounds: "
              << "[" << grid->worldBBox().min()[0] << "," << grid->worldBBox().min()[1] << ","
              << grid->worldBBox().min()[2] << "] -> [" << grid->worldBBox().max()[0] << ","
              << grid->worldBBox().max()[1] << "," << grid->worldBBox().max()[2] << "]" << std::endl;

    std::cout << "Bounds: "
              << "[" << treeIndexBbox.min()[0] << "," << treeIndexBbox.min()[1] << "," << treeIndexBbox.min()[2]
              << "] -> [" << treeIndexBbox.max()[0] << "," << treeIndexBbox.max()[1] << "," << treeIndexBbox.max()[2]
              << "]" << std::endl;

    printf("############### END #############\n");
}


__global__ void eject_dust_particles(
    float* d_points,           // Input: flat buffer with terrain particle positions and velocities
    int* dust_flags,               // Input: flags indicating if dust is created
    int* prefix_sum,               // Input: prefix sum for dust particle indices
    DustParticle* dust_particles,  // Output: array of dust particles
    float alpha,                   // Scaling factor for dust velocity
    float min_z_velocity,          // Minimum Z velocity for dust particles
    float max_z_velocity,          // Maximum Z velocity for dust particles
    int n_particles,               // Number of terrain particles
    float current_time,           // Current time
    int curr_n_dust_particles,
    int dpptp
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n_particles && dust_flags[idx] == 1) {
        //printf("idx: %d\n", idx);
        // Get the index for this dust particle in the output array
        int dust_idx = (curr_n_dust_particles + prefix_sum[idx]) * dpptp;
       // printf("idx: %d | offset: %d | prefixsum: %d => %d\n", idx, curr_n_dust_particles * dpptp, prefix_sum[idx], dust_idx);
        // Calculate the index in the flat buffer for this particle's velocity components
        int velocity_idx = idx * 6 + 3;  // Velocity starts at idx+3, idx+4, idx+5
        int pos_idx = 6*idx;
        float3 pos = make_float3(d_points[pos_idx], d_points[pos_idx + 1], d_points[pos_idx + 2]);
        // Extract velocity components from the flat buffer
        float vx = d_points[velocity_idx];
        float vy = d_points[velocity_idx + 1];
        float vz = d_points[velocity_idx + 2];
     
       
        // Compute the magnitude of horizontal velocity in the XY plane
        float v_magnitude = sqrt(vx * vx + vy * vy + vz*vz);
       
        //// Compute the dust particle's initial speed
        float dust_speed = v_magnitude;

        //// Compute the ejection angle in the XY plane
        //float theta_xy = atan2f(vy, vx);

        //// Compute velocity components for the dust particle
        //float v_x = dust_speed * cosf(theta_xy);
        //float v_y = dust_speed * sinf(theta_xy);
        // Set up random state (one per thread)
        curandState rand_state;
        curand_init(1234ULL, idx, 0, &rand_state); // Seed with idx for unique random sequence
        //printf("JJ: idx: %d | offset: %d | prefixsum: %d => %d\n", idx, curr_n_dust_particles * dpptp, prefix_sum[idx],dust_idx);
        // Generate random theta (angle in XY plane) and phi (angle from horizontal)
       
        // Initialize the dust particle
        float theta_max = 10 * CUDA_PI_F / 180;
        float phi_min = 0 * CUDA_PI_F / 180;
        float phi_max = 20 * CUDA_PI_F / 180;
        float voxeljitter_max = 0.01f;
        float voxeljitter_min = 0.f;
        
        for (int i = 0; i < dpptp; i++) {

            int I = i > 0 ? 1 : 0;
            float frac = i * (1/dpptp);
            float theta = CUDA_PI_F - curand_uniform(&rand_state) * 2 * theta_max - theta_max;
            float phi = phi_min + frac*(phi_max - phi_min);//curand_uniform(&rand_state) * (phi_max - phi_min) + phi_min;

            float v_xy_proj = dust_speed * cos(phi);
            float v_x = v_xy_proj * cos(theta);
            float v_y = v_xy_proj * sin(theta);
            float v_z = dust_speed * sin(phi);

            float3 vel = make_float3(v_x, v_y, v_z);
           
            float jitterX = curand_uniform(&rand_state) * (voxeljitter_max - voxeljitter_min) + voxeljitter_min;
            float jitterY  = curand_uniform(&rand_state) * (voxeljitter_max - voxeljitter_min) + voxeljitter_min;
            float jitterZ = curand_uniform(&rand_state) * (voxeljitter_max - voxeljitter_min) + voxeljitter_min;
            //float3 rand_pos = make_float3(pos.x + I*jitterX, pos.y + I*jitterY, pos.z + I*jitterZ);
            // make_float3(d_points[pos_idx] + I * jitterX, d_points[pos_idx + 1] + I*jitterY, d_points[pos_idx + 2] + I*jitterZ)
            //printf("idx: %d | offset: %d | prefixsum: %d => %d | dustidx: %d\n", idx, curr_n_dust_particles * dpptp, prefix_sum[idx], dust_idx, dust_idx + i);
            DustParticle dustp;
            dustp.pos = make_float3(pos.x + I * jitterX, pos.y + I * jitterY, pos.z);
            dustp.vel = make_float3(v_x, v_y, v_z);
            dustp.pos0 = make_float3(pos.x + I * jitterX, pos.y + I * jitterY, pos.z);
            dustp.vel0 = make_float3(v_x, v_y, v_z);
            dustp.t0 = current_time;
            dustp.isActive = true;
            dustp.mass = 0.1f;
            dust_particles[dust_idx + i] = dustp;
        /*   printf("dust idx: %d |Acc: %d | t: %f | vel: %f,%f,%f | pos: %f,%f,%f\n", dust_idx + i,
                   dust_particles[dust_idx + i].isActive, dust_particles[dust_idx + i].t0,
                   dust_particles[dust_idx + i].vel.x, dust_particles[dust_idx + i].vel.y,
                   dust_particles[dust_idx + i].vel.z, dust_particles[dust_idx + i].pos.x,
                   dust_particles[dust_idx + i].pos.y, dust_particles[dust_idx + i].pos.z);*/
           //

        }

        //printf("cn: %d |idx: %d | dust idx: %d | dpos: (%f,%f,%f)\n", curr_n_dust_particles, idx, dust_idx, dust_particles[dust_idx].pos.x, dust_particles[dust_idx].pos.y, dust_particles[dust_idx].pos.z);
      /*  if (vel.z > 0.f) {
            printf("idx: %d | t: %f | vel: %f,%f,%f | pos: %f,%f,%f\n", dust_idx, dustp.t0, vel.x,vel.y,vel.z, pos.x,pos.y,pos.z);
        }*/
    }
}

__global__ void update_dust_particle_positions(DustParticle* d_dust,  // Array of dust particles (input/output)
                                               int n_active_particles,
                                               float curr_time) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < n_active_particles && d_dust[idx].isActive) {
        // Get the current particle
        DustParticle& particle = d_dust[idx];
        //printf("idx: %d | pos: (%f,%f,%f)\n", idx, particle.pos.x, particle.pos.y, particle.pos.z);
        float delta_time = curr_time - particle.t0;
        // Update position using the velocity and time step
        float3 oldpos = particle.pos;
        float3 oldvel = particle.vel;
        float oldtime = particle.t0;
        bool oldActive = particle.isActive;
        particle.pos.x = particle.pos.x + particle.vel.x * delta_time;
        particle.pos.y = particle.pos.y + particle.vel.y * delta_time;
        particle.pos.z = particle.pos.z + particle.vel.z * delta_time - 0.5f * GRAVITY * delta_time * delta_time;  // Z is affected by gravity

        // Update the velocity in the Z direction due to gravity
        particle.vel.z = particle.vel.z -  GRAVITY * delta_time;
        particle.t0 = curr_time;
        // Deactivate the particle if it has fallen below its original Z position
      /*  if (particle.pos.z < particle.pos0.z) {
            particle.isActive = false;
        }*/
     
       /* if (fabs(particle.pos.x) > 5.f || fabs(particle.pos.y) > 5.f || fabs(particle.pos.z) > 10.f) {
            particle.isActive = false;
            
        }*/

        /* printf("nAcc: %d | idx: %d | Prev: t: %f, pos: (%f,%f,%f), vel: (%f,%f,%f), Acc:%d | Curr: t: %f, pos: (%f,%f,%f), vel: (%f,%f,%f), Acc: %d\n",
            n_active_particles,idx,oldtime, oldpos.x, oldpos.y, oldpos.z, oldvel.x, oldvel.y, oldvel.z, oldActive, curr_time, particle.pos.x,
            particle.pos.y, particle.pos.z, particle.vel.x, particle.vel.y, particle.vel.z, particle.isActive);*/

      
    }
}

__global__ void precompute_dust_flags(
    float* terrain_data,       // Input: flat buffer with terrain particle positions and velocities
    int* dust_flags,           // Output: 0 or 1, indicating if dust is created
    float threshold_velocity,  // Velocity threshold for ejecting dust
    float z_thresh,
    int n_particles            // Number of terrain particles
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < n_particles) {
        // Calculate the index in the flat buffer for this particle's velocity components
        int velocity_idx = idx * 6 + 3;  // Velocity starts at idx+3, idx+4, idx+5
        

        // Extract velocity components from the flat buffer
        float vx = terrain_data[velocity_idx];
        float vy = terrain_data[velocity_idx + 1];
        float vz = terrain_data[velocity_idx + 2];

        float pos_z = terrain_data[idx*6 + 2];

        float v_magnitude = sqrt(vx * vx + vy * vy + vz * vz);

        // Set flag to 1 if terrain velocity exceeds threshold, else 0
        dust_flags[idx] = (v_magnitude > threshold_velocity && pos_z >= z_thresh && pos_z <= 2.55f) ? 1 : 0;
    }
}

void compute_prefix_sum(int* d_dust_flags, int* d_prefix_sum, int n_particles) {
    printf("Computing Prefix Sum!\n");
    thrust::device_ptr<int> thrust_flags(d_dust_flags);
    thrust::device_ptr<int> thrust_prefix_sum(d_prefix_sum);
    thrust::exclusive_scan(thrust_flags, thrust_flags + n_particles, thrust_prefix_sum);
}

void resize_dust_array(int new_capacity, int current_dust_capacity, DustParticle* d_dust) {
    // Allocate new memory if necessary
    DustParticle* new_dust_array;
    cudaMalloc(&new_dust_array, new_capacity * sizeof(DustParticle));

    // Copy existing particles to the new array if it exists
    if (current_dust_capacity > 0) {
        cudaMemcpy(new_dust_array, d_dust, current_dust_capacity * sizeof(DustParticle),
                    cudaMemcpyDeviceToDevice);
    }

    // Free old array
    if (d_dust) {
        cudaFree(d_dust);
    }

    // Update with new array
    d_dust = new_dust_array;
    current_dust_capacity = new_capacity;
    
}

__global__ void mark_active_particles(
    DustParticle* particles,  // Input: array of particles
    int* active_flags,        // Output: binary flags indicating active particles (1 if active, 0 otherwise)
    int n_particles           // Number of particles
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n_particles) {
        active_flags[idx] = particles[idx].isActive ? 1 : 0;
    }
}
__global__ void compact_active_particles(
    DustParticle* particles,            // Input: array of particles
    nanovdb::Vec3f* compacted_particles,  // Output: array for compacted active particles
    int* active_flags,                  // Input: binary flags indicating active particles
    int* prefix_sum,                    // Input: prefix sum to determine target positions
    int n_particles                     // Number of particles
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n_particles && active_flags[idx] == 1) {
        int new_idx = prefix_sum[idx];                  // Target index in compacted array
        float3 pos = particles[idx].pos;
        compacted_particles[new_idx] = nanovdb::Vec3f(pos.x,pos.y,pos.z);
        //printf("idx: %d | pos: (%f,%f,%f) | new idx: %d | cpos: (%f,%f,%f)\n", idx, pos.x, pos.y, pos.z, new_idx, compacted_particles[new_idx][0], compacted_particles[new_idx][1], compacted_particles[new_idx][2]);
    }
}
//void extract_active_particles(DustParticle* d_particles,
//                              int n_particles,
//                              nanovdb::Vec3f* d_active_particles,
//                              int& active_particle_count) {
//    // Allocate memory for flags and prefix sum
//    int* d_active_flags;
//    int* d_prefix_sum;
//
//    cudaMalloc(&d_active_flags, n_particles * sizeof(int));
//    cudaMalloc(&d_prefix_sum, n_particles * sizeof(int));
//
//    // Step 1: Mark active particles
//    int block_size = 1024;
//    int grid_size = (n_particles + block_size - 1) / block_size;
//    mark_active_particles<<<grid_size, block_size>>>(d_particles, d_active_flags, n_particles);
//
//    // Step 2: Compute prefix sum
//    compute_prefix_sum(d_active_flags, d_prefix_sum, n_particles);
//
//    // Step 3: Get the number of active particles from the last element of the prefix sum + last flag
//    cudaMemcpy(&active_particle_count, &d_prefix_sum[n_particles - 1], sizeof(int), cudaMemcpyDeviceToHost);
//
//    int h_last_flag;
//    cudaMemcpy(&h_last_flag, &d_active_flags[n_particles - 1], sizeof(int), cudaMemcpyDeviceToHost);
//    active_particle_count += h_last_flag;
//
//    // If no active particles, clean up and return
//    if (active_particle_count == 0) {
//        cudaFree(d_active_flags);
//        cudaFree(d_prefix_sum);
//        return;
//    }
//
//    // Step 4: Allocate memory for compacted active particles
//    cudaMalloc(&d_active_particles, active_particle_count * sizeof(nanovdb::Vec3f));
//
//    // Step 5: Compact the active particles into the new array
//    compact_active_particles<<<grid_size, block_size>>>(d_particles, d_active_particles, d_active_flags, d_prefix_sum, n_particles);
//
//
//    
//    nanovdb::Vec3f* h_active_particles = new nanovdb::Vec3f[active_particle_count];
//    cudaMemcpy(h_active_particles, d_active_particles, active_particle_count * sizeof(nanovdb::Vec3f),
//               cudaMemcpyDeviceToHost);
//    for (int i = 0; i < active_particle_count; i++) {
//        nanovdb::Vec3f p = h_active_particles[i];
//        printf("i: %d | p: (%f,%f,%f)\n", i, p[0], p[1], p[2]);
//    }
//
//    // Free intermediate arrays
//    cudaFree(d_active_flags);
//    cudaFree(d_prefix_sum);
//}




__global__ void copyParticlesKernel(DustParticle* new_dust_array, DustParticle* d_dust, int num_particles) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < num_particles) {
        //printf("idx: %d < %d\n", idx, num_particles);
        DustParticle dust0 = d_dust[idx];
        DustParticle dust1;
        dust1.pos = dust0.pos;
        dust1.vel = dust0.vel;
        dust1.pos0 = dust0.pos0;
        dust1.vel0 = dust0.vel0;
        dust1.t0 = dust0.t0;
        dust1.mass = dust0.mass;
        dust1.isActive = dust0.isActive;
        new_dust_array[idx] = dust1;
    /*    printf("idx: %d | np: (%f,%f,%f) | nAc: %d\n", new_dust_array[idx].pos.x, new_dust_array[idx].pos.y,
               new_dust_array[idx].pos.z, new_dust_array[idx].isActive);*/
    }
}

void createNanoVDBGridHandle(void* h_points_buffer,
                             int n,
                             const double voxel_size,
                             int window_size,
                             nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& handle,
                             nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& normalHandle,
                             nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& densityHandle) {


    
    float* d_points_float_buffer = nullptr;
    cudaMalloc((void**)&d_points_float_buffer, sizeof(float) * 6 * n);
    cudaMemcpy(d_points_float_buffer, h_points_buffer, sizeof(float) * 6 * n, cudaMemcpyHostToDevice);


    //// Loop over the particles and add them to 
    using BuildT = nanovdb::Point;//uint32_t;
    using Vec3T = nanovdb::Vec3f;
    using BufferT = nanovdb::CudaDeviceBuffer;



    nanovdb::Vec3f* d_points = nullptr;
    cudaMalloc((void**)&d_points, sizeof(nanovdb::Vec3f) * n);
 
    cuda_convert_float_buffer_to_NVDBVec3f(d_points_float_buffer, d_points, n);


    printf("Creating NanoVDB Grid with %d points w/ voxel size: %f!\n", n, voxel_size);
    nanovdb::CudaPointsToGrid<BuildT> converter(voxel_size);  // unit map
    //converter.setPointType(nanovdb::PointType::World32);
    converter.setVerbose();
    handle = converter.getHandle(d_points, n);
    printf("Done Creating NanoVDB Grid with %d points!\n", n);
   
    nanovdb::NanoGrid<BuildT>* grid = handle.deviceGrid<BuildT>();
    handle.deviceDownload();
    nanovdb::NanoGrid<BuildT>* grid_h = handle.grid<BuildT>();
    auto* tree = grid_h->treePtr();
    const uint32_t maxPointsPerVoxel = converter.maxPointsPerVoxel();
    const uint32_t maxPointsPerLeaf = converter.maxPointsPerLeaf();
    

  
    printf("############### VDB GRID INFORMATION ################\n");
    printf("Grid Size: %d\n", grid_h->gridSize());
    printf("Grid Class: %s\n", nanovdb::toStr(handle.gridMetaData()->gridClass()));
    printf("Grid Type: %s\n", nanovdb::toStr(handle.gridType(0)));
    printf("Point Count: %d\n", (int)grid_h->pointCount());
    printf("Upper Internal Nodes: %d\n", grid_h->tree().nodeCount(2));
    printf("Lower Internal Nodes: %d\n", grid_h->tree().nodeCount(1));
    printf("Leaf Nodes: %d\n", grid_h->tree().nodeCount(0));
    printf("Active Voxels: %d\n", grid_h->activeVoxelCount());
    printf("Size of nanovdb::Point: %\nd", sizeof(BuildT));
    printf("maxPointsPerVoxel: %d\n maxPointsPerLeaf: %d\n", maxPointsPerVoxel, maxPointsPerLeaf);

    float wBBoxDimZ = (float)grid_h->worldBBox().dim()[2] * 2;
    nanovdb::Vec3<float> wBBoxCenter =
        nanovdb::Vec3<float>(grid_h->worldBBox().min() + grid_h->worldBBox().dim() * 0.5f);
    nanovdb::CoordBBox treeIndexBbox = grid_h->tree().bbox();

    std::cout << "WorldBbox Center: (" << wBBoxCenter[0] << "," << wBBoxCenter[1] << "," << wBBoxCenter[2] << ")" << std::endl;
    std::cout << "WorldBBox Dimensions: [" << grid_h->worldBBox().dim()[0] << "," << grid_h->worldBBox().dim()[1] << "," << grid_h->worldBBox().dim()[2] << "]" << std::endl;
    std::cout << "WolrdBBox Bounds: "
              << "[" << grid_h->worldBBox().min()[0] << "," << grid_h->worldBBox().min()[1] << "," << grid_h->worldBBox().min()[2]
               << "] -> [" << grid_h->worldBBox().max()[0] << "," << grid_h->worldBBox().max()[1] << "," << grid_h->worldBBox().max()[2]
               << "]" << std::endl;

    std::cout << "Bounds: "
              << "[" << treeIndexBbox.min()[0] << "," << treeIndexBbox.min()[1] << "," << treeIndexBbox.min()[2]
              << "] -> [" << treeIndexBbox.max()[0] << "," << treeIndexBbox.max()[1] << "," << treeIndexBbox.max()[2]
              << "]" << std::endl;

    printf("############### END #############\n");
     // create a new grid for storing surface normals
    printf("Creating Normal Grid....\n");
    nanovdb::build::Grid<Vec3T>  normal_grid(Vec3T(0, 0, 0));
    normal_grid.setTransform(voxel_size);
    // Make density field
    nanovdb::build::Grid<float> density_grid(0.f);
    density_grid.setTransform(voxel_size);
    density_grid.setName("density");
    nanovdb::PointAccessor<Vec3T, BuildT> acc(*grid_h);
    int nvoxels = 0;
    int nskipped = 0;
    int numVoxels = grid_h->activeVoxelCount();
    float voxel_density = voxel_size* voxel_size* voxel_size;
    nanovdb::Coord* h_coords = new nanovdb::Coord[numVoxels];
    Vec3T* h_normals = new Vec3T[numVoxels];
    for (auto it2 = grid_h->tree().root().cbeginChild(); it2; ++it2) {
        for (auto it1 = it2->cbeginChild(); it1; ++it1) {
            for (auto it0 = it1->cbeginChild(); it0; ++it0) {
                for (auto vox = it0->cbeginValueOn(); vox; ++vox) {


                    nanovdb::Coord ijk = vox.getCoord();
                    nanovdb::Coord ijkW = grid_h->indexToWorld(ijk);
                    h_coords[nvoxels++] = ijk;
                    float curr_density = density_grid.getValue(ijk);

                    density_grid.setValue(ijk, curr_density+1);
                    //const Vec3T *start, *stop;
                    //const uint64_t count = acc.voxelPoints(ijk, start, stop);
                    //Vec3T wijk = grid_h->indexToWorld(ijk.asVec3s());
                    //nanovdb::Coord pointIdx = nanovdb::RoundDown<nanovdb::Coord>(grid_h->worldToIndex(start[0]));
                    ////printf("count: %d | wijk: (%f,%f,%f) | start: (%f,%f,%f) \n", count, wijk[0], wijk[1], wijk[2], p[0], p[1], p[2]);
                    //std::vector<Vec3T> neighbors;
                    //int windowSize = 3;  // Adjust based on your needs
                    //for (int x = -windowSize; x <= windowSize; ++x) {
                    //    for (int y = -windowSize; y <= windowSize; ++y) {
                    //        for (int z = -windowSize; z <= windowSize; ++z) {
                    //            nanovdb::Coord neighborCoord = ijk.offsetBy(x, y, z);
                    //            if (acc.isActive(neighborCoord)) {
                    //                neighbors.push_back(grid_h->indexToWorld(neighborCoord.asVec3s()));
                    //            }
                    //        }
                    //    }
                    //}

                    //// Compute the normal using PCA
                    //if (neighbors.size() >= 3) {
                    //    Vec3T normal;
                    //    computePCA(neighbors, normal);
                    //    //printf("N: %d | Normal:(%f,%f,%f)\n", neighbors.size(), normal[0], normal[1],normal[2]);
                    //    // Store the normal in the new grid
                    //    normal_grid.setValue(ijk, normal);
                    //} else {
                    //    normal_grid.setValue(ijk, Vec3T(0,0,1));
                    //    nskipped++;
                    //}

                   // nvoxels++;


                }  // loop over active voxels in the leaf node
            } // loop over child nodes of the lower internal nodes
        }  // loop over child nodes of the upper internal nodes
    }// loop over child nodes of the root
    std::cout << "Iterated over " << nvoxels << " Skipped " << nskipped <<  std::endl;

    // Copy coords to GPU
    nanovdb::Coord* d_coords;
    cudaMalloc(&d_coords, numVoxels * sizeof(nanovdb::Coord));
    cudaMemcpy(d_coords, h_coords, numVoxels * sizeof(nanovdb::Coord), cudaMemcpyHostToDevice);

    Vec3T* d_normals;
    cudaMalloc(&d_normals, numVoxels * sizeof(Vec3T));

     // Compute normals
    int threadsPerBlock = 1024;
    int blocksPerGrid = (numVoxels + threadsPerBlock - 1) / threadsPerBlock;

    computeNormalsKernel<<<blocksPerGrid, threadsPerBlock>>>(d_coords, d_normals,numVoxels, window_size, grid);
    cudaMemcpy(h_normals, d_normals, numVoxels * sizeof(nanovdb::Vec3f), cudaMemcpyDeviceToHost);

    for (int i = 0; i < numVoxels; ++i) {
        normal_grid.setValue(h_coords[i], h_normals[i]);
    }


    normalHandle = nanovdb::createNanoGrid<nanovdb::build::Grid<Vec3T>, Vec3T, BufferT>(normal_grid);
    normalHandle.deviceUpload();
    normalHandle.deviceDownload();
    auto grid_n = normalHandle.grid<Vec3T>();
    nanovdb::NanoGrid<nanovdb::Vec3f>* normal_grid_d = normalHandle.deviceGrid<Vec3T>();

    // Smooth normals
    smoothNormalsKernel<<<blocksPerGrid, threadsPerBlock>>>(d_coords, d_normals, numVoxels, window_size, normal_grid_d);
    cudaMemcpy(h_normals, d_normals, numVoxels * sizeof(nanovdb::Vec3f), cudaMemcpyDeviceToHost);
    for (int i = 0; i < numVoxels; ++i) { // Inefficient! Is there a way to edit a NanoGrid in GPU?
        normal_grid.setValue(h_coords[i], h_normals[i]);
    }
    printGridInformation<nanovdb::Vec3f>(grid_n, normalHandle, "NORMAL GRID");


    densityHandle = nanovdb::createNanoGrid<nanovdb::build::Grid<float>, float, BufferT>(density_grid);
    densityHandle.deviceUpload();
    densityHandle.deviceDownload();
    auto grid_density_h = densityHandle.grid<float>();
    //nanovdb::NanoGrid<float>* density_grid_d = density.deviceGrid<Vec3T>();
    printGridInformation<float>(grid_density_h, densityHandle, "DENSITY GRID");

    //Save density grid
    printf("\nSAVING DENSITY GRID....\n");
    try {
        nanovdb::io::writeGrid("crmdensity.nvdb", densityHandle);  // Write the NanoVDB grid to file and throw if writing fails
    } catch (const std::exception& e) {
        std::cerr << "An exception occurred: \"" << e.what() << "\"" << std::endl;
    }
    printf("SAVING DONE!\n");
    
    // free memory
    cudaFree(d_points);
    cudaFree(d_coords);
    cudaFree(d_normals);
    //cudaFree(normal_grid_d);

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


void createDustGrid(nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& dustHandle, void* h_points, int n, float threshold_vel, void** dust_particles_ptr, int& num_dust_particles, float time, int& frame, float z_thresh) {
    
    printf("\n######### RUNNING DUST SIMULATION ############\n");
    float alpha = 1.f;               // Scaling factor for dust velocity
    //float threshold_velocity = 50.0f;  // Velocity threshold for ejecting dust
    float min_z_velocity = 1.5f;
    float max_z_velocity = 1.5f;
    int dpptp = 10;  // # dust particles ejected per terrain particle

    DustParticle* d_dust = (DustParticle*) *dust_particles_ptr;

    float* d_points = nullptr;
    CUDA_SAFE_CALL(cudaMalloc((void**)&d_points, sizeof(float) * 6 * n));
    CUDA_SAFE_CALL(cudaMemcpy(d_points, h_points, sizeof(float) * 6 * n, cudaMemcpyHostToDevice));

    float* h_buffer = (float*)h_points;
    // Find min and max bounds of the terrain
    float min_x = FLT_MAX;
    float max_x = -FLT_MAX;
    float min_y = FLT_MAX;
    float max_y = -FLT_MAX;
    float min_z = FLT_MAX;
    float max_z = -FLT_MAX;
    for (int i = 0; i < n; i++) {
        float x = h_buffer[i * 6];
        float y = h_buffer[i * 6 + 1];
        float z = h_buffer[i * 6 + 2];
        min_x = fminf(min_x, x);
        max_x = fmaxf(max_x, x);
        min_y = fminf(min_y, y);
        max_y = fmaxf(max_y, y);
        min_z = fminf(min_z, z);
        max_z = fmaxf(max_z, z);
    }
    printf("Terrain Bounds: (%f,%f,%f) -> (%f,%f,%f)\n", min_x, min_y, min_z, max_x, max_y, max_z);
    
    // precompute amount of dust particles
    int* d_dust_flags;
    int* d_prefix_sum;

    CUDA_SAFE_CALL(cudaMalloc(&d_dust_flags, n * sizeof(int)));
    CUDA_SAFE_CALL(cudaMalloc(&d_prefix_sum, n * sizeof(int)));

    int threadsPerBlock = 512;
    int blocksPerGrid = (n + threadsPerBlock - 1) / threadsPerBlock;
    printf("Computing num dust particles to eject.......\n");
    precompute_dust_flags<<<blocksPerGrid, threadsPerBlock>>>(d_points, d_dust_flags, threshold_vel, z_thresh, n);
    CUDA_SAFE_CALL(cudaGetLastError());  // Check if kernel launch failed
    CUDA_SAFE_CALL(cudaDeviceSynchronize());

    compute_prefix_sum(d_dust_flags, d_prefix_sum, n);
    CUDA_SAFE_CALL(cudaGetLastError());  // Check if kernel launch failed
    CUDA_SAFE_CALL(cudaDeviceSynchronize());
    
    int h_num_new_dust_particles;
    CUDA_SAFE_CALL(cudaMemcpy(&h_num_new_dust_particles, &d_prefix_sum[n - 1], sizeof(int), cudaMemcpyDeviceToHost));

    int h_last_flag;
    CUDA_SAFE_CALL(cudaMemcpy(&h_last_flag, &d_dust_flags[n - 1], sizeof(int), cudaMemcpyDeviceToHost));

    h_num_new_dust_particles += h_last_flag;
    printf("Ejecting %d dust particles x %d =  %d!\n", h_num_new_dust_particles, dpptp, h_num_new_dust_particles*dpptp);
    if (h_num_new_dust_particles == 0 && num_dust_particles == 0)
        return;
   
    if (num_dust_particles == 0) {
        printf("Fitst Pass! Initializing dust array\n");
        CUDA_SAFE_CALL(cudaMalloc(&d_dust, h_num_new_dust_particles * dpptp * sizeof(DustParticle)));
    } else if(h_num_new_dust_particles > 0){
        printf("Resizing dust array from %d to %d....\n", num_dust_particles*dpptp, (h_num_new_dust_particles + num_dust_particles)*dpptp);
        //resize_dust_array(h_num_new_dust_particles+num_dust_particles, num_dust_particles, d_dust);
        DustParticle* new_dust_array;
        int new_capacity = (h_num_new_dust_particles+num_dust_particles)*dpptp;
        CUDA_SAFE_CALL(cudaMalloc(&new_dust_array, new_capacity * sizeof(DustParticle)));

        // Copy existing particles to the new array if it exists
        if (num_dust_particles > 0) {
            //cudaMemcpy(new_dust_array, d_dust, num_dust_particles * sizeof(DustParticle), cudaMemcpyDeviceToDevice);
            printf("Copying old dust array into new one...\n");
            threadsPerBlock = 512;
            blocksPerGrid = (num_dust_particles*dpptp + threadsPerBlock - 1) / threadsPerBlock;
            copyParticlesKernel<<<blocksPerGrid, threadsPerBlock>>>(new_dust_array, d_dust, num_dust_particles*dpptp);
            CUDA_SAFE_CALL(cudaGetLastError());  // Check if kernel launch failed
            CUDA_SAFE_CALL(cudaDeviceSynchronize());
        }
        //cudaDeviceSynchronize();
        // Free old array
        if (d_dust) {
            CUDA_SAFE_CALL(cudaFree(d_dust));
        }
        // Update with new array
        d_dust = new_dust_array;
    }
    //cudaDeviceSynchronize();
    
    printf("Ejecting dust particles!\n");
    threadsPerBlock = 512;
    blocksPerGrid = (n + threadsPerBlock - 1) / threadsPerBlock;
    eject_dust_particles<<<blocksPerGrid, threadsPerBlock>>>(d_points, d_dust_flags, d_prefix_sum, d_dust, alpha,
                                                             min_z_velocity, max_z_velocity, n, time,
                                                            num_dust_particles, dpptp);
    CUDA_SAFE_CALL(cudaGetLastError());  // Check if kernel launch failed
    CUDA_SAFE_CALL(cudaDeviceSynchronize());
    int old_num_dust_particles = num_dust_particles;
    num_dust_particles += h_num_new_dust_particles;



    printf("Advancing Simulation: Time: %f\n", time);
    blocksPerGrid = (num_dust_particles*dpptp + threadsPerBlock - 1) / threadsPerBlock;
    update_dust_particle_positions<<<blocksPerGrid, threadsPerBlock>>>(d_dust, num_dust_particles*dpptp, time);
    CUDA_SAFE_CALL(cudaGetLastError());  // Check if kernel launch failed
    CUDA_SAFE_CALL(cudaDeviceSynchronize());

    // Update dust particles ptr
    *dust_particles_ptr = (void*)d_dust;

    /*DustParticle* h_dust = new DustParticle[num_dust_particles*dpptp];
    CUDA_SAFE_CALL(cudaMemcpy(h_dust, d_dust, num_dust_particles*dpptp * sizeof(DustParticle), cudaMemcpyDeviceToHost));*/
  /*  for (int i = 0; i < num_dust_particles; i++) {
        float3 pos = h_dust[i].pos;
        printf("idx: %d | p: (%f,%f,%f) | Active: %d\n", i, pos.x,pos.y,pos.z, h_dust[i].isActive);
    }*/
    // TDOD: Check collisions
    // TODO: Remove inactivae particles



    // Extract active particles
    printf("Extracting active particles out of %d particles....\n", num_dust_particles*dpptp);
    nanovdb::Vec3f* d_active_particles = nullptr;  // Compacted array of active particles
    int active_particle_count = 0;

    int* d_acc_active_flags;
    int* d_acc_prefix_sum;

    CUDA_SAFE_CALL(cudaMalloc(&d_acc_active_flags, num_dust_particles*dpptp * sizeof(int)));
    CUDA_SAFE_CALL(cudaMalloc(&d_acc_prefix_sum, num_dust_particles*dpptp * sizeof(int)));

    // Step 1: Mark active particles
    int block_size = 512;
    int grid_size = (num_dust_particles*dpptp + block_size - 1) / block_size;
    mark_active_particles<<<grid_size, block_size>>>(d_dust, d_acc_active_flags, num_dust_particles * dpptp);
    CUDA_SAFE_CALL(cudaGetLastError());  // Check if kernel launch failed
    CUDA_SAFE_CALL(cudaDeviceSynchronize());
    // Step 2: Compute prefix sum
    compute_prefix_sum(d_acc_active_flags, d_acc_prefix_sum, num_dust_particles*dpptp);
    CUDA_SAFE_CALL(cudaGetLastError());  // Check if kernel launch failed
    CUDA_SAFE_CALL(cudaDeviceSynchronize());
    // Step 3: Get the number of active particles from the last element of the prefix sum + last flag
    CUDA_SAFE_CALL(cudaMemcpy(&active_particle_count, &d_acc_prefix_sum[num_dust_particles*dpptp - 1], sizeof(int),
                             cudaMemcpyDeviceToHost));

    int h_acc_last_flag;
    CUDA_SAFE_CALL(cudaMemcpy(&h_acc_last_flag, &d_acc_active_flags[num_dust_particles*dpptp - 1], sizeof(int),
                             cudaMemcpyDeviceToHost));
    active_particle_count += h_acc_last_flag;

    // If no active particles, clean up and return
    if (active_particle_count == 0) {
        cudaFree(d_acc_active_flags);
        cudaFree(d_acc_prefix_sum);
        return;
    }

    // Step 4: Allocate memory for compacted active particles
    CUDA_SAFE_CALL(cudaMalloc(&d_active_particles, active_particle_count * sizeof(nanovdb::Vec3f)));

    // Step 5: Compact the active particles into the new array
    compact_active_particles<<<grid_size, block_size>>>(d_dust, d_active_particles, d_acc_active_flags, d_acc_prefix_sum,
                                                        num_dust_particles*dpptp);
    CUDA_SAFE_CALL(cudaGetLastError());  // Check if kernel launch failed
    CUDA_SAFE_CALL(cudaDeviceSynchronize());

    nanovdb::Vec3f* h_active_particles = new nanovdb::Vec3f[active_particle_count];
    CUDA_SAFE_CALL(cudaMemcpy(h_active_particles, d_active_particles, active_particle_count * sizeof(nanovdb::Vec3f),
               cudaMemcpyDeviceToHost));
    //for (int i = 0; i < active_particle_count; i++) {
    //    nanovdb::Vec3f p = h_active_particles[i];
    //    printf("i: %d | p: (%f,%f,%f)\n", i, p[0], p[1], p[2]);
    //}

    // Free intermediate arrays
    CUDA_SAFE_CALL(cudaFree(d_acc_active_flags));
    CUDA_SAFE_CALL(cudaFree(d_acc_prefix_sum));

    //// Extract active particles
    //printf("Extracting Active Particles....\n");
    //extract_active_particles(d_dust, num_dust_particles, d_active_particles, active_particle_count);
    if (!active_particle_count) {
        printf("No Active Particles!\n");
        return;
    }
    printf("There are %d active dust particles\n", active_particle_count);

    //nanovdb::Vec3f* h_active_particles = new nanovdb::Vec3f[active_particle_count];
    //cudaMemcpy(h_active_particles, d_active_particles, active_particle_count * sizeof(nanovdb::Vec3f),
    //           cudaMemcpyDeviceToHost);
    //for (int i = 0; i < active_particle_count; i++) {
    //    nanovdb::Vec3f p = h_active_particles[i];
    //    printf("i: %d | p: (%f,%f,%f)\n", i, p[0], p[1], p[2]);
    //}

    CUDA_SAFE_CALL(cudaFree(d_dust_flags));
    CUDA_SAFE_CALL(cudaFree(d_prefix_sum));
    CUDA_SAFE_CALL(cudaFree(d_points));


     // Create CudaPointsToGrod
    // Make density field
    using BuildT = nanovdb::Point;  // uint32_t;
    using Vec3T = nanovdb::Vec3f;
    using BufferT = nanovdb::CudaDeviceBuffer;

    float voxel_size = .001f;
    //printf("Creating NanoVDB Grid with %d points w/ voxel size: %f!\n", active_particle_count, voxel_size);
    //nanovdb::CudaPointsToGrid<BuildT> converter(voxel_size);  // unit map
    //// converter.setPointType(nanovdb::PointType::World32);
    //converter.setVerbose();
    //nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> handle = converter.getHandle(d_active_particles, active_particle_count);
    //printf("Done Creating NanoVDB Grid with %d points!\n", n);

    //nanovdb::NanoGrid<BuildT>* grid = handle.deviceGrid<BuildT>();
    //handle.deviceDownload();
    //nanovdb::NanoGrid<BuildT>* grid_h = handle.grid<BuildT>();
    //auto* tree = grid_h->treePtr();
    //const uint32_t maxPointsPerVoxel = converter.maxPointsPerVoxel();
    //const uint32_t maxPointsPerLeaf = converter.maxPointsPerLeaf();

    //printf("############### VDB GRID INFORMATION ################\n");
    //printf("Grid Size: %d\n", grid_h->gridSize());
    //printf("Grid Class: %s\n", nanovdb::toStr(handle.gridMetaData()->gridClass()));
    //printf("Grid Type: %s\n", nanovdb::toStr(handle.gridType(0)));
    //printf("Point Count: %d\n", (int)grid_h->pointCount());
    //printf("Upper Internal Nodes: %d\n", grid_h->tree().nodeCount(2));
    //printf("Lower Internal Nodes: %d\n", grid_h->tree().nodeCount(1));
    //printf("Leaf Nodes: %d\n", grid_h->tree().nodeCount(0));
    //printf("Active Voxels: %d\n", grid_h->activeVoxelCount());
    //printf("Size of nanovdb::Point: %\nd", sizeof(BuildT));
    //printf("maxPointsPerVoxel: %d\n maxPointsPerLeaf: %d\n", maxPointsPerVoxel, maxPointsPerLeaf);

    //float wBBoxDimZ = (float)grid_h->worldBBox().dim()[2] * 2;
    //nanovdb::Vec3<float> wBBoxCenter =
    //    nanovdb::Vec3<float>(grid_h->worldBBox().min() + grid_h->worldBBox().dim() * 0.5f);
    //nanovdb::CoordBBox treeIndexBbox = grid_h->tree().bbox();

    //std::cout << "WorldBbox Center: (" << wBBoxCenter[0] << "," << wBBoxCenter[1] << "," << wBBoxCenter[2] << ")"
    //          << std::endl;
    //std::cout << "WorldBBox Dimensions: [" << grid_h->worldBBox().dim()[0] << "," << grid_h->worldBBox().dim()[1] << ","
    //          << grid_h->worldBBox().dim()[2] << "]" << std::endl;
    //std::cout << "WolrdBBox Bounds: "
    //          << "[" << grid_h->worldBBox().min()[0] << "," << grid_h->worldBBox().min()[1] << ","
    //          << grid_h->worldBBox().min()[2] << "] -> [" << grid_h->worldBBox().max()[0] << ","
    //          << grid_h->worldBBox().max()[1] << "," << grid_h->worldBBox().max()[2] << "]" << std::endl;

    //std::cout << "Bounds: "
    //          << "[" << treeIndexBbox.min()[0] << "," << treeIndexBbox.min()[1] << "," << treeIndexBbox.min()[2]
    //          << "] -> [" << treeIndexBbox.max()[0] << "," << treeIndexBbox.max()[1] << "," << treeIndexBbox.max()[2]
    //          << "]" << std::endl;

    //printf("############### END #############\n");

    nanovdb::build::Grid<float> density_grid(0.f);
    density_grid.setTransform(voxel_size);
    density_grid.setName("density");
    //nanovdb::PointAccessor<Vec3T, BuildT> acc(*grid_h);
    int nvoxels = 0;
    //int nskipped = 0;
    //int numVoxels = grid_h->activeVoxelCount();
    //float voxel_density = voxel_size * voxel_size * voxel_size;
    //nanovdb::Coord* h_coords = new nanovdb::Coord[numVoxels];
    //Vec3T* h_normals = new Vec3T[numVoxels];
    float MIN_DENSITY = 0.f;
    float MAX_DENSITY = FLT_MIN;
    //const Vec3T *start = nullptr, *stop = nullptr;
    //for (auto it2 = grid_h->tree().root().cbeginChild(); it2; ++it2) {
    //    for (auto it1 = it2->cbeginChild(); it1; ++it1) {
    //        for (auto it0 = it1->cbeginChild(); it0; ++it0) {
    //            for (auto vox = it0->cbeginValueOn(); vox; ++vox) {
    //                nanovdb::Coord ijk = vox.getCoord();
    //                nanovdb::Coord ijkW = grid_h->indexToWorld(ijk);
    //                h_coords[nvoxels++] = ijk;

    //                const auto* leaf = acc.get<nanovdb::GetLeaf<BuildT>>(ijk);
    //                const uint64_t count = acc.voxelPoints(ijk, start, stop);
    //                float curr_density = density_grid.getValue(ijk);
    //                for (uint64_t j = 0; j < count; ++j) {
    //                    curr_density++;
    //                }
    //                density_grid.setValue(ijk, curr_density);

    //                if (curr_density >= MAX_DENSITY) {   
    //                    MAX_DENSITY = curr_density;
    //                }
    //                

    //            }  // loop over active voxels in the leaf node
    //        }      // loop over child nodes of the lower internal nodes
    //    }          // loop over child nodes of the upper internal nodes
    //}   
    
    for (int i = 0; i < active_particle_count; i++) {
        nanovdb::Vec3f p = h_active_particles[i];
        nanovdb::Coord ijk(p[0] / voxel_size, p[1] / voxel_size, p[2] / voxel_size);

        //std::cout << "p:" << p[0] << "," << p[1] << "," << p[2] << "|, "<< "Coord: " << ijk[0] << "," << ijk[1] << "," << ijk[2] << std::endl;
        float density = density_grid.getValue(ijk)+1;
        density_grid.setValue(ijk, density);

        if (density >= MAX_DENSITY) {
             MAX_DENSITY = density;
        }
    }

    for (int i = 0; i < active_particle_count; i++) {
        nanovdb::Vec3f p = h_active_particles[i];
        nanovdb::Coord ijk(p[0] / voxel_size, p[1] / voxel_size, p[2] / voxel_size);
        float density = density_grid.getValue(ijk) + 1;
        float norm_density = (density - MIN_DENSITY) / (MAX_DENSITY - MIN_DENSITY);
        density_grid.setValue(ijk, norm_density);
    }


    // loop over child nodes of the root

    std::cout << "Iterated over " << nvoxels << "MAX DENSITY: " << MAX_DENSITY << ", MIN DENSITY: " << MIN_DENSITY <<  std::endl;
    // Normalize density grid
    //for (auto it2 = grid_h->tree().root().cbeginChild(); it2; ++it2) {
    //    for (auto it1 = it2->cbeginChild(); it1; ++it1) {
    //        for (auto it0 = it1->cbeginChild(); it0; ++it0) {
    //            for (auto vox = it0->cbeginValueOn(); vox; ++vox) {
    //                nanovdb::Coord ijk = vox.getCoord();
    //                nanovdb::Coord ijkW = grid_h->indexToWorld(ijk);
    //                //h_coords[nvoxels++] = ijk;
    //                float curr_density = density_grid.getValue(ijk);
    //                float norm_density = (curr_density - MIN_DENSITY) / (MAX_DENSITY - MIN_DENSITY);
    //                density_grid.setValue(ijk, norm_density);

    //            }  // loop over active voxels in the leaf node
    //        }      // loop over child nodes of the lower internal nodes
    //    }          // loop over child nodes of the upper internal nodes
    //}              // loop over child nodes of the root

    dustHandle = nanovdb::createNanoGrid<nanovdb::build::Grid<float>, float, BufferT>(density_grid);
    dustHandle.deviceUpload();
    //densityHandle.deviceDownload();
    auto grid_density_h = dustHandle.grid<float>();
    // nanovdb::NanoGrid<float>* density_grid_d = density.deviceGrid<Vec3T>();
    printGridInformation<float>(grid_density_h, dustHandle, "DENSITY GRID");

    //// Save density grid
    printf("\nSAVING DUST DENSITY GRID....\n");
    try {
        char formatted_name[13];  // 4 for "dust", 3 for the number, 1 for space, and 1 for null terminator

        // Format the name and number into the C-string
        snprintf(formatted_name, sizeof(formatted_name), "dust%03d.nvdb", frame);
        nanovdb::io::writeGrid(formatted_name, dustHandle);  // Write the NanoVDB grid to file and throw if writing fails
        frame += 1;
    } catch (const std::exception& e) {
        std::cerr << "An exception occurred: \"" << e.what() << "\"" << std::endl;
    }
    printf("SAVING DONE!\n");


    
     CUDA_SAFE_CALL(cudaFree(d_active_particles));
    printf("########## END DUST SIMULATION ############");
}   

}  // namespace sensor
}  // namespace chrono
#endif


__global__ void initializeBufferKernel(float* buffer, int N) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < N) {
        buffer[idx] = 0.f;
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
    initializeBufferKernel<<<blocksPerGrid, threadsPerBlock>>>((float*)frame, w * h * 4);
}

void initializeBuffer(void* frame, void* albedo, void* normal, int w, int h) {
    int threadsPerBlock = 512;
    int blocksPerGrid = (w * h * 4 + threadsPerBlock - 1) / threadsPerBlock;
    initializeBufferKernel<<<blocksPerGrid, threadsPerBlock>>>((__half*)frame, (__half*)albedo, (__half*)normal, w * h * 4);
}