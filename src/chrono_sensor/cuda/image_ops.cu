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
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

#include <cuda.h>
#include "image_ops.cuh"
#include "chrono_sensor/optix/shaders/device_utils.h"
#include <iostream>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/execution_policy.h>

namespace chrono {
namespace sensor {

__global__ void image_gauss_kernel_vert(unsigned char* buf, int w, int h, int c, int f_width, float* dweights) {
    int index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    // only run for each output pixel
    if (index < w * h * c) {
        // float f_std = (float)f / 2.f;
        // int f_width = (int)(2.f * 3.14f * f_std);

        int channel = index % c;
        int col = index / c % w;
        int row = index / c / w;

        float sum = 0;
        for (int i = -f_width; i <= f_width; i++) {
            int index_in = channel + col * c + abs(row + i) * w * c;
            if (row + i >= h)
                index_in = channel + col * c + (2 * h - (row + i + 1)) * w * c;

            // float weight = exp(-i * i / (2 * f_std * f_std)) / sqrtf(2.f * 3.14f * f_std * f_std);
            sum += dweights[i + f_width] * ((float)buf[index_in]);
            // sum += ((float)buf[index_in]);
        }
        sum = fminf(255.f,fmaxf(0.f,sum));
        buf[index] = (unsigned char)(sum);
    }
}

__global__ void image_gauss_kernel_horiz(unsigned char* buf, int w, int h, int c, int f_width, float* dweights) {
    int index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    // only run for each output pixel
    if (index < w * h * c) {
        int channel = index % c;
        int col = index / c % w;
        int row = index / c / w;
        float sum = 0;
        for (int i = -f_width; i <= f_width; i++) {
            int index_in = channel + abs(col + i) * c + row * w * c;
            if (col + i >= w)
                index_in = channel + (2 * w - (col + i + 1)) * c + row * w * c;
            sum += dweights[i + f_width] * ((float)buf[index_in]);
        }
        sum = fminf(255.f,fmaxf(0.f,sum));
        buf[index] = (unsigned char)(sum);
    }
}

// merge pixels by the factor
__global__ void image_alias_kernel(unsigned char* bufIn,
                                   unsigned char* bufOut,
                                   int w_out,
                                   int h_out,
                                   int factor,
                                   int pix_size) {
    int out_index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    int w_in = w_out * factor;
    int h_in = h_out * factor;
    //
    // only run for each output pixel
    if (out_index < w_out * h_out * pix_size) {
        int idc_out = out_index % pix_size;
        int idx_out = (out_index / pix_size) % w_out;
        int idy_out = (out_index / pix_size) / w_out;

        float mean = 0.0;

        for (int i = -1; i < factor + 1; i++) {
            for (int j = -1; j < factor + 1; j++) {
                int idc_in = idc_out;
                int idx_in = idx_out * factor + j;
                int idy_in = idy_out * factor + i;

                // reflect when out of range

                if (idx_in < 0)
                    idx_in = -idx_in - 1;
                else if (idx_in >= w_in)
                    idx_in = 2 * w_in - (idx_in + 1);
                if (idy_in < 0)
                    idy_in = -idy_in - 1;
                else if (idy_in >= h_in)
                    idy_in = 2 * h_in - (idy_in + 1);

                int in_index = idy_in * w_in * pix_size + idx_in * pix_size + idc_in;
                mean += (float)bufIn[in_index];
            }
        }
        // bufOut[out_index] = (unsigned char)(mean / (factor * factor));
        bufOut[out_index] = (unsigned char)(mean / ((factor + 2) * (factor + 2)));
        if (idc_out == 3) {
            bufOut[out_index] = 255;
        }
        // bufOut[out_index] = (unsigned char)(25 * idc_out);
    }
}

// merge pixels by the factor
__global__ void image_alias_float_kernel(float* bufIn, float* bufOut, int w_out, int h_out, int factor, int pix_size) {
    int out_index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    int w_in = w_out * factor;
    //
    // only run for each output pixel
    if (out_index < w_out * h_out * pix_size) {
        int idc_out = out_index % pix_size;
        int idx_out = (out_index / pix_size) % w_out;
        int idy_out = (out_index / pix_size) / w_out;

        float mean = 0.f;

        for (int i = 0; i < factor; i++) {
            for (int j = 0; j < factor; j++) {
                int idc_in = idc_out;
                int idx_in = idx_out * factor + j;
                int idy_in = idy_out * factor + i;

                int in_index = idy_in * w_in * pix_size + idx_in * pix_size + idc_in;
                mean += bufIn[in_index];
            }
        }
        bufOut[out_index] = mean / (factor * factor);
    }
}
// merge pixels by the factor
__global__ void image_half4_to_uchar4_kernel(__half* bufIn, unsigned char* bufOut, int N) {
    int idx = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer
    if (idx < N) {
        bufOut[idx] = (unsigned char)(clamp(__half2float(bufIn[idx]), 0.f, 1.f) * 255.f);
        //printf("bufIn[%d] = %f, bufOut[%d] = %d\n", idx, __half2float(bufIn[idx]), idx, bufOut[idx]);
    }
}

__global__ void image_float4_to_uchar4_kernel(float* bufIn, unsigned char* bufOut, int N) {
    int idx = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer
    if (idx < N) {
        bufOut[idx] = (unsigned char)(clamp(bufIn[idx], 0.f, 1.f) * 255.f);
    }

 }
//__global__ void minmax_kernel_2d(float* input,
//                                 float* min_output,
//                                 float* max_output,
//                                 const int width,
//                                 const int height) {
//    extern __shared__ float sdata[];
//
//     int tid = threadIdx.x;
//    int i = blockIdx.x * blockDim.x * 2 + threadIdx.x;
//
//    float min_val = (i < width * height) ? input[i] : FLT_MAX;
//    float max_val = (i < width * height) ? input[i] : -FLT_MAX;
//
//    if (i + blockDim.x < width * height) {
//        float val = input[i + blockDim.x];
//        min_val = fminf(min_val, val);
//        max_val = fmaxf(max_val, val);
//    }
//
//    sdata[tid * 2] = min_val;
//    sdata[tid * 2 + 1] = max_val;
//    __syncthreads();
//
//    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
//        if (tid < s) {
//            sdata[tid * 2] = min_val = fminf(min_val, sdata[(tid + s) * 2]);
//            sdata[tid * 2 + 1] = max_val = fmaxf(max_val, sdata[(tid + s) * 2 + 1]);
//        }
//        __syncthreads();
//    }
//
//    if (tid == 0) {
//        atomicMin(min_output, min_val);
//        atomicMax(max_output, max_val);
//    }
//}


__global__ void depth_to_uchar4_kernel(float* bufIn, unsigned char* bufOut, float d_min, float d_max, int N) {
    int idx = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer
    if (idx < N) {
        float normalized_depth = clamp((bufIn[idx] - d_min) / (d_max - d_min), 0.f, 1.f);
        unsigned char intensity = (unsigned char)(normalized_depth * 255.f);

        // Gray scale colormap
        bufOut[idx * 4 + 0] = intensity;
        bufOut[idx * 4 + 1] = intensity;
        bufOut[idx * 4 + 2] = intensity;
        bufOut[idx * 4 + 3] = (unsigned char)255;

    }
       
}



void cuda_image_gauss_blur_char(void* buf, int w, int h, int c, int factor, CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (w * h * c + nThreads - 1) / nThreads;

    float f_std = (float)factor / 4.f;
    int f_width = (int)(3.14f * f_std);

    int entries = 2 * f_width + 1;

    float* weights = new float[entries];

    for (int i = 0; i <= 2 * f_width; i++) {
        int offset = i - f_width;
        weights[i] = exp(-offset * offset / (2 * f_std * f_std)) / sqrtf(2.f * 3.14f * f_std * f_std);
    }
    float* dweights;
    cudaMalloc(&dweights, entries * sizeof(float));
    cudaMemcpy(dweights, weights, entries * sizeof(float), cudaMemcpyHostToDevice);

    image_gauss_kernel_vert<<<nBlocks, nThreads, 0, stream>>>((unsigned char*)buf, w, h, c, f_width, dweights);
    image_gauss_kernel_horiz<<<nBlocks, nThreads, 0, stream>>>((unsigned char*)buf, w, h, c, f_width, dweights);
    cudaFree(dweights);
    delete[] weights;
}

void cuda_image_alias(void* bufIn, void* bufOut, int w_out, int h_out, int factor, int pix_size, CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (w_out * h_out * pix_size + nThreads - 1) / nThreads;

    image_alias_kernel<<<nBlocks, nThreads, 0, stream>>>((unsigned char*)bufIn, (unsigned char*)bufOut, w_out, h_out,
                                                         factor, pix_size);
}

void cuda_image_alias_float(void* bufIn,
                            void* bufOut,
                            int w_out,
                            int h_out,
                            int factor,
                            int pix_size,
                            CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (w_out * h_out * pix_size + nThreads - 1) / nThreads;

    image_alias_float_kernel<<<nBlocks, nThreads, 0, stream>>>((float*)bufIn, (float*)bufOut, w_out, h_out, factor,
                                                               pix_size);
}

void cuda_image_half4_to_uchar4(void* bufIn, void* bufOut, int w, int h, CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (w * h * 4 + nThreads - 1) / nThreads;
    image_half4_to_uchar4_kernel<<<nBlocks, nThreads, 0, stream>>>((__half*)bufIn, (unsigned char*)bufOut, w * h * 4);
}

void cuda_image_float4_to_uchar4(void* bufIn, void* bufOut, int w, int h, CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (w * h * 4 + nThreads - 1) / nThreads;
    image_float4_to_uchar4_kernel<<<nBlocks, nThreads, 0, stream>>>((float*)bufIn, (unsigned char*)bufOut, w * h * 4);
}

void cuda_depth_to_uchar4(void* bufIn, void* bufOut, int w, int h, CUstream& stream) {
    // Set up kernel launch configuration
    // int blockSize = 256;
    // int gridSize = (w * h + blockSize * 2 - 1) / (blockSize * 2);

    /*float *d_min, *d_max;
    cudaMalloc(&d_min, sizeof(float));
    cudaMalloc(&d_max, sizeof(float));

    cudaMemcpy(d_min, &MIN, sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_max, &MAX, sizeof(float), cudaMemcpyHostToDevice);*/

    thrust::device_vector<float> bufIn_thrust((float*)bufIn, (float*)bufIn + w * h);
    thrust::device_ptr<float> buffIn_ptr = thrust::device_pointer_cast((float*)bufIn);
    // thrust::pair<float*, float*> result = thrust::minmax_element(thrust::device, (float*)bufIn, (float*)bufIn + w *
    // h);

    thrust::pair<thrust::device_vector<float>::iterator, thrust::device_vector<float>::iterator> result =
        thrust::minmax_element(bufIn_thrust.begin(), bufIn_thrust.end());

    // Launch the kernel
    // minmax_kernel_2d<<<gridSize, blockSize, blockSize * 2 * sizeof(float)>>>((float*)bufIn, d_min, d_max, w, h);

    // cudaDeviceSynchronize();

    const int nThreads = 512;
    int nBlocks = (w * h + nThreads - 1) / nThreads;

    depth_to_uchar4_kernel<<<nBlocks, nThreads, 0, stream>>>((float*)bufIn, (unsigned char*)bufOut, *(result.first),
                                                             *(result.second), w * h);
}

//--------------------------------------------//
// Functions for cuda_camera_exposure_correct //
//--------------------------------------------//
// kernel function to modify image exposure
// Step 1: convert image from pixel domain to exposure domain, and calculate mean values of R, G, and B
__global__ void exposure_correct_kernel_1(unsigned char* bufPtr, size_t pixel_num, float* dev_expsr_means,
                                          int* dev_pixel_counts) {
    size_t pixel_idx = blockDim.x * blockIdx.x + threadIdx.x;

    // declare shared memory
    // extern __shared__ float shared_memo[];
    
    if (pixel_idx < pixel_num) {
        // get pixel values and convert to float format
        float pix_r = ((float)(bufPtr[pixel_idx * 4])) / 255.0;
        float pix_g = ((float)(bufPtr[pixel_idx * 4 + 1])) / 255.0;
        float pix_b = ((float)(bufPtr[pixel_idx * 4 + 2])) / 255.0;

        // convert pixel domain into exposure domain
        
        if (0.001 < pix_r && pix_r < 0.999) {
            atomicAdd(&(dev_expsr_means[0]), logf(pix_r / (1.0 - pix_r)) / pixel_num);
            // shared_memo[threadIdx.x * 3] = logf(pix_r / (1.0 - pix_r)) / pixel_num;
            atomicAdd(&(dev_pixel_counts[0]), 1);
        }
        if (0.001 < pix_g && pix_g < 0.999) {
            atomicAdd(&(dev_expsr_means[1]), logf(pix_g / (1.0 - pix_g)) / pixel_num);
            // shared_memo[threadIdx.x * 3 + 1] = logf(pix_g / (1.0 - pix_g)) / pixel_num;
            atomicAdd(&(dev_pixel_counts[1]), 1);
        }
        if (0.001 < pix_b && pix_b < 0.999) {
            atomicAdd(&(dev_expsr_means[2]), logf(pix_b / (1.0 - pix_b)) / pixel_num);
            // shared_memo[threadIdx.x * 3 + 2] = logf(pix_b / (1.0 - pix_b)) / pixel_num;
            atomicAdd(&(dev_pixel_counts[2]), 1);
        }
        
        /* a more efficient way but having bug
        // wait for all threads finishing putting values in shared memory
        __syncthreads();

        // use the 0th thread in the block to accumulate the results of the other threads in the same block
        if (threadIdx.x == 0) {
            for (size_t thread_idx = 1; thread_idx < blockDim.x && thread_idx < pixel_num; ++thread_idx) {
                shared_memo[0] += shared_memo[thread_idx * 3]; // R exposure partial sum
                shared_memo[1] += shared_memo[thread_idx * 3 + 1]; // G exposure partial sum
                shared_memo[2] += shared_memo[thread_idx * 3 + 2]; // B exposure partial sum
            }
            __syncthreads();
            // printf("%d, %f, %f, %f\n", pixel_idx, shared_memo[0], shared_memo[1], shared_memo[2]);
            atomicAdd(&(dev_expsr_means[0]), shared_memo[0]);
            atomicAdd(&(dev_expsr_means[1]), shared_memo[1]);
            atomicAdd(&(dev_expsr_means[2]), shared_memo[2]);
            // printf("%d, %f, %f, %f\n", pixel_idx, dev_expsr_means[0], dev_expsr_means[1], dev_expsr_means[2]);
        }
        */
        
    }
}

// kernel function to modify image exposure
// Step 2: do linear mapping and then convert back into pixel domain
__global__ void exposure_correct_kernel_2(unsigned char* bufPtr, size_t pixel_num, float a, float a1, float b, float b1,
                                          float* dev_expsr_means, int* dev_pixel_counts) {
    size_t pixel_idx = blockDim.x * blockIdx.x + threadIdx.x;
    if (pixel_idx < pixel_num) {
        // get pixel values and convert to float format
        float pix_r = ((float)(bufPtr[pixel_idx * 4])) / 255.0;
        float pix_g = ((float)(bufPtr[pixel_idx * 4 + 1])) / 255.0;
        float pix_b = ((float)(bufPtr[pixel_idx * 4 + 2])) / 255.0;

        // convert into exposure domain
        float exp_r = logf(pix_r / (1.0 - pix_r));
        float exp_g = logf(pix_g / (1.0 - pix_g));
        float exp_b = logf(pix_b / (1.0 - pix_b));

        // do linear mapping
        float expsr_r_mean = dev_expsr_means[0] * pixel_num / dev_pixel_counts[0];
        float expsr_g_mean = dev_expsr_means[1] * pixel_num / dev_pixel_counts[1];
        float expsr_b_mean = dev_expsr_means[2] * pixel_num / dev_pixel_counts[2];
        exp_r = (a + a1 * expsr_r_mean) * exp_r + (b + b1 * expsr_r_mean);
        exp_g = (a + a1 * expsr_g_mean) * exp_g + (b + b1 * expsr_g_mean);
        exp_b = (a + a1 * expsr_b_mean) * exp_b + (b + b1 * expsr_b_mean);

        // convert back into pixel domain and prevent overflow
        pix_r = clamp(1.0 / (1.0 + exp(- exp_r)), 0.f, 1.f);
        pix_g = clamp(1.0 / (1.0 + exp(- exp_g)), 0.f, 1.f);
        pix_b = clamp(1.0 / (1.0 + exp(- exp_b)), 0.f, 1.f);

        // convert back to char and save in image
        bufPtr[pixel_idx * 4] = (unsigned char)(pix_r * 255.999);
        bufPtr[pixel_idx * 4 + 1] = (unsigned char)(pix_g * 255.999);
        bufPtr[pixel_idx * 4 + 2] = (unsigned char)(pix_b * 255.999);
    }
}

// host function to modify image exposure
__host__ void cuda_camera_exposure_correct(unsigned char* bufPtr, size_t width, size_t height, float a, float a1, float b,
                                           float b1, CUstream& stream) {
    const int threads_per_block = 512;
    const size_t channel_num = 3;
    
    size_t pixel_num = width * height;
    const int blocks_per_grid = (pixel_num + threads_per_block - 1) / threads_per_block;
    // const int shared_memo_size = (3 * threads_per_block) * sizeof(float); // 3 * 512 * 4 = 6K [Bytes]
    
    float *dev_expsr_means;
    cudaMalloc((void**)&dev_expsr_means, sizeof(float) * channel_num);
    cudaMemset(dev_expsr_means, 0, sizeof(float) * channel_num);

    int *dev_pixel_counts;
    cudaMalloc((void**)&dev_pixel_counts, sizeof(int) * channel_num);
    cudaMemset(dev_pixel_counts, 0, sizeof(int) * channel_num);

    // exposure_correct_kernel_1<<<blocks_per_grid, threads_per_block, shared_memo_size, stream>>>(bufPtr, pixel_num, dev_expsr_means, dev_pixel_counts);
    exposure_correct_kernel_1<<<blocks_per_grid, threads_per_block, 0, stream>>>(bufPtr, pixel_num, dev_expsr_means, dev_pixel_counts);
    exposure_correct_kernel_2<<<blocks_per_grid, threads_per_block, 0, stream>>>(bufPtr, pixel_num, a, a1, b, b1, dev_expsr_means, dev_pixel_counts);
    
    cudaFree(dev_expsr_means);
    cudaFree(dev_pixel_counts);
}


////---- Functions for cuda_normal_to_uchar4 ----////
// kernel function
__global__ void normal_to_uchar4_kernel(float* bufIn, unsigned char* bufOut, int pixel_num) {
    int pixel_idx = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer
    if (pixel_idx < pixel_num) {
        float nrmlz_normal_element = 0.f;
        for (int element_idx = 0; element_idx < 3; ++element_idx) {
            nrmlz_normal_element = clamp((bufIn[pixel_idx * 3 + element_idx] + 1.0) / 2.0, 0.f, 1.f);
            bufOut[pixel_idx * 4 + element_idx] = (unsigned char)(nrmlz_normal_element * 255.999);;
        }
        bufOut[pixel_idx * 4 + 3] = (unsigned char)255;    
    }
}

// host function
__host__ void cuda_normal_to_uchar4(void* bufIn, void* bufOut, int width, int height, CUstream& stream) {
    int pixel_num = width * height;
    // Set up kernel launch configuration
    const int threads_per_block = 512;
    const int blocks_per_grid = (pixel_num + threads_per_block - 1) / threads_per_block;
    
    // Launch the kernel
    normal_to_uchar4_kernel<<<blocks_per_grid, threads_per_block, 0, stream>>>((float*)bufIn, (unsigned char*)bufOut, pixel_num);
}


////---- Functions for cuda_image_RGBDhalf4_to_half4 ----////
// kernel function
__global__ void RGBDhalf4_to_Half4_kernel(__half* bufIn, __half* bufOut, int pixel_num) {
    int pixel_idx = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer
    if (pixel_idx < pixel_num) {
        // iterate over R, G, and B channels
        for (int element_idx = 0; element_idx < 3; ++element_idx) {
            bufOut[pixel_idx * 4 + element_idx] = bufIn[pixel_idx * 4 + element_idx];
        }
        // set A channel to 1.0f
        bufOut[pixel_idx * 4 + 3] = (__half)1.0;
    }
}

// host function
__host__ void cuda_image_RGBDhalf4_to_Half4(void* bufIn, void* bufOut, int width, int height, CUstream& stream) {
    int pixel_num = width * height;
    // Set up kernel launch configuration
    const int threads_per_block = 512;
    const int blocks_per_grid = (pixel_num + threads_per_block - 1) / threads_per_block;
    
    // Launch the kernel
    RGBDhalf4_to_Half4_kernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
        (__half*)bufIn, (__half*)bufOut, pixel_num
    );
}


////---- Functions for cuda_RGBDhalf4_to_uchar ----////
// kernel function
__global__ void RGBDhalf4_to_uchar_kernel(__half* bufIn, unsigned char* bufOut, __half d_min, __half d_max, int N) {
    int idx = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer
    if (idx < N) {
        // only output D-channel
        bufOut[idx] = (unsigned char)(clamp((bufIn[4 * idx + 3] - d_min) / (d_max - d_min), 0., 1.) * 255.);
        // bufOut[idx] = (unsigned char)128;
    }
}

// host function
void cuda_RGBDhalf4_to_uchar(void* bufIn, void* bufOut, int w, int h, CUstream& stream) {

    thrust::device_vector<__half> buf_in_thrust((__half*)bufIn, (__half*)bufIn + 4 * w * h);
    /*
    // create a strided range to access only the D channel
    auto start = thrust::make_transform_iterator(buf_in_thrust.begin() + 3, thrust::placeholders::_1 + 4);
    auto end = thrust::make_transform_iterator(buf_in_thrust.end(), thrust::placeholders::_1 + 4);

    // return the maximum value in the D channel
    thrust::pair<thrust::device_vector<__half>::iterator, thrust::device_vector<__half>::iterator> result = 
        thrust::minmax_element(
            start, end, thrust::stride_iterator<thrust::device_vector<__half>::iterator>(buf_in_thrust.begin() + 3, 4)
        );
    */

    thrust::pair<thrust::device_vector<__half>::iterator, thrust::device_vector<__half>::iterator> result =
        thrust::minmax_element(buf_in_thrust.begin(), buf_in_thrust.end());

    // Set up kernel launch configuration
    const int nThreads = 512;
    int nBlocks = (w * h + nThreads - 1) / nThreads;
    RGBDhalf4_to_uchar_kernel<<<nBlocks, nThreads, 0, stream>>>(
        (__half*)bufIn, (unsigned char*)bufOut, *(result.first), *(result.second), w * h
    );

}


////---- Functions for cuda_image_half4_to_uint16_t4 ----////
// kernel function
__global__ void cuda_image_half4_to_uint16_t4_kernel(__half* bufIn, uint16_t* bufOut, int N) {
    int idx = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer
    if (idx < N) {
        bufOut[idx] = (uint16_t)(clamp(__half2float(bufIn[idx]), 0.f, 1.f) * 65534.999f);
        // if (idx % 4 == 0)
        //     printf("%u\n", bufOut[idx]);
    }
}

// host function
void cuda_image_half4_to_uint16_t4(void* bufIn, void* bufOut, int w, int h, CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (w * h * 4 + nThreads - 1) / nThreads;
    cuda_image_half4_to_uint16_t4_kernel<<<nBlocks, nThreads, 0, stream>>>(
        (__half*)bufIn, (uint16_t*)bufOut, w * h * 4
    );
}


////---- Functions for cuda_image_alias_rgba16 ----////
// kernel function
// merge pixels by the factor
__global__ void image_alias_rgba16_kernel(uint16_t* bufIn, uint16_t* bufOut, int w_out, int h_out, int factor,
                                           int channel_num) {
    int out_idx = (blockDim.x * blockIdx.x + threadIdx.x);  // index of output buffer entry

    int w_in = w_out * factor;
    //
    // // only run for each output entry
    if (out_idx < w_out * h_out * channel_num) {
        int channel_out = out_idx % channel_num;
        int x_out = (out_idx / channel_num) % w_out;
        int y_out = (out_idx / channel_num) / w_out;

        unsigned int mean = 0;

        for (int i = 0; i < factor; i++) {
            for (int j = 0; j < factor; j++) {
                int channel_in = channel_out;
                int x_in = x_out * factor + j;
                int y_in = y_out * factor + i;

                int in_idx = y_in * w_in * channel_num + x_in * channel_num + channel_in;
                mean += (unsigned int)(bufIn[in_idx]);
            }
        }
        bufOut[out_idx] = (uint16_t)(mean / (factor * factor));

        // if (out_idx % 4 == 0)
        //     printf("%u\n", bufOut[out_idx]);
    }
}

// host function
void cuda_image_alias_rgba16(void* bufIn, void* bufOut, int w_out, int h_out, int factor, int channel_num,
                             CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (w_out * h_out * channel_num + nThreads - 1) / nThreads;

    image_alias_rgba16_kernel<<<nBlocks, nThreads, 0, stream>>>((uint16_t*)bufIn, (uint16_t*)bufOut, w_out, h_out,
                                                         factor, channel_num);
}

}  // namespace sensor
}  // namespace chrono
