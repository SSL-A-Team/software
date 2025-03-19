// Copyright 2025 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "ateam_spatial/min_max_loc_kernel.hpp"

namespace ateam_spatial
{

/*
 * Implementation based on https://www.apriorit.com/dev-blog/614-cpp-cuda-accelerate-algorithm-cpu-gpu#thread
 */

__device__ void atomicMax(float* const address, const float value)
{
    if (*address >= value)
    {
        return;
    }
  
    int* const addressAsI = (int*)address;
    int old = *addressAsI, assumed;
  
    do
    {
        assumed = old;
        if (__int_as_float(assumed) >= value)
        {
            break;
        }
  
        old = atomicCAS(addressAsI, assumed, __float_as_int(value));
    } while (assumed != old);
}

__device__ void atomicMin(float* const address, const float value)
{
    if (*address <= value)
    {
        return;
    }
  
    int* const addressAsI = (int*)address;
    int old = *addressAsI, assumed;
  
    do
    {
        assumed = old;
        if (__int_as_float(assumed) <= value)
        {
            break;
        }
  
        old = atomicCAS(addressAsI, assumed, __float_as_int(value));
    } while (assumed != old);
}

__global__ void min_max_loc_kernel(const float * buffer, const std::size_t buffer_size, float * block_min_out, float * block_max_out, std::size_t * block_min_loc_out, std::size_t * block_max_loc_out)
{
  __shared__ float shared_max;
  __shared__ float shared_min;
  __shared__ std::size_t shared_max_idx;
  __shared__ std::size_t shared_min_idx;

  if (0 == threadIdx.x)
  {
    shared_max = buffer[0];
    shared_min = buffer[0];
    shared_max_idx = 0;
    shared_min_idx = 0;
  }

  __syncthreads();

  const std::size_t start_idx = threadIdx.x + blockIdx.x * blockDim.x;

  float local_max = buffer[start_idx];
  float local_min = buffer[start_idx];
  std::size_t local_max_idx = start_idx;
  std::size_t local_min_idx = start_idx;

  for (int i = start_idx; i < buffer_size; i += blockDim.x)
  {
    float val = buffer[i];

    if (local_max < val)
    {
      local_max = val;
      local_max_idx = i;
    }
    if(local_min > val)
    {
      local_min = val;
      local_min_idx = i;
    }
  }

  atomicMax(&shared_max, local_max);
  atomicMin(&shared_min, local_min);

  __syncthreads();

  if (shared_max == local_max)
  {
    shared_max_idx = local_max_idx;
  }
  if (shared_min == local_min)
  {
    shared_min_idx = local_min_idx;
  }

  __syncthreads();

  if (0 == threadIdx.x)
  {
      block_max_out[blockIdx.x] = shared_max;
      block_max_loc_out[blockIdx.x] = shared_max_idx;
      block_min_out[blockIdx.x] = shared_min;
      block_min_loc_out[blockIdx.x] = shared_min_idx;
  }
}

}  // namespace ateam_spatial
