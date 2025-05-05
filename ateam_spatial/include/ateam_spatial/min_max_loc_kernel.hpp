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

#ifndef ATEAM_SPATIAL__MIN_MAX_LOC_KERNEL_HPP_
#define ATEAM_SPATIAL__MIN_MAX_LOC_KERNEL_HPP_

namespace ateam_spatial
{

__device__ void atomicMax(uint32_t* const address, const uint32_t index, const float * buffer)
{
  if(buffer[*address] >= buffer[index]) {
    return;
  }

  uint32_t old = *address;
  uint32_t assumed = old;

  do
  {
    assumed = old;
    if (buffer[assumed] >= buffer[index])
    {
      break;
    }

    old = atomicCAS(address, assumed, index);
  } while (assumed != old);
}


__device__ void atomicMin(uint32_t* const address, const uint32_t index, const float * buffer)
{
  if(buffer[*address] <= buffer[index]) {
    return;
  }

  uint32_t old = *address;
  uint32_t assumed = old;

  do
  {
    assumed = old;
    if (buffer[assumed] <= buffer[index])
    {
      break;
    }

    old = atomicCAS(address, assumed, index);
  } while (assumed != old);
}

template<unsigned int blockSize>
__global__ void max_loc_kernel(const float * buffer, const uint32_t buffer_size, uint32_t * loc_out)
{
  const auto idx = blockDim.x * blockIdx.x + threadIdx.x;
  __shared__ uint32_t shared_index_data[blockSize];
  if (idx < buffer_size){

    /*copy to shared memory*/
    shared_index_data[threadIdx.x] = idx;
    __syncthreads();

    for(int stride=1; stride < blockDim.x; stride *= 2) {
      if (threadIdx.x % (2*stride) == 0) {
        const auto lhs_block_index = threadIdx.x;
        const auto rhs_block_index = threadIdx.x + stride;
        const auto lhs_index = shared_index_data[lhs_block_index];
        const auto rhs_index = shared_index_data[rhs_block_index];
        const float lhs_value = buffer[lhs_index];
        const float rhs_value = buffer[rhs_index];
        if(lhs_value > rhs_value) {
          shared_index_data[lhs_block_index] = lhs_index;
        } else {
          shared_index_data[lhs_block_index] = rhs_index;
        }
      }
      __syncthreads();
    }
  }
  if (threadIdx.x == 0) {
    atomicMax(loc_out, shared_index_data[0], buffer);
  }
}

template<unsigned int blockSize>
__global__ void min_loc_kernel(const float * buffer, const uint32_t buffer_size, uint32_t * loc_out)
{
  const auto idx = blockDim.x * blockIdx.x + threadIdx.x;
  __shared__ uint32_t shared_index_data[blockSize];
  if (idx < buffer_size){

    /*copy to shared memory*/
    shared_index_data[threadIdx.x] = idx;
    __syncthreads();

    for(int stride=1; stride < blockDim.x; stride *= 2) {
      if (threadIdx.x % (2*stride) == 0) {
        const auto lhs_block_index = threadIdx.x;
        const auto rhs_block_index = threadIdx.x + stride;
        const auto lhs_index = shared_index_data[lhs_block_index];
        const auto rhs_index = shared_index_data[rhs_block_index];
        const float lhs_value = buffer[lhs_index];
        const float rhs_value = buffer[rhs_index];
        if(lhs_value < rhs_value) {
          shared_index_data[lhs_block_index] = lhs_index;
        } else {
          shared_index_data[lhs_block_index] = rhs_index;
        }
      }
      __syncthreads();
    }
  }
  if (threadIdx.x == 0) {
    atomicMin(loc_out, shared_index_data[0], buffer);
  }
}

}  // namespace ateam_spatial

#endif  // ATEAM_SPATIAL__MIN_MAX_LOC_KERNEL_HPP_
