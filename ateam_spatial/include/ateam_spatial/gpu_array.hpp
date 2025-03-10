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

#ifndef ATEAM_SPATIAL__GPU_ARRAY_HPP_
#define ATEAM_SPATIAL__GPU_ARRAY_HPP_

#include <cuda_runtime_api.h>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <array>
#include <stdexcept>
#include <string>

namespace ateam_spatial
{

template<typename T, std::size_t S>
class GpuArray {
public:
  GpuArray()
  {
    assert(std::is_trivial_v<T>);
    const auto ret = cudaMalloc(&gpu_memory_, S * sizeof(T));
    if(ret != cudaSuccess) {
      throw std::runtime_error(std::string("cudaMalloc failed: ") + cudaGetErrorString(ret));
    }
  }

  ~GpuArray()
  {
    const auto ret = cudaFree(gpu_memory_);
    if(ret != cudaSuccess) {
      std::cerr << "cudaFree failed: " << cudaGetErrorString(ret) << std::endl;
    }
  }

  T * Get()
  {
    return reinterpret_cast<T *>(gpu_memory_);
  }

  std::size_t Size() const
  {
    return S;
  }

  void CopyToGpu(const typename std::array<T, S> & values)
  {
    const auto ret = cudaMemcpy(gpu_memory_, values.data(), values.size() * sizeof(T),
        cudaMemcpyHostToDevice);
    if(ret != cudaSuccess) {
      throw std::runtime_error(std::string("cudaMemcpy failed: ") + cudaGetErrorString(ret));
    }
  }

  void CopyFromGpu(typename std::array<T, S> & values)
  {
    const auto ret = cudaMemcpy(values.data(), gpu_memory_, values.size() * sizeof(T),
        cudaMemcpyDeviceToHost);
    if(ret != cudaSuccess) {
      throw std::runtime_error(std::string("cudaMemcpy failed: ") + cudaGetErrorString(ret));
    }
  }

private:
  void * gpu_memory_ = nullptr;
};

}  // namespace ateam_spatial

#endif  // ATEAM_SPATIAL__GPU_ARRAY_HPP_
