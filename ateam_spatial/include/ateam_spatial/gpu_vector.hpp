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

#ifndef ATEAM_SPATIAL__GPU_VECTOR_HPP_
#define ATEAM_SPATIAL__GPU_VECTOR_HPP_

#include <cuda_runtime_api.h>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <string>

namespace ateam_spatial
{

template<typename T>
class GpuVector {
public:
  GpuVector()
  {
    assert(std::is_trivial_v<T>);
  }

  GpuVector(const GpuVector<T> &) = delete;

  GpuVector(GpuVector<T> && other)
  {
    gpu_memory_ = other.gpu_memory_;
    size_ = other.size_;
    other.gpu_memory_ = nullptr;
    other.size_ = 0;
  }

  ~GpuVector()
  {
    if(gpu_memory_ == nullptr) {
      return;
    }
    const auto ret = cudaFree(gpu_memory_);
    if(ret != cudaSuccess) {
      std::cerr << "cudaFree failed: " << cudaGetErrorString(ret) << std::endl;
    }
  }

  GpuVector<T>& operator=(GpuVector<T> && other)
  {
    FreeGpuMemory();
    gpu_memory_ = other.gpu_memory_;
    size_ = other.size_;
    other.gpu_memory_ = nullptr;
    other.size_ = 0;
    return *this;
  }

  T * Get()
  {
    return reinterpret_cast<T *>(gpu_memory_);
  }

  std::size_t Size() const
  {
    return size_;
  }

  void CopyToGpu(const typename std::vector<T> & values)
  {
    AllocateGpuMemory(values.size());
    const auto ret = cudaMemcpy(gpu_memory_, values.data(), values.size() * sizeof(T),
        cudaMemcpyHostToDevice);
    if(ret != cudaSuccess) {
      throw std::runtime_error(std::string("cudaMemcpy failed: ") + cudaGetErrorString(ret));
    }
  }

  void CopyFromGpu(typename std::vector<T> & values)
  {
    if(size_ == 0) {
      values.clear();
      return;
    }
    values.resize(size_);
    const auto ret = cudaMemcpy(values.data(), gpu_memory_, size_ * sizeof(T),
        cudaMemcpyDeviceToHost);
    if(ret != cudaSuccess) {
      throw std::runtime_error(std::string("cudaMemcpy failed: ") + cudaGetErrorString(ret));
    }
  }

private:
  void * gpu_memory_ = nullptr;
  std::size_t size_ = 0;

  void AllocateGpuMemory(const std::size_t new_size)
  {
    if(new_size == size_) {
      return;
    }
    FreeGpuMemory();
    if(new_size == 0) {
      return;
    }
    const auto ret = cudaMalloc(&gpu_memory_, new_size * sizeof(T));
    if(ret != cudaSuccess) {
      throw std::runtime_error(std::string("cudaMalloc failed: ") + cudaGetErrorString(ret));
    }
    size_ = new_size;
  }

  void FreeGpuMemory()
  {
    if(gpu_memory_ == nullptr) {
      return;
    }
    const auto ret = cudaFree(gpu_memory_);
    if(ret != cudaSuccess) {
      throw std::runtime_error(std::string("cudaFree failed: ") + cudaGetErrorString(ret));
    }
    gpu_memory_ = nullptr;
    size_ = 0;
  }
};

}  // namespace ateam_spatial

#endif  // ATEAM_SPATIAL__GPU_VECTOR_HPP_
