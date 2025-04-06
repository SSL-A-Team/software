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

#ifndef ATEAM_SPATIAL__GPU_MULTIBUFFER_HPP_
#define ATEAM_SPATIAL__GPU_MULTIBUFFER_HPP_

#include <cuda_runtime_api.h>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <string>

namespace ateam_spatial
{

/**
 * @brief Manages a collection of buffers in GPU memory.
 *
 * Buffers are allocated in one continuous block.
 *
 * @tparam T The value type to store in the buffers
 */
template<typename T>
class GpuMultibuffer {
public:
  GpuMultibuffer(const std::size_t buffer_count, const std::size_t buffer_size)
  : buffer_count_(buffer_count),
    buffer_size_(buffer_size)
  {
    assert(std::is_trivial_v<T>);
    if(buffer_count == 0 || buffer_size == 0) {
      return;
    }
    const auto buffer_byte_size = buffer_size_ * sizeof(T);
    const auto total_byte_size = buffer_count_ * buffer_byte_size;
    {
      const auto ret = cudaMalloc(&gpu_memory_, total_byte_size);
      if(ret != cudaSuccess) {
        throw std::runtime_error(std::string("cudaMalloc failed: ") + cudaGetErrorString(ret));
      }
    }
    {
      const auto ret = cudaMemset(gpu_memory_, 0, total_byte_size);
      if(ret != cudaSuccess) {
        throw std::runtime_error(std::string("cudaMemset failed: ") + cudaGetErrorString(ret));
      }
    }
  }

  GpuMultibuffer()
  : GpuMultibuffer(0, 0) {}

  GpuMultibuffer(const GpuMultibuffer &) = delete;

  GpuMultibuffer(GpuMultibuffer && other)
  {
    gpu_memory_ = other.gpu_memory_;
    buffer_count_ = other.buffer_count_;
    buffer_size_ = other.buffer_size_;
    other.gpu_memory_ = nullptr;
    other.buffer_count_ = 0;
    other.buffer_size_ = 0;
  }

  ~GpuMultibuffer()
  {
    if(gpu_memory_ == nullptr) {
      return;
    }
    const auto ret = cudaFree(gpu_memory_);
    if(ret != cudaSuccess) {
      std::cerr << "cudaFree failed: " << cudaGetErrorString(ret) << std::endl;
    }
  }

  GpuMultibuffer<T> & operator=(GpuMultibuffer<T> && other)
  {
    if(gpu_memory_ != nullptr) {
      const auto ret = cudaFree(gpu_memory_);
      if(ret != cudaSuccess) {
        throw std::runtime_error(std::string("cudaFree failed: ") + cudaGetErrorString(ret));
      }
    }
    gpu_memory_ = other.gpu_memory_;
    buffer_count_ = other.buffer_count_;
    buffer_size_ = other.buffer_size_;
    other.gpu_memory_ = nullptr;
    other.buffer_count_ = 0;
    other.buffer_size_ = 0;
    return *this;
  }

  T * Get()
  {
    return reinterpret_cast<T *>(gpu_memory_);
  }

  T * Get(std::size_t index)
  {
    return GetBufferStartPointer(index);
  }

  std::size_t BufferCount() const
  {
    return buffer_count_;
  }

  std::size_t BufferSize() const
  {
    return buffer_size_;
  }

  bool IsEmpty() const
  {
    return buffer_count_ == 0 || buffer_size_ == 0;
  }

  void CopyToGpu(const std::size_t buffer_index, const typename std::vector<T> & values)
  {
    if(values.size() != buffer_size_) {
      throw std::invalid_argument("Incorrect size of values given to GpuMultibuffer::CopyToGpu");
    }
    auto buffer_ptr = GetBufferStartPointer(buffer_index);
    const auto ret = cudaMemcpy(buffer_ptr, values.data(), buffer_size_ * sizeof(T),
        cudaMemcpyHostToDevice);
    if(ret != cudaSuccess) {
      throw std::runtime_error(std::string("cudaMemcpy failed: ") + cudaGetErrorString(ret));
    }
  }

  void CopyFromGpu(const std::size_t buffer_index, typename std::vector<T> & values)
  {
    auto buffer_ptr = GetBufferStartPointer(buffer_index);
    values.resize(buffer_size_);

    const auto ret = cudaMemcpy(values.data(), buffer_ptr, buffer_size_ * sizeof(T),
        cudaMemcpyDeviceToHost);
    if(ret != cudaSuccess) {
      throw std::runtime_error(std::string("cudaMemcpy failed: ") + cudaGetErrorString(ret));
    }
  }

  T CopyValueFromGpu(const std::size_t buffer_index, const std::size_t value_index)
  {
    auto buffer_ptr = GetBufferStartPointer(buffer_index);
    buffer_ptr += value_index;
    float dest = 0.f;
    const auto ret = cudaMemcpy(&dest, buffer_ptr, 1, cudaMemcpyDeviceToHost);
    if(ret != cudaSuccess) {
      throw std::runtime_error(std::string("cudaMemcpy failed: ") + cudaGetErrorString(ret));
    }
    return dest;
  }

private:
  void * gpu_memory_ = nullptr;
  std::size_t buffer_count_ = 0;
  std::size_t buffer_size_ = 0;

  T * GetBufferStartPointer(const std::size_t index)
  {
    if(index >= buffer_count_) {
      throw std::invalid_argument(
          "Index must be less than the number of buffers in GpuMultibuffer.");
    }
    return reinterpret_cast<T *>(gpu_memory_) + (index * buffer_size_);
  }
};

}  // namespace ateam_spatial

#endif  // ATEAM_SPATIAL__GPU_MULTIBUFFER_HPP_
