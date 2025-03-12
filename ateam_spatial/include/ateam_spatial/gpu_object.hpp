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

#ifndef ATEAM_SPATIAL__GPU_OBJECT_HPP_
#define ATEAM_SPATIAL__GPU_OBJECT_HPP_

#include <cuda_runtime_api.h>
#include <cassert>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace ateam_spatial
{

  template <typename T>
  class GpuObject
  {
  public:
    GpuObject()
    {
      assert(std::is_trivial_v<T>);
      const auto ret = cudaMalloc(&gpu_memory_, sizeof(T));
      if (ret != cudaSuccess)
      {
        throw std::runtime_error(std::string("cudaMalloc failed: ") + cudaGetErrorString(ret));
      }
    }

    explicit GpuObject(const T &value)
        : GpuObject()
    {
      CopyToGpu(value);
    }

    GpuObject(const GpuObject &) = delete;

    GpuObject(GpuObject &other)
    {
      gpu_memory_ = other.gpu_memory_;
      other.gpu_memory_ = nullptr;
    }

    ~GpuObject()
    {
      if (gpu_memory_ == nullptr)
      {
        return;
      }
      const auto ret = cudaFree(gpu_memory_);
      if (ret != cudaSuccess)
      {
        std::cerr << "cudaFree failed: " << cudaGetErrorString(ret) << std::endl;
      }
    }

    GpuObject<T>& operator=(GpuObject<T> &&other)
    {
      if (gpu_memory_ != nullptr)
      {
        const auto ret = cudaFree(gpu_memory_);
        if (ret != cudaSuccess)
        {
          throw std::runtime_error(std::string("cudaFree failed: ") + cudaGetErrorString(ret));
        }
      }
      gpu_memory_ = other.gpu_memory_;
      other.gpu_memory_ = nullptr;
      return *this;
    }

    T *Get()
    {
      return reinterpret_cast<T *>(gpu_memory_);
    }

    void CopyToGpu(const T &value)
    {
      const auto ret = cudaMemcpy(gpu_memory_, &value, sizeof(T), cudaMemcpyHostToDevice);
      if (ret != cudaSuccess)
      {
        throw std::runtime_error(std::string("cudaMemcpy failed: ") + cudaGetErrorString(ret));
      }
    }

    void CopyFromGpu(T &value)
    {
      const auto ret = cudaMemcpy(&value, gpu_memory_, sizeof(T), cudaMemcpyDeviceToHost);
      if (ret != cudaSuccess)
      {
        throw std::runtime_error(std::string("cudaMemcpy failed: ") + cudaGetErrorString(ret));
      }
    }

  private:
    void *gpu_memory_ = nullptr;
  };

} // namespace ateam_spatial

#endif // ATEAM_SPATIAL__GPU_OBJECT_HPP_
