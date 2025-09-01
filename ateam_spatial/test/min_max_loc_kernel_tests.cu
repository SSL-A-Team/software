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

#include <gtest/gtest.h>
#include <random>
#include <ateam_spatial/min_max_loc_kernel.hpp>
#include <ateam_spatial/gpu_array.hpp>
#include <ateam_spatial/gpu_object.hpp>

TEST(MinMaxLocKernelTests, MinLoc)
{
  const std::array<float, 10> data{0.f, 18.f, 4.f, 14.f, 8.f, 12.f, 10.f, 6.f, 16.f, 2.f};
  ateam_spatial::GpuArray<float, 10> gpu_data;
  gpu_data.CopyToGpu(data);
  constexpr int threads_per_block = 64;
  const dim3 block_size(threads_per_block);
  const dim3 grid_size(std::ceil(static_cast<float>(data.size()) / block_size.x));
  ateam_spatial::GpuObject<uint32_t> gpu_min_index;
  ateam_spatial::min_loc_kernel<threads_per_block><<<grid_size, block_size>>>(gpu_data.Get(), gpu_data.Size(), gpu_min_index.Get());
  if (const auto ret = cudaDeviceSynchronize(); ret != cudaSuccess)
  {
    FAIL() << "Failed to launch min_loc_kernel: " << cudaGetErrorString(ret);
  }
  if (const auto ret = cudaGetLastError(); ret != cudaSuccess)
  {
    FAIL() << "Failed to run min_loc_kernel: " << cudaGetErrorString(ret);
  }
  uint32_t min_index = 0;
  gpu_min_index.CopyFromGpu(min_index);
  EXPECT_EQ(min_index, 0);
}

TEST(MinMaxLocKernelTests, MaxLoc)
{
  const std::array<float, 10> data{0.f, 18.f, 4.f, 14.f, 8.f, 12.f, 10.f, 6.f, 16.f, 2.f};
  ateam_spatial::GpuArray<float, 10> gpu_data;
  gpu_data.CopyToGpu(data);
  constexpr int threads_per_block = 64;
  const dim3 block_size(threads_per_block);
  const dim3 grid_size(std::ceil(static_cast<float>(data.size()) / block_size.x));
  ateam_spatial::GpuObject<uint32_t> gpu_max_index;
  ateam_spatial::max_loc_kernel<threads_per_block><<<grid_size, block_size>>>(gpu_data.Get(), gpu_data.Size(), gpu_max_index.Get());
  if (const auto ret = cudaDeviceSynchronize(); ret != cudaSuccess)
  {
    FAIL() << "Failed to launch max_loc_kernel: " << cudaGetErrorString(ret);
  }
  if (const auto ret = cudaGetLastError(); ret != cudaSuccess)
  {
    FAIL() << "Failed to run max_loc_kernel: " << cudaGetErrorString(ret);
  }
  uint32_t max_index = 0;
  gpu_max_index.CopyFromGpu(max_index);
  EXPECT_EQ(max_index, 1);
}

TEST(MinMaxLocKernelTests, MaxLoc_MultipleBlocks)
{
  std::array<float, 1440000> data{};
  std::random_device rdev;
  std::default_random_engine randeng(rdev());
  std::uniform_real_distribution<float> randist(0.0, 200.0);
  std::generate(data.begin(), data.end(), [&]()
                { return randist(randeng); });
  // Fill the first half with zeros to ensure the max is not in the first block
  std::fill_n(data.begin(), data.size() / 2, 0.f);
  ateam_spatial::GpuArray<float, 1440000> gpu_data;
  gpu_data.CopyToGpu(data);
  constexpr int threads_per_block = 64;
  const dim3 block_size(threads_per_block);
  const dim3 grid_size(std::ceil(static_cast<float>(data.size()) / block_size.x));
  ateam_spatial::GpuObject<uint32_t> gpu_max_index;
  ateam_spatial::max_loc_kernel<threads_per_block><<<grid_size, block_size>>>(gpu_data.Get(), gpu_data.Size(), gpu_max_index.Get());
  if (const auto ret = cudaDeviceSynchronize(); ret != cudaSuccess)
  {
    FAIL() << "Failed to launch max_loc_kernel: " << cudaGetErrorString(ret);
  }
  if (const auto ret = cudaGetLastError(); ret != cudaSuccess)
  {
    FAIL() << "Failed to run max_loc_kernel: " << cudaGetErrorString(ret);
  }

  const auto max_iter = std::max_element(data.begin(), data.end());

  ASSERT_NE(max_iter, data.end()) << "Failed to find a max value.";
  const auto expected_max_index = std::distance(data.begin(), max_iter);

  uint32_t max_index = 0;
  gpu_max_index.CopyFromGpu(max_index);
  EXPECT_EQ(max_index, expected_max_index);
}

TEST(MinMaxLocKernelTests, MaxLoc_UnalignedDataSize)
{
  std::array<float, 72> data{};
  std::random_device rdev;
  std::default_random_engine randeng(rdev());
  std::uniform_real_distribution<float> randist(0.0, 200.0);
  std::generate(data.begin(), data.end(), [&]()
                { return randist(randeng); });
  ateam_spatial::GpuArray<float, 72> gpu_data;
  gpu_data.CopyToGpu(data);
  constexpr int threads_per_block = 64;
  const dim3 block_size(threads_per_block);
  const dim3 grid_size(std::ceil(static_cast<float>(data.size()) / block_size.x));
  ateam_spatial::GpuObject<uint32_t> gpu_max_index;
  ateam_spatial::max_loc_kernel<threads_per_block><<<grid_size, block_size>>>(gpu_data.Get(), gpu_data.Size(), gpu_max_index.Get());
  if (const auto ret = cudaDeviceSynchronize(); ret != cudaSuccess)
  {
    FAIL() << "Failed to launch max_loc_kernel: " << cudaGetErrorString(ret);
  }
  if (const auto ret = cudaGetLastError(); ret != cudaSuccess)
  {
    FAIL() << "Failed to run max_loc_kernel: " << cudaGetErrorString(ret);
  }

  const auto max_iter = std::max_element(data.begin(), data.end());

  ASSERT_NE(max_iter, data.end()) << "Failed to find a max value.";
  const auto expected_max_index = std::distance(data.begin(), max_iter);

  uint32_t max_index = 0;
  gpu_max_index.CopyFromGpu(max_index);
  EXPECT_EQ(max_index, expected_max_index);
}
