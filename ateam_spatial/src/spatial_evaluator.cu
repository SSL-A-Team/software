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

#include "ateam_spatial/spatial_evaluator.hpp"
#include <cuda_runtime_api.h>
#include <algorithm>
#include "ateam_spatial/render_kernel.hpp"
#include "ateam_spatial/update_maps_kernel.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"
#include "ateam_spatial/gpu_vector.hpp"
#include "ateam_spatial/min_max_loc_kernel.hpp"
#include "ateam_spatial/device_availability.hpp"

namespace ateam_spatial
{

SpatialEvaluator::SpatialEvaluator()
{
  settings_.resolution = 5e-2; // TODO(barulicm): parameterize this
  cuda_device_available_ = IsCudaDeviceAvailable();
  if(!cuda_device_available_) {
    std::cerr << "Could not detect CUDA device. GPU-accelerated spatial code is disabled." << std::endl;
  }
}

void SpatialEvaluator::UpdateMaps(const FieldDimensions &field, const Ball &ball, const std::array<Robot, 16> &our_bots, const std::array<Robot, 16> &their_bots)
{
  UpdateBufferSizes(field);
  if(!IsReady()) {
    return;
  }
  gpu_spatial_settings_.CopyToGpu(settings_);
  gpu_field_dims_.CopyToGpu(field);
  gpu_ball_.CopyToGpu(ball);
  gpu_our_bots_.CopyToGpu(our_bots);
  gpu_their_bots_.CopyToGpu(their_bots);

  const dim3 block_size(32,32,1);
  const auto grid_x = std::ceil(static_cast<float>(settings_.width) / block_size.x);
  const auto grid_y = std::ceil(static_cast<float>(settings_.height) / block_size.y);
  const auto grid_z = static_cast<std::size_t>(MapId::MapCount);
  const dim3 grid_size(grid_x, grid_y, grid_z);
  update_maps_kernel<<<grid_size, block_size>>>(gpu_spatial_settings_.Get(), gpu_field_dims_.Get(), gpu_ball_.Get(), gpu_our_bots_.Get(), gpu_their_bots_.Get(), gpu_map_buffers_.Get(), gpu_map_buffers_.BufferCount(), gpu_map_buffers_.BufferSize());
  if(const auto ret = cudaDeviceSynchronize();ret != cudaSuccess) {
    throw std::runtime_error(std::string("Failed to launch update maps kernel: ") + cudaGetErrorString(ret));
  }
  if(const auto ret = cudaGetLastError(); ret != cudaSuccess) {
    throw std::runtime_error(std::string("Failed to run update maps kernel: ") + cudaGetErrorString(ret));
  }
}

void SpatialEvaluator::CopyMapBuffer(const MapId map, std::vector<float> & destination)
{
  gpu_map_buffers_.CopyFromGpu(static_cast<std::size_t>(map), destination);
}


void SpatialEvaluator::RenderMapBuffer(const MapId map, std::vector<uint8_t> & destination)
{
  if(!IsReady()) {
    return;
  }
  const auto max_index = GetMaxLoc(map);
  const auto min_index = GetMinLoc(map);
  const auto max_value = gpu_map_buffers_.CopyValueFromGpu(static_cast<std::size_t>(map), max_index);
  const auto min_value = gpu_map_buffers_.CopyValueFromGpu(static_cast<std::size_t>(map), min_index);

  const dim3 block_size(64);
  const dim3 grid_size(std::ceil(static_cast<float>(gpu_map_buffers_.BufferSize()) / block_size.x));
  render_kernel<<<grid_size, block_size>>>(gpu_map_buffers_.Get(static_cast<std::size_t>(map)), gpu_map_buffers_.BufferSize(), min_value, max_value, gpu_render_buffer_.Get());
  if(const auto ret = cudaDeviceSynchronize();ret != cudaSuccess) {
    throw std::runtime_error(std::string("Failed to launch render kernel: ") + cudaGetErrorString(ret));
  }
  if(const auto ret = cudaGetLastError(); ret != cudaSuccess) {
    throw std::runtime_error(std::string("Failed to run render kernel: ") + cudaGetErrorString(ret));
  }

  gpu_render_buffer_.CopyFromGpu(destination);
}

Point SpatialEvaluator::GetMaxLocation(const MapId map)
{
  if(!IsReady()) {
    return Point{0.f,0.f};
  }
  const auto max_index = GetMaxLoc(map);
  const auto max_y = max_index / settings_.width;
  const auto max_x = max_index % settings_.width;
  return Point{
    SpatialToRealX(max_x, cached_field_dims_, settings_),
    SpatialToRealY(max_y, cached_field_dims_, settings_)
  };
}

float SpatialEvaluator::GetValueAtLocation(const MapId map, const Point & location)
{
  if(!IsReady()) {
    return 0.f;
  }
  const auto index = (RealToSpatialY(location.y, cached_field_dims_, settings_) * settings_.width)
                      + RealToSpatialX(location.x, cached_field_dims_, settings_);
  return gpu_map_buffers_.CopyValueFromGpu(static_cast<std::size_t>(MapId::ReceiverPositionQuality), index);
}

bool SpatialEvaluator::IsReady() {
  return cuda_device_available_ && settings_.height != 0 && settings_.width != 0;
}

void SpatialEvaluator::UpdateBufferSizes(const FieldDimensions &field)
{
  if(field == cached_field_dims_) {
    return;
  }
  const auto world_width_spatial = WorldWidthSpatial(field, settings_);
  const auto world_height_spatial = WorldHeightSpatial(field, settings_);
  const auto buffer_size = world_width_spatial * world_height_spatial;
  gpu_map_buffers_ = GpuMultibuffer<float>(static_cast<std::size_t>(MapId::MapCount), buffer_size);
  gpu_render_buffer_ = GpuVector<uint8_t>(buffer_size);
  settings_.width = world_width_spatial;
  settings_.height = world_height_spatial;
  cached_field_dims_ = field;
}

std::size_t SpatialEvaluator::GetMaxLoc(const MapId map)
{
  const auto gpu_buffer = gpu_map_buffers_.Get(static_cast<std::size_t>(map));
  constexpr auto kThreadsPerBlock = 64;
  const dim3 block_size(kThreadsPerBlock);
  const dim3 grid_size(std::ceil(static_cast<float>(gpu_map_buffers_.BufferSize()) / block_size.x));
  GpuObject<uint32_t> gpu_max_index;
  max_loc_kernel<kThreadsPerBlock><<<grid_size, block_size>>>(gpu_buffer, gpu_map_buffers_.BufferSize(), gpu_max_index.Get());
  if(const auto ret = cudaDeviceSynchronize();ret != cudaSuccess) {
    throw std::runtime_error(std::string("Failed to launch max_loc_kernel: ") + cudaGetErrorString(ret));
  }
  if(const auto ret = cudaGetLastError(); ret != cudaSuccess) {
    throw std::runtime_error(std::string("Failed to run max_loc_kernel: ") + cudaGetErrorString(ret));
  }
  uint32_t result;
  gpu_max_index.CopyFromGpu(result);
  return result;
}

std::size_t SpatialEvaluator::GetMinLoc(const MapId map)
{
  const auto gpu_buffer = gpu_map_buffers_.Get(static_cast<std::size_t>(map));
  constexpr auto kThreadsPerBlock = 64;
  const dim3 block_size(kThreadsPerBlock);
  const dim3 grid_size(std::ceil(static_cast<float>(gpu_map_buffers_.BufferSize()) / block_size.x));
  GpuObject<uint32_t> gpu_min_index;
  min_loc_kernel<kThreadsPerBlock><<<grid_size, block_size>>>(gpu_buffer, gpu_map_buffers_.BufferSize(), gpu_min_index.Get());
  if(const auto ret = cudaDeviceSynchronize();ret != cudaSuccess) {
    throw std::runtime_error(std::string("Failed to launch max_loc_kernel: ") + cudaGetErrorString(ret));
  }
  if(const auto ret = cudaGetLastError(); ret != cudaSuccess) {
    throw std::runtime_error(std::string("Failed to run max_loc_kernel: ") + cudaGetErrorString(ret));
  }
  uint32_t result;
  gpu_min_index.CopyFromGpu(result);
  return result;
}

}  // namespace ateam_spatial
