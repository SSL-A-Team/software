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

namespace ateam_spatial
{

SpatialEvaluator::SpatialEvaluator()
{
  settings_.resolution = 1e-2; // TODO(barulicm): parameterize this
}

void SpatialEvaluator::UpdateMaps(const FieldDimensions &field, const Ball &ball, const std::array<Robot, 16> &our_bots, const std::array<Robot, 16> &their_bots)
{
  UpdateBufferSizes(field);
  if(settings_.height * settings_.width == 0) {
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
  if(settings_.height * settings_.width == 0) {
    return;
  }
  const auto min_max_loc = GetMinMaxLoc(map);

  const dim3 block_size(64);
  const dim3 grid_size(std::ceil(static_cast<float>(gpu_map_buffers_.BufferSize()) / block_size.x));
  render_kernel<<<grid_size, block_size>>>(gpu_map_buffers_.Get(static_cast<std::size_t>(map)), gpu_map_buffers_.BufferSize(), min_max_loc.min_value, min_max_loc.max_value, gpu_render_buffer_.Get());
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
  if(settings_.height * settings_.width == 0) {
    return Point{0.f,0.f};
  }
  const auto min_max_loc = GetMinMaxLoc(map);
  const auto max_y = min_max_loc.max_index / settings_.width;
  const auto max_x = min_max_loc.max_index % settings_.width;
  return Point{
    SpatialToRealX(max_x, cached_filed_dims_, settings_),
    SpatialToRealY(max_y, cached_filed_dims_, settings_)
  };
}

float SpatialEvaluator::GetValueAtLocation(const MapId map, const Point & location)
{
  if(settings_.height * settings_.width == 0) {
    return 0.f;
  }
  const auto index = (RealToSpatialY(location.y, cached_filed_dims_, settings_) * settings_.width)
                      + RealToSpatialX(location.x, cached_filed_dims_, settings_);
  return gpu_map_buffers_.CopyValueFromGpu(static_cast<std::size_t>(MapId::ReceiverPositionQuality), index);
}

void SpatialEvaluator::UpdateBufferSizes(const FieldDimensions &field)
{
  if(field == cached_filed_dims_) {
    return;
  }
  const auto world_width_spatial = WorldWidthSpatial(field, settings_);
  const auto world_height_spatial = WorldHeightSpatial(field, settings_);
  const auto buffer_size = world_width_spatial * world_height_spatial;
  gpu_map_buffers_ = GpuMultibuffer<float>(static_cast<std::size_t>(MapId::MapCount), buffer_size);
  gpu_render_buffer_ = GpuVector<uint8_t>(buffer_size);
  settings_.width = world_width_spatial;
  settings_.height = world_height_spatial;
}

SpatialEvaluator::MinMaxLocResult SpatialEvaluator::GetMinMaxLoc(const MapId map)
{
  float * gpu_buffer = gpu_map_buffers_.Get(static_cast<std::size_t>(map));
  const dim3 block_size(64);
  const dim3 grid_size(std::ceil(static_cast<float>(gpu_map_buffers_.BufferSize()) / block_size.x));
  GpuVector<float> gpu_block_maxs(grid_size.x);
  GpuVector<float> gpu_block_mins(grid_size.x);
  GpuVector<std::size_t> gpu_block_max_locs(grid_size.x);
  GpuVector<std::size_t> gpu_block_min_locs(grid_size.x);
  min_max_loc_kernel<<<grid_size, block_size>>>(gpu_buffer, gpu_map_buffers_.BufferSize(), gpu_block_mins.Get(), gpu_block_maxs.Get(), gpu_block_min_locs.Get(), gpu_block_max_locs.Get());
  if(const auto ret = cudaDeviceSynchronize();ret != cudaSuccess) {
    throw std::runtime_error(std::string("Failed to launch minmax kernel: ") + cudaGetErrorString(ret));
  }
  if(const auto ret = cudaGetLastError(); ret != cudaSuccess) {
    throw std::runtime_error(std::string("Failed to run minmax kernel: ") + cudaGetErrorString(ret));
  }
  std::vector<float> block_maxs;
  std::vector<float> block_mins;
  std::vector<std::size_t> block_max_locs;
  std::vector<std::size_t> block_min_locs;
  gpu_block_maxs.CopyFromGpu(block_maxs);
  gpu_block_mins.CopyFromGpu(block_mins);
  gpu_block_max_locs.CopyFromGpu(block_max_locs);
  gpu_block_min_locs.CopyFromGpu(block_min_locs);
  MinMaxLocResult result;
  result.max_value = block_maxs[0];
  result.min_value = block_mins[0];
  for(auto i = 1ul; i < grid_size.x; ++i)
  {
    if(result.max_value < block_maxs[i])
    {
      result.max_value = block_maxs[i];
      result.max_index = i;
    }
    if(result.min_value > block_mins[i])
    {
      result.min_value = block_mins[i];
      result.min_index = i;
    }
  }
  return result;
}

}  // namespace ateam_spatial
