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
#include "ateam_spatial/placeholder.hpp"

namespace ateam_spatial
{

SpatialEvaluator::SpatialEvaluator()
{
  settings_.resolution = 1e-3; // TODO(barulicm): parameterize this
}

void SpatialEvaluator::UpdateMaps(const FieldDimensions &field, const Ball &ball, const std::array<Robot, 16> &our_bots, const std::array<Robot, 16> &their_bots)
{
  UpdateBufferSizes(field);
  gpu_spatial_settings_.CopyToGpu(settings_);
  gpu_field_dims_.CopyToGpu(field);
  gpu_ball_.CopyToGpu(ball);
  gpu_our_bots_.CopyToGpu(our_bots);
  gpu_their_bots_.CopyToGpu(their_bots);

  const dim3 block_size(32,32,1);
  const auto grid_x = std::ceil(settings_.width / block_size.x);
  const auto grid_y = std::ceil(settings_.height / block_size.y);
  const auto grid_z = static_cast<std::size_t>(MapId::MapCount);
  const dim3 grid_size(grid_x, grid_y, grid_z);
  update_maps_kernel<<<grid_size, block_size>>>(gpu_spatial_settings_.Get(), gpu_field_dims_.Get(), gpu_ball_.Get(), gpu_our_bots_.Get(), gpu_their_bots_.Get(), gpu_map_buffers_.Get(), gpu_map_buffers_.BufferCount(), gpu_map_buffers_.BufferSize());
  if(const auto ret = cudaDeviceSynchronize();ret != cudaSuccess) {
    // TODO(barulicm): re-read cuda error handling blog post
    throw std::runtime_error(std::string("Failed to launch update maps kernel: ") + cudaGetErrorString(ret));
  }
  if(const auto ret = cudaGetLastError(); ret != cudaSuccess) {
    throw std::runtime_error(std::string("Failed to run update maps kernel: ") + cudaGetErrorString(ret));
  }
}

void SpatialEvaluator::CopyMapBuffer(const MapId & map, std::vector<float> & destination)
{
  gpu_map_buffers_.CopyFromGpu(static_cast<std::size_t>(map), destination);
}


void SpatialEvaluator::RenderMapBuffer(const MapId & map, std::vector<uint8_t> & destination)
{
  GpuVector<uint8_t> gpu_destination(settings_.width * settings_.height);

  const dim3 block_size(32, 32);
  const auto grid_x = std::ceil(settings_.width / block_size.x);
  const auto grid_y = std::ceil(settings_.height / block_size.y);
  const dim3 grid_size(grid_x, grid_y);
  render_kernel<<<grid_size, block_size>>>(map, gpu_spatial_settings_.Get(), gpu_map_buffers_.Get(), gpu_map_buffers_.BufferCount(), gpu_map_buffers_.BufferSize(), gpu_destination.Get());
  if(const auto ret = cudaDeviceSynchronize();ret != cudaSuccess) {
    // TODO(barulicm): re-read cuda error handling blog post
    throw std::runtime_error(std::string("Failed to launch render kernel: ") + cudaGetErrorString(ret));
  }
  if(const auto ret = cudaGetLastError(); ret != cudaSuccess) {
    throw std::runtime_error(std::string("Failed to run render kernel: ") + cudaGetErrorString(ret));
  }

  gpu_destination.CopyFromGpu(destination);
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
  settings_.width = world_width_spatial;
  settings_.height = world_height_spatial;
}

}  // namespace ateam_spatial
