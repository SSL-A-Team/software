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
#include "ateam_spatial/update_maps_kernel.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial
{

SpatialEvaluator::SpatialEvaluator()
{
  settings_.resolution = 1e-3; // TODO(barulicm): parameterize this
}

void SpatialEvaluator::UpdateMaps(const FieldDimensions &field, const Ball &ball, const std::array<Robot, 16> &our_bots, const std::array<Robot, 16> &their_bots)
{
  UpdateBufferSizes(field);
  update_maps_kernel<<<1,1>>>();
  const auto ret = cudaDeviceSynchronize();
  if(ret != cudaSuccess) {
    // TODO(barulicm): re-read cuda error handling blog post
    throw std::runtime_error(std::string("cudaDeviceSynchronize failed: ") + cudaGetErrorString(ret));
  }
}

void SpatialEvaluator::CopyMapBuffer(const SpatialEvaluator::Maps & map, std::vector<float> & destination)
{
  gpu_map_buffers_.CopyFromGpu(static_cast<std::size_t>(map), destination);
}

void SpatialEvaluator::UpdateBufferSizes(const FieldDimensions &field)
{
  if(field == cached_filed_dims_) {
    return;
  }
  const auto world_width_spatial = WorldWidthSpatial(field, settings_);
  const auto world_height_spatial = WorldHeightSpatial(field, settings_);
  const auto buffer_size = world_width_spatial * world_height_spatial;
  gpu_map_buffers_ = GpuMultibuffer<float>(static_cast<std::size_t>(Maps::MapCount), buffer_size);
}

}  // namespace ateam_spatial
