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

#include "ateam_spatial/render_kernel.hpp"
#include <cstdio>

namespace ateam_spatial
{

__global__ void render_kernel(const MapId map, const SpatialSettings * settings, const float * map_buffers,
                              const std::size_t map_count, const std::size_t map_buffer_size,
                              uint8_t * output_buffer)
{
  if(map_count != static_cast<std::size_t>(MapId::MapCount)) {
    printf("update_maps_kernel(): map_count does not match expected value.");
    return;
  }

  const auto map_index = static_cast<std::size_t>(map);

  const auto spatial_x = (blockIdx.x * blockDim.x) + threadIdx.x;
  const auto spatial_y = (blockIdx.y * blockDim.y) + threadIdx.y;
  if(spatial_x >= settings->width || spatial_y >= settings->height) {
    return;
  }

  const auto map_buffer_index = (spatial_y * settings->width) + spatial_x;
  const auto input_buffer_index = (map_index * map_buffer_size) + map_buffer_index;

  // TODO(barulicm): How to efficiently find the max value for scaling?
  output_buffer[map_buffer_index] = map_buffers[input_buffer_index] * (256 / 16.0);
}

}  // namespace ateam_spatial
