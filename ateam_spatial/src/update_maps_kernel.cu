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

#include "ateam_spatial/update_maps_kernel.hpp"
#include <cstdio>
#include "ateam_spatial/maps/receiver_position_quality.hpp"
#include "ateam_spatial/map_id.hpp"

namespace ateam_spatial
{

__global__ void update_maps_kernel(const SpatialSettings * spatial_settings,
  const FieldDimensions * field_dimensions, const Ball * ball,
  const Robot * our_bots, const Robot * their_bots,
  float * map_buffers, const std::size_t map_count,
  const std::size_t map_buffer_size)
{
  if(map_count != static_cast<std::size_t>(MapId::MapCount)) {
    printf("update_maps_kernel(): map_count does not match expected value.\n");
    return;
  }

  const auto map_index = blockIdx.z;
  if(map_index >= map_count) {
    return;
  }

  const auto spatial_x = (blockIdx.x * blockDim.x) + threadIdx.x;
  const auto spatial_y = (blockIdx.y * blockDim.y) + threadIdx.y;
  if(spatial_x >= spatial_settings->width || spatial_y >= spatial_settings->height) {
    return;
  }

  const auto map_buffer_index = (spatial_y * spatial_settings->width) + spatial_x;
  const auto output_buffer_index = (map_index * map_buffer_size) + map_buffer_index;

  float output = 0.0f;
  switch(static_cast<MapId>(map_index)) {
    case MapId::ReceiverPositionQuality:
      output = maps::ReceiverPositionQuality(spatial_x, spatial_y, *field_dimensions, *spatial_settings);
      break;
  }

  map_buffers[output_buffer_index] = output;
}

}  // namespace ateam_spatial
