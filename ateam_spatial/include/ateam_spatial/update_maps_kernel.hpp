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

#ifndef ATEAM_SPATIAL__UPDATE_MAPS_KERNEL_HPP_
#define ATEAM_SPATIAL__UPDATE_MAPS_KERNEL_HPP_

#include "types.hpp"

#ifdef __cplusplus
extern "C" {
#endif

namespace ateam_spatial
{

__global__ void update_maps_kernel(const SpatialSettings * spatial_settings,
                                   const FieldDimensions * field_dimensions, const Ball * ball,
                                   const Robot * our_bots, const Robot * their_bots,
                                   float * map_buffers, const std::size_t map_count,
                                   const std::size_t map_buffer_size);

}  // namespace ateam_spatial

#ifdef __cplusplus
}
#endif

#endif  // ATEAM_SPATIAL__UPDATE_MAPS_KERNEL_HPP_
