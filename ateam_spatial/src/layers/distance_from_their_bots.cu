// Copyright 2024 A Team
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

#include "types.hpp"
#include "coordinate_coversions.hpp"

namespace ateam_spatial::layers {

CUDA_HOSTDEV float DistanceFromBot(
  const int x, const int y, const Robot & bot)
{
    return hypotf(x - bot.x, y - bot.y);
}

  __global__
void DistanceFromTheirBotsKernel(unsigned char* Pout, unsigned char* Pin, Robot* & their_robots,
    const SpatialSettings & settings)
{
  int col = blockIdx.x * blockDim.x + threadIdx.x;
  int row = blockIdx.y * blockDim.y + threadIdx.y;
  int num_robots = sizeof(their_robots) / sizeof(their_robots[0]);

  if (col < width && row < height){
    int out_coord_array_pos = row * settings.width + col;

    // Since the width is bigger than the height, we'll assume this is the max possible value
    float distance = settings.width * settings.resolution;

    for (int i; i < num_robots; ++i){
        float robot_dist = DistanceFromBot(col, row, their_robots[i])
        if (robot_dist > distance) {
            distance = robot_dist;
        }
    }
    Pout[out_coord_array_pos] = SpatialToRealDist(distance);
  }
}

}  // namespace ateam_spatial::layers
