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

#ifndef ATEAM_SPATIAL__LAYERS__DISTANCE_FROM_THEIR_BOTS_HPP_
#define ATEAM_SPATIAL__LAYERS__DISTANCE_FROM_THEIR_BOTS_HPP_

#include "ateam_spatial/types.hpp"
#include "ateam_spatial/cuda_hostdev.hpp"

namespace ateam_spatial::layers
{

CUDA_HOSTDEV float DistanceFromBot(
  const int x, const int y, const Robot & bot,
  const SpatialSettings & settings);

__global__
void DistanceFromTheirBotsKernel(unsigned char* Pout, unsigned char* Pin, Robot* & their_robots,
    const SpatialSettings & settings);
  

}  // namespace ateam_spatial::layers

#endif  // ATEAM_SPATIAL__LAYERS__DISTANCE_DOWN_FIELD_HPP_
