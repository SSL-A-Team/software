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

#ifndef ATEAM_SPATIAL__DISTANCE_TO_BOTS_KERNEL_HPP_
#define ATEAM_SPATIAL__DISTANCE_TO_BOTS_KERNEL_HPP_

namespace ateam_spatial
{

__global__ void distance_to_robots(const float * output, const Robot* robots, 
    const FieldDimensions field_dims, const SpatialSettings settings, int num_robots)
    {
        // From SSL rules, in m
        const float robot_radius = 0.09f;
    
        // Global pixel position
        int x = blockIdx.x * blockDim.x + threadIdx.x;
        int y = blockIdx.y * blockDim.y + threadIdx.y;
    
        if (x >= settings.width || y >= settings.height) return;
    
        // Compute distance from (x, y) to all robots
        float min_dist = 1e20f;
    
        // We assume the robots passed in have been filtered in advance to only
        // be valid/visible robots
        for (int i = 0; i < num_robots; ++i) {
            float dx = x * settings.resolution - robots[i].x;
            float dy = y * settings.resolution - robots[i].y;
            float dist = fabsf(sqrtf(dx * dx + dy * dy) - robot_radius);
            min_dist = fminf(min_dist, dist);
        }
    
        // Write to output buffer
        output[y * field_width + x] = min_dist;
    }
}