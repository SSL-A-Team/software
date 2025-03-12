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

#ifndef ATEAM_SPATIAL__SPATIAL_EVALUATOR_HPP_
#define ATEAM_SPATIAL__SPATIAL_EVALUATOR_HPP_

#include <string>
#include <vector>
#include "gpu_array.hpp"
#include "gpu_object.hpp"
#include "gpu_multibuffer.hpp"
#include "types.hpp"

namespace ateam_spatial
{

  class SpatialEvaluator
  {
  public:
    enum class Maps : std::size_t {
      ReceiverPositionQuality = 0,
      MapCount
    };

    SpatialEvaluator();

    void UpdateMaps(const FieldDimensions &field,
                    const Ball &ball,
                    const std::array<Robot, 16> &our_bots,
                    const std::array<Robot, 16> &their_bots);

    void CopyMapBuffer(const Maps & map, std::vector<float> & destination);

  private:
    SpatialSettings settings_;
    FieldDimensions cached_filed_dims_;
    GpuObject<FieldDimensions> gpu_field_dims_;
    GpuObject<Ball> gpu_ball_;
    GpuArray<Robot, 16> gpu_our_bots_;
    GpuArray<Robot, 16> gpu_their_bots_;
    GpuMultibuffer<float> gpu_map_buffers_;

    void UpdateBufferSizes(const FieldDimensions &field);
  };

} // namespace ateam_spatial

#endif // ATEAM_SPATIAL__SPATIAL_EVALUATOR_HPP_
