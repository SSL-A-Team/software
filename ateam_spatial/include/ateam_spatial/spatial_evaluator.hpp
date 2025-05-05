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
#include "gpu_vector.hpp"
#include "types.hpp"
#include "map_id.hpp"

namespace ateam_spatial
{

  class SpatialEvaluator
  {
  public:
    SpatialEvaluator();

    void UpdateMaps(const FieldDimensions &field,
                    const Ball &ball,
                    const std::array<Robot, 16> &our_bots,
                    const std::array<Robot, 16> &their_bots);

    void CopyMapBuffer(const MapId map, std::vector<float> & destination);

    void RenderMapBuffer(const MapId map, std::vector<uint8_t> & destination);

    const SpatialSettings & GetSettings() const {
      return settings_;
    }

    Point GetMaxLocation(const MapId map);

    float GetValueAtLocation(const MapId map, const Point & location);

  private:
    bool cuda_device_available_;
    SpatialSettings settings_;
    FieldDimensions cached_field_dims_;
    GpuObject<SpatialSettings> gpu_spatial_settings_;
    GpuObject<FieldDimensions> gpu_field_dims_;
    GpuObject<Ball> gpu_ball_;
    GpuArray<Robot, 16> gpu_our_bots_;
    GpuArray<Robot, 16> gpu_their_bots_;
    GpuMultibuffer<float> gpu_map_buffers_;
    GpuVector<uint8_t> gpu_render_buffer_;

    bool IsReady();

    void UpdateBufferSizes(const FieldDimensions &field);

    std::size_t GetMaxLoc(const MapId map);
    std::size_t GetMinLoc(const MapId map);
  };

} // namespace ateam_spatial

#endif // ATEAM_SPATIAL__SPATIAL_EVALUATOR_HPP_
