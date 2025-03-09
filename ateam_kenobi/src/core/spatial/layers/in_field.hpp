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

#ifndef CORE__SPATIAL__LAYERS__IN_FIELD_HPP_
#define CORE__SPATIAL__LAYERS__IN_FIELD_HPP_

#include "core/spatial/spatial_layer_factory.hpp"

namespace ateam_kenobi::spatial::layers
{

class InField final : public SpatialLayerFactory {
public:
  InField()
  : SpatialLayerFactory("InField") {}

  void FillLayer(cv::Mat & layer, const World & world) override
  {
    SetupLayer(layer, CV_8UC1);
    if(layer.size() == prev_size) {
      // No need to regenerate this layer unless the field size changes
      return;
    }
    prev_size = layer.size();

    const auto half_field_length = world.field.field_length / 2.0;
    const auto half_field_width = world.field.field_width / 2.0;


    cv::Rect field_bounds(
      cv::Point(WorldToLayerX(-half_field_length), WorldToLayerY(-half_field_width)),
      cv::Point(WorldToLayerX(half_field_length), WorldToLayerY(half_field_width))
    );

    layer = cv::Scalar(0);

    cv::rectangle(layer, field_bounds, cv::Scalar(255), cv::FILLED);
  }

private:
  cv::Size prev_size;
};
}  // namespace ateam_kenobi::spatial::layers

#endif  // CORE__SPATIAL__LAYERS__IN_FIELD_HPP_
