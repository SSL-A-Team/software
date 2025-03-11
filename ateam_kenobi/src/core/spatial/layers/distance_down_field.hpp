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

#ifndef CORE__SPATIAL__LAYERS__DISTANCE_DOWN_FIELD_HPP_
#define CORE__SPATIAL__LAYERS__DISTANCE_DOWN_FIELD_HPP_

#include "core/spatial/spatial_layer_factory.hpp"

namespace ateam_kenobi::spatial::layers
{

class DistanceDownField final : public SpatialLayerFactory {
public:
  DistanceDownField()
  : SpatialLayerFactory("DistanceDownField") {}

  void FillLayer(cv::Mat & layer, const World &) override
  {
    SetupLayer(layer, CV_32FC1);
    if(layer.size() == prev_size) {
      // No need to regenerate this layer unless the field size changes
      return;
    }
    prev_size = layer.size();

    const auto half_world_width = WorldWidth() / 2.0;

    for(auto x = 0; x < LayerWidth(); ++x) {
      for(auto y = 0; y < LayerHeight(); ++y) {
        layer.at<float>(y, x) = LayerToWorldX(x) + half_world_width;
      }
    }
  }

private:
  cv::Size prev_size;
};

}  // namespace ateam_kenobi::spatial::layers

#endif  // CORE__SPATIAL__LAYERS__DISTANCE_DOWN_FIELD_HPP_
