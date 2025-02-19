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

#ifndef SPATIAL__LAYERS__THEIR_DEFENSE_AREA_KEEPOUT_HPP_
#define SPATIAL__LAYERS__THEIR_DEFENSE_AREA_KEEPOUT_HPP_

#include "spatial/spatial_layer_factory.hpp"

namespace ateam_kenobi::spatial::layers
{

class TheirDefenseAreaKeepout : public SpatialLayerFactory {
public:
  TheirDefenseAreaKeepout()
  : SpatialLayerFactory("TheirDefenseAreaKeepout") {}

  void FillLayer(cv::Mat & layer, const World & world) override
  {
    SetupLayer(layer, CV_8UC1);

    if(world.field.field_length == prev_field.field_length ||
      world.field.field_width == prev_field.field_width ||
      world.field.defense_area_width == prev_field.defense_area_width ||
      world.field.defense_area_depth == prev_field.defense_area_depth ||
      world.field.boundary_width == prev_field.boundary_width)
    {
        // Geometry has not been changed, no need to update layer.
      return;
    }
    prev_field = world.field;

    const auto half_field_length = world.field.field_length / 2.0;
    const auto half_defense_area_width = world.field.defense_area_width / 2.0;

    layer = cv::Scalar(255);

    const ateam_geometry::Point top_left(half_field_length - world.field.defense_area_depth -
      padding_, -half_defense_area_width - padding_);
    const ateam_geometry::Point bottom_right(half_field_length, half_defense_area_width + padding_);

    const auto top_left_layer = WorldToLayer(top_left);
    const auto bottom_right_layer = WorldToLayer(bottom_right);

    cv::rectangle(layer, top_left_layer, bottom_right_layer, cv::Scalar(0), cv::FILLED);
  }

private:
  Field prev_field;
  const double padding_ = kRobotDiameter;
};

}  // namespace ateam_kenobi::spatial::layers

#endif  // SPATIAL__LAYERS__THEIR_DEFENSE_AREA_KEEPOUT_HPP_
