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

#ifndef SPATIAL__LAYERS__DISTANCE_FROM_THEIR_BOTS_HPP_
#define SPATIAL__LAYERS__DISTANCE_FROM_THEIR_BOTS_HPP_

#include <algorithm>
#include <limits>
#include "spatial/spatial_layer_factory.hpp"

namespace ateam_kenobi::spatial::layers
{

class DistanceFromTheirBots final : public SpatialLayerFactory {
public:
  DistanceFromTheirBots()
  : SpatialLayerFactory("DistanceFromTheirBots") {}

  void FillLayer(cv::Mat & layer, const World & world) override
  {
    SetupLayer(layer, CV_32FC1);
    for(auto r = 0; r < layer.rows; ++r) {
      auto row = layer.ptr<float>(r);
      for(auto c = 0; c < layer.cols; ++c) {
        row[c] = GetDistanceFromNearestBot(ateam_geometry::Point(LayerToWorldX(c),
            LayerToWorldY(r)), world);
      }
    }
  }

private:
  float GetDistanceFromNearestBot(const ateam_geometry::Point & p, const World & world)
  {
    float distance = std::numeric_limits<float>::max();
    for(const auto & robot : world.their_robots) {
      if(!robot.visible) {
        continue;
      }
      distance = std::min(distance,
          static_cast<float>(CGAL::approximate_sqrt(CGAL::squared_distance(p, robot.pos))));
    }
    return distance;
  }
};

}  // namespace ateam_kenobi::spatial::layers

#endif  // SPATIAL__LAYERS__DISTANCE_FROM_THEIR_BOTS_HPP_
