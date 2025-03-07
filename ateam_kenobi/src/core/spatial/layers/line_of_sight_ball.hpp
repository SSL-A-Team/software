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

#ifndef SPATIAL__LAYERS__LINE_OF_SIGHT_BALL_HPP_
#define SPATIAL__LAYERS__LINE_OF_SIGHT_BALL_HPP_

#include <algorithm>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include "core/spatial/spatial_layer_factory.hpp"
#include "robot_shadows.hpp"

namespace ateam_kenobi::spatial::layers
{

class LineOfSightBall final : public SpatialLayerFactory {
public:
  LineOfSightBall()
  : SpatialLayerFactory("LineOfSightBall") {}

  void FillLayer(cv::Mat & layer, const World & world) override
  {
    SetupLayer(layer, CV_8UC1);
    layer = cv::Scalar(255);  // fills layer with white
    AddRobotShadows(layer, world);
  }

private:
  void AddRobotShadows(cv::Mat & layer, const World & world)
  {
    const cv::Scalar black{0};
    for(const auto & robot : world.their_robots) {
      if(!robot.visible) {
        continue;
      }
      const auto robot_radius_layer = WorldToLayerDist(kRobotRadius);
      cv::circle(layer, WorldToLayer(robot.pos), robot_radius_layer, black, cv::FILLED);

      const auto [top_ray, bottom_ray] = GetRobotShadowRays(robot, world.ball.pos);
      cv::line(layer, WorldToLayer(top_ray.source()), WorldToLayer(top_ray.point(1)), black);
      cv::line(layer, WorldToLayer(bottom_ray.source()), WorldToLayer(bottom_ray.point(1)), black);

      const auto shadow_poly = GetRobotShadowPoly(robot, world.ball.pos, world);
      if(shadow_poly.empty()) {
        continue;
      }
      std::vector<cv::Point> shadow_poly_layer;
      shadow_poly_layer.reserve(shadow_poly.size());
      std::ranges::transform(shadow_poly, std::back_inserter(shadow_poly_layer),
        [this](const auto & wp){
          return WorldToLayer(wp);
      });
      cv::fillPoly(layer, shadow_poly_layer, black);
    }
  }
};

}  // namespace ateam_kenobi::spatial::layers

#endif  // SPATIAL__LAYERS__LINE_OF_SIGHT_BALL_HPP_
