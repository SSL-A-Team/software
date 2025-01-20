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

#ifndef LAYERS__LINE_OF_SIGHT_THEIR_GOAL_HPP_
#define LAYERS__LINE_OF_SIGHT_THEIR_GOAL_HPP_

#include <ateam_common/robot_constants.hpp>
#include "spatial/spatial_layer_factory.hpp"
#include "robot_shadows.hpp"

namespace ateam_kenobi::spatial::layers
{

class LineOfSightTheirGoal : public SpatialLayerFactory {
public:
  LineOfSightTheirGoal()
  : SpatialLayerFactory("LineOfSightTheirGoal") {}

  void FillLayer(cv::Mat & layer, const World & world) override
  {
    SetupLayer(layer, CV_8UC1);

    layer = cv::Scalar(255);  // fills layer with white

    FillBackBoundryArea(layer, world);
    AddCornerShadows(layer, world);
    AddRobotShadows(layer, world);
  }

private:
  const float kMinimumShotAngle = 0.5236;  // 30 deg

  void FillBackBoundryArea(cv::Mat & layer, const World & world) {
    const auto half_field_length = world.field.field_length/2.0;
    const auto half_world_width = (world.field.field_width/2.0) + world.field.boundary_width;
    const cv::Point2d top_left(half_field_length, -half_world_width);
    const cv::Point2d bottom_right(half_field_length + world.field.boundary_width, half_world_width);
    const auto top_left_layer = WorldToLayer(top_left);
    const auto bottom_right_layer = WorldToLayer(bottom_right);
    cv::rectangle(layer, top_left_layer, bottom_right_layer, cv::Scalar(0), cv::FILLED);
  }

  void AddCornerShadows(cv::Mat & layer, const World & world)
  {
    const auto half_field_length_layer = WorldToLayerX(world.field.field_length/2.0);
    const cv::Point top_field_corner{half_field_length_layer, 0};
    const cv::Point bottom_field_corner{half_field_length_layer, LayerHeight()};

    const auto half_goal_height_world = world.field.goal_width / 2.0;
    const cv::Point top_goal_corner{half_field_length_layer, WorldToLayerY(half_goal_height_world)};
    const cv::Point bottom_goal_corner{half_field_length_layer, WorldToLayerY(-half_goal_height_world)};

    const auto side_corner_x_world = ((WorldWidth() - half_goal_height_world) / 2.0) * tan(kMinimumShotAngle);
    const auto side_corner_x = WorldToLayerX((world.field.field_length / 2.0) - side_corner_x_world);
    const cv::Point top_side_corner{side_corner_x, 0};
    const cv::Point bottom_side_corner{side_corner_x, LayerHeight()};

    const cv::Scalar black{0};
    cv::fillPoly(layer, std::vector<cv::Point>{top_field_corner, top_goal_corner, top_side_corner}, black);
    cv::fillPoly(layer, std::vector<cv::Point>{bottom_field_corner, bottom_goal_corner, bottom_side_corner}, black);
  }

  void AddRobotShadows(cv::Mat & layer, const World & world) {
    const cv::Scalar black{0};
    const auto half_field_length = world.field.field_length / 2.0;
    const ateam_geometry::Segment goal_segment{
      ateam_geometry::Point{half_field_length, -world.field.goal_width / 2.0},
      ateam_geometry::Point{half_field_length, world.field.goal_width / 2.0}
    };
    for(const auto & robot : world.their_robots) {
      if(!robot.visible) {
        continue;
      }
      const auto robot_radius_layer = WorldToLayerDist(kRobotRadius);
      cv::circle(layer, WorldToLayer(robot.pos), robot_radius_layer, black, cv::FILLED);
      if(robot.pos.x() > half_field_length) {
        continue;
      }
      const auto shadow_poly = GetRobotShadowPoly(robot, goal_segment, world);
      if(shadow_poly.empty()) {
        continue;
      }
      std::vector<cv::Point> shadow_poly_layer;
      shadow_poly_layer.reserve(shadow_poly.size());
      std::ranges::transform(shadow_poly, std::back_inserter(shadow_poly_layer), [this](const auto & wp){
        return WorldToLayer(wp);
      });
      cv::fillPoly(layer, shadow_poly_layer, black);
    }
  }

};

}  // namespace ateam_kenobi::spatial

#endif  // LAYERS__LINE_OF_SIGHT_THEIR_GOAL_HPP_
