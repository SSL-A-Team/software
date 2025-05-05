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

#ifndef PLAYS__TEST_PLAYS__TEST_SPATIAL_MAP_PLAY_HPP_
#define PLAYS__TEST_PLAYS__TEST_SPATIAL_MAP_PLAY_HPP_

#include <vector>
#include "core/path_planning/path_planner.hpp"
#include "core/motion/motion_controller.hpp"
#include "core/stp/play.hpp"
#include <opencv2/opencv.hpp>
#include <ateam_spatial/spatial_evaluator.hpp>

namespace ateam_kenobi::plays
{
class TestSpatialMapPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "TestSpatialMapPlay";

  explicit TestSpatialMapPlay(stp::Options stp_options)
  : stp::Play(kPlayName, stp_options)
  {
  }

  void reset() override {}

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override
  {
    const auto half_field_length = (world.field.field_length / 2.0) + world.field.boundary_width;
    const auto half_field_width = (world.field.field_width / 2.0) + world.field.boundary_width;
    ateam_geometry::Rectangle bounds {
      ateam_geometry::Point{-half_field_length, -half_field_width},
      ateam_geometry::Point{half_field_length, half_field_width}
    };

    const auto map_id = ateam_spatial::MapId::ReceiverPositionQuality;

    std::vector<uint8_t> rendered_map;
    world.spatial_evaluator->RenderMapBuffer(map_id, rendered_map);

    const auto spatial_settings = world.spatial_evaluator->GetSettings();

    getOverlays().drawHeatmap("heatmap", bounds, rendered_map, spatial_settings.width,
        spatial_settings.height);

    const auto max_pos = world.spatial_evaluator->GetMaxLocation(map_id);

    getOverlays().drawCircle("heatmap_max",
        ateam_geometry::makeCircle(ateam_geometry::Point(max_pos.x, max_pos.y), kRobotRadius));

    return {};
  }
};
}  // namespace ateam_kenobi::plays

#endif  // PLAYS__TEST_PLAYS__TEST_SPATIAL_MAP_PLAY_HPP_
