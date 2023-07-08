// Copyright 2021 A Team
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


#include "goalie.hpp"

namespace ateam_kenobi::skills
{

Goalie::Goalie(
  visualization::OverlayPublisher & overlay_publisher,
  visualization::PlayInfoPublisher & play_info_publisher)
: overlay_publisher_(overlay_publisher), play_info_publisher_(play_info_publisher),
  easy_move_to_(overlay_publisher)
{
  reset();
}

void Goalie::reset()
{
  easy_move_to_.reset();
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.use_default_obstacles = false;
  easy_move_to_.setPlannerOptions(planner_options);
}

void Goalie::runFrame(
  const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  const auto robot_id = world.referee_info.our_goalie_id;
  const auto & maybe_robot = world.our_robots.at(robot_id);
  if (!maybe_robot) {
    // Assigned robot is not visible
    return;
  }

  ateam_geometry::Segment goalie_line = ateam_geometry::Segment(
    ateam_geometry::Point(-(world.field.field_length / 2.0) + 0.25, world.field.goal_width / 2.0),
    ateam_geometry::Point(-(world.field.field_length / 2.0) + 0.25, -world.field.goal_width / 2.0)
  );
  overlay_publisher_.drawLine("goalie_line", {goalie_line.point(0), goalie_line.point(1)}, "blue");

  easy_move_to_.setTargetPosition(
    ateam_geometry::NearestPointOnSegment(
      goalie_line,
      world.ball.pos));
  easy_move_to_.face_point(world.ball.pos);
  motion_commands.at(robot_id) = easy_move_to_.runFrame(maybe_robot.value(), world);
}

} // namespace ateam_kenobi::skills
