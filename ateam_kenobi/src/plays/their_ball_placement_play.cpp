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


#include "their_ball_placement_play.hpp"
#include <angles/angles.h>
#include <ateam_common/robot_constants.hpp>
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

TheirBallPlacementPlay::TheirBallPlacementPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options)
{
  createIndexedChildren<play_helpers::EasyMoveTo>(easy_move_tos_, "EasyMoveTo");
}

stp::PlayScore TheirBallPlacementPlay::getScore(const World & world)
{
  const auto & cmd = world.referee_info.running_command;
  if (cmd == ateam_common::GameCommand::BallPlacementTheirs) {
    return stp::PlayScore::Max();
  }
  return stp::PlayScore::NaN();
}

void TheirBallPlacementPlay::reset()
{
  for (auto & emt : easy_move_tos_) {
    emt.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> TheirBallPlacementPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);

  const ateam_geometry::Point placement_point = [this, &world]() {
      if (world.referee_info.designated_position.has_value()) {
        return world.referee_info.designated_position.value();
      } else {
        RCLCPP_WARN(
        getLogger(),
        "No designated position set in referee info, using ball position.");
        return world.ball.pos;
      }
    }();

  const auto point_to_ball = world.ball.pos - placement_point;
  const auto angle = std::atan2(point_to_ball.y(), point_to_ball.x());

  ateam_geometry::Polygon polygon_points;
  polygon_points.push_back(
    placement_point +
    0.5 * ateam_geometry::Vector(std::cos(angle + M_PI / 2), std::sin(angle + M_PI / 2)));
  polygon_points.push_back(
    placement_point +
    0.5 * ateam_geometry::Vector(std::cos(angle - M_PI / 2), std::sin(angle - M_PI / 2)));
  polygon_points.push_back(
    world.ball.pos +
    0.5 * ateam_geometry::Vector(std::cos(angle - M_PI / 2), std::sin(angle - M_PI / 2)));
  polygon_points.push_back(
    world.ball.pos +
    0.5 * ateam_geometry::Vector(std::cos(angle + M_PI / 2), std::sin(angle + M_PI / 2)));

  getOverlays().drawCircle(
    "placement_avoid_point", ateam_geometry::makeCircle(
      placement_point,
      0.5), "red", "red");
  getOverlays().drawCircle(
    "placement_avoid_ball", ateam_geometry::makeCircle(
      world.ball.pos,
      0.5), "red", "red");
  getOverlays().drawPolygon("placement_avoid_zone", polygon_points, "red", "red");

  for (auto ind = 0ul; ind < available_robots.size(); ++ind) {
    const auto & robot = available_robots[ind];
    auto & emt = easy_move_tos_[robot.id];

    const auto placement_segment = ateam_geometry::Segment(placement_point, world.ball.pos);
    const auto nearest_point = ateam_geometry::nearestPointOnSegment(placement_segment, robot.pos);

    ateam_geometry::Point target_position = robot.pos;
    if (ateam_geometry::norm(robot.pos - nearest_point) < 0.6 + kRobotRadius) {
      target_position = nearest_point +
        0.7 * ateam_geometry::Vector(std::cos(angle + M_PI / 2), std::sin(angle + M_PI / 2));

      const auto alternate_position = nearest_point +
        0.7 * ateam_geometry::Vector(std::cos(angle - M_PI / 2), std::sin(angle - M_PI / 2));


      if (ateam_geometry::norm(target_position - robot.pos) >
        ateam_geometry::norm(alternate_position - robot.pos))
      {
        target_position = alternate_position;
      }

      getPlayInfo()["Robots"][std::to_string(robot.id)] = "MOVING";
    } else {
      getPlayInfo()["Robots"][std::to_string(robot.id)] = "-";
    }

    emt.setTargetPosition(target_position);
    emt.face_point(world.ball.pos);
    maybe_motion_commands[robot.id] = emt.runFrame(robot, world);
  }

  return maybe_motion_commands;
}

}  // namespace ateam_kenobi::plays
