// Copyright 2023 A Team
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

#include "stop_play.hpp"
#include <limits>
#include <ateam_msgs/msg/robot_motion_command.hpp>


namespace ateam_kenobi::plays
{
StopPlay::StopPlay()
: BasePlay("StopPlay")
{
  play_helpers::EasyMoveTo::CreateArray(easy_move_tos_, overlays_.getChild("EasyMoveTo"));
  for (auto & move_to : easy_move_tos_) {
    move_to.setMaxVelocity(1.0);
  }
  StopPlay::reset();
}

double StopPlay::getScore(const World & world)
{
  switch (world.referee_info.running_command) {
    case ateam_common::GameCommand::Stop:
    case ateam_common::GameCommand::BallPlacementOurs:
    case ateam_common::GameCommand::BallPlacementTheirs:
      return std::numeric_limits<double>::max();
    default:
      return std::numeric_limits<double>::lowest();
  }
}

void StopPlay::reset()
{
  for (auto & move_to : easy_move_tos_) {
    move_to.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> StopPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> stop_motion_commands;
  for (size_t robot_id = 0; robot_id < 16; ++robot_id) {
    // only going to do 8 robots for now to avoid crowding on one side of the ball
    // avoid the ball by 0.7m just to be safe
    double radius = 0.7;
    const auto & maybe_robot = world.our_robots[robot_id];

    if (maybe_robot.has_value()) {
      // move the robot if its in the danger zone halt if its not
      if (ateam_geometry::norm(maybe_robot.value().pos, world.ball.pos) < radius) {
        getPlayInfo()["robots"][std::to_string(robot_id)] = "moving";
        const auto & robot = maybe_robot.value();
        ateam_geometry::Vector offset_vector = radius * ateam_geometry::normalize(
          robot.pos - world.ball.pos);

        const auto & destination = ateam_geometry::Point(
          world.ball.pos.x() + offset_vector.x(), world.ball.pos.y() + offset_vector.y());

        auto & easy_move_to = easy_move_tos_.at(robot_id);
        easy_move_to.setTargetPosition(destination);
        stop_motion_commands.at(robot_id) = easy_move_to.runFrame(robot, world);
      } else {
        getPlayInfo()["robots"][std::to_string(robot_id)] = "safe";
        // literally halt if this one robot is not in the danger zone
        stop_motion_commands[robot_id] = ateam_msgs::msg::RobotMotionCommand{};
      }
      continue;
    }
    stop_motion_commands[robot_id] = std::nullopt;  // already done but just to be explicit
  }
  // Draw Keepout Circle
  getOverlays().drawCircle(
    "keepout_circle",
    ateam_geometry::makeCircle(world.ball.pos, 0.35), "red", "transparent");

  return stop_motion_commands;
}
}  // namespace ateam_kenobi::plays
