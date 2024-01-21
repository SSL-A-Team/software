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


#include "our_kickoff_play.hpp"
#include "types/world.hpp"
#include "skills/goalie.hpp"
#include "skills/kick.hpp"
#include "robot_assignment.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{
OurKickoffPlay::OurKickoffPlay(
  visualization::OverlayPublisher & overlay_publisher,
  visualization::PlayInfoPublisher & play_info_publisher)
: BasePlay(overlay_publisher, play_info_publisher),
  line_kick_skill_(overlay_publisher),
  goalie_skill_(overlay_publisher, play_info_publisher)
{
}

void OurKickoffPlay::reset()
{
  goalie_skill_.reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> OurKickoffPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;
  std::vector<Robot> current_available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(current_available_robots, world);

  /* Get list of robots that can be kickers which excludes the last robot to touch the ball in the
   * kickoff
   */
  auto valid_kickers = current_available_robots;
  if (prev_id_ != -1) {
    play_helpers::removeRobotWithId(valid_kickers, prev_id_);
  }

  // get the closest valid kicker
  ateam_geometry::Point kicker_point = ateam_geometry::Point(-0.25, 0);
  const auto & maybe_kicker =
    world.our_robots.at(robot_assignment::assign(valid_kickers, {kicker_point}).begin()->first);

  if (maybe_kicker.has_value()) {
    const Robot & kicker = maybe_kicker.value();
    // handle kicker behavior and remove it from the total available
    play_helpers::removeRobotWithId(current_available_robots, kicker.id);

    // Go to starting point
    if (world.referee_info.running_command == ateam_common::GameCommand::PrepareKickoffOurs) {
      /* TODO(anon): consider if we can make this dynamic enough to reuse this for more than
       * just kickoff
       */
      auto viz_circle = ateam_geometry::makeCircle(kicker_point, kRobotRadius);
      overlay_publisher_.drawCircle(
        "destination_" + std::to_string(
          kicker.id), viz_circle, "blue", "transparent");


      auto & easy_move_to = easy_move_tos_.at(kicker.id);
      easy_move_to.setTargetPosition(kicker_point);
      easy_move_to.face_point(world.ball.pos);
      maybe_motion_commands.at(kicker.id) = easy_move_to.runFrame(kicker, world);

      play_info_publisher_.message["Our Kickoff Play"]["State"] = "Preparing";
      play_info_publisher_.message["Our Kickoff Play"]["Kicker Id"] = kicker.id;

      // Kick the ball
    } else if (world.referee_info.running_command == ateam_common::GameCommand::NormalStart) {
      line_kick_skill_.setTargetPoint(ateam_geometry::Point(world.field.field_length / 4.0, world.field.field_width/4));
      line_kick_skill_.setKickSpeed(2.5);
      maybe_motion_commands.at(kicker.id) = line_kick_skill_.runFrame(world, kicker);

      // we passed over the center line so we probably touched the ball
      if (kicker.pos.x() > 0.1) {
        prev_id_ = kicker.id;
      }

      play_info_publisher_.message["Our Kickoff Play"]["State"] = "Kicking";
    }
  }

  std::vector<ateam_geometry::Point> defender_positions = {
    ateam_geometry::Point(-0.3, world.field.field_width/4),
    ateam_geometry::Point(-0.3, -world.field.field_width/4),
    ateam_geometry::Point(-world.field.field_length/4, world.field.field_width*0.375),
    ateam_geometry::Point(-world.field.field_length/4, -world.field.field_width*0.375)
  };

  // handle all non-kicker robots
  const auto & robot_assignments = robot_assignment::assign(
    current_available_robots,
    defender_positions);

  for (const auto [robot_id, pos_ind] : robot_assignments) {
    const auto & maybe_assigned_robot = world.our_robots.at(robot_id);

    if (!maybe_assigned_robot) {
      // TODO(barulicm): log this?
      continue;
    }

    const Robot & robot = maybe_assigned_robot.value();

    auto & easy_move_to = easy_move_tos_.at(robot_id);

    const auto & target_position = defender_positions.at(pos_ind);

    auto viz_circle = ateam_geometry::makeCircle(target_position, kRobotRadius);
    overlay_publisher_.drawCircle(
      "destination_" + std::to_string(
        robot_id), viz_circle, "blue", "transparent");

    easy_move_to.setTargetPosition(target_position);
    easy_move_to.face_absolute(0);  // face away from our goal

    maybe_motion_commands.at(robot_id) = easy_move_to.runFrame(robot, world);
  }

  // Have the goalie do stuff
  goalie_skill_.runFrame(world, maybe_motion_commands);
  if (world.our_robots.at(world.referee_info.our_goalie_id).has_value()) {
    const Robot & goalie_robot = world.our_robots.at(world.referee_info.our_goalie_id).value();
    this->play_info_publisher_.message["Our Kickoff Play"]["robots"][world.referee_info.
      our_goalie_id]["pos"] = {goalie_robot.pos.x(), goalie_robot.pos.y()};
  }

  play_info_publisher_.send_play_message("our_kickoff_play");
  return maybe_motion_commands;
}
}  // namespace ateam_kenobi::plays
