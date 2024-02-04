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
#include "play_helpers/robot_assignment.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{
OurKickoffPlay::OurKickoffPlay()
: BasePlay("OurKickoffPlay"),
  line_kick_skill_(getOverlays().getChild("line_kick")),
  goalie_skill_(getOverlays().getChild("goalie"))
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

  goalie_skill_.runFrame(world, maybe_motion_commands);

  support_positions_ = {
    ateam_geometry::Point(-0.3, world.field.field_width / 4),
    ateam_geometry::Point(-0.3, -world.field.field_width / 4),
    ateam_geometry::Point(-world.field.field_length / 4, world.field.field_width * 0.375),
    ateam_geometry::Point(-world.field.field_length / 4, -world.field.field_width * 0.375)
  };

  std::vector<ateam_geometry::Point> assignment_points;
  assignment_points.push_back(kicker_point_);
  assignment_points.insert(
    assignment_points.end(), support_positions_.begin(),
    support_positions_.end());

  std::vector<std::vector<int>> disallowed_ids(assignment_points.size(), std::vector<int>{});
  if (world.double_touch_forbidden_id_) {
    disallowed_ids[0] = {*world.double_touch_forbidden_id_};
  }

  const auto assignments = play_helpers::assignRobots(
    current_available_robots, assignment_points,
    disallowed_ids);


  if (assignments[0].has_value()) {
    runKicker(world, *assignments[0], maybe_motion_commands);
  }

  std::vector<std::optional<Robot>> support_bots(assignments.begin() + 1, assignments.end());
  runSupportBots(world, support_bots, maybe_motion_commands);

  return maybe_motion_commands;
}

void OurKickoffPlay::runKicker(
  const World & world, const Robot & kicker,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & motion_commands)
{
  if (world.referee_info.running_command == ateam_common::GameCommand::PrepareKickoffOurs) {
    auto viz_circle = ateam_geometry::makeCircle(kicker_point_, kRobotRadius);
    getOverlays().drawCircle(
      "destination_" + std::to_string(
        kicker.id), viz_circle, "blue", "transparent");

    auto & easy_move_to = easy_move_tos_.at(kicker.id);
    easy_move_to.setTargetPosition(kicker_point_);
    easy_move_to.face_point(world.ball.pos);
    motion_commands.at(kicker.id) = easy_move_to.runFrame(kicker, world);

    getPlayInfo()["State"] = "Preparing";
    getPlayInfo()["Kicker Id"] = kicker.id;
  } else if (world.referee_info.running_command == ateam_common::GameCommand::NormalStart) {
    line_kick_skill_.setTargetPoint(
      ateam_geometry::Point(
        world.field.field_length / 4.0,
        world.field.field_width / 4));
    line_kick_skill_.setKickSpeed(0.4);
    motion_commands.at(kicker.id) = line_kick_skill_.runFrame(world, kicker);

    getPlayInfo()["State"] = "Kicking";
  }
}

void OurKickoffPlay::runSupportBots(
  const World & world, const std::vector<std::optional<Robot>> & support_bots,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & motion_commands)
{
  for(auto ind = 0ul; ind < support_bots.size(); ++ind) {
    const auto & maybe_robot = support_bots[ind];
    if(!maybe_robot) {
      continue;
    }
    const auto & robot = *maybe_robot;
    const auto & target_position = support_positions_[ind];

    auto & easy_move_to = easy_move_tos_.at(robot.id);

    auto viz_circle = ateam_geometry::makeCircle(target_position, kRobotRadius);
    getOverlays().drawCircle(
      "destination_" + std::to_string(
        robot.id), viz_circle, "blue", "transparent");

    easy_move_to.setTargetPosition(target_position);
    easy_move_to.face_absolute(0);  // face away from our goal

    motion_commands.at(robot.id) = easy_move_to.runFrame(robot, world);
  }
}

}  // namespace ateam_kenobi::plays
