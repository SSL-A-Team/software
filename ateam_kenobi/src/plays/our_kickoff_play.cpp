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
#include <limits>
#include "types/world.hpp"
#include "skills/goalie.hpp"
#include "play_helpers/robot_assignment.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{
OurKickoffPlay::OurKickoffPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  line_kick_skill_(createChild<skills::LineKick>("line_kick")),
  defense_(createChild<tactics::StandardDefense>("defense"))
{
}

double OurKickoffPlay::getScore(const World & world)
{
  if (world.in_play) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto & cmd = world.referee_info.running_command;
  const auto & prev = world.referee_info.prev_command;
  if (cmd == ateam_common::GameCommand::PrepareKickoffOurs ||
    (cmd == ateam_common::GameCommand::NormalStart &&
    prev == ateam_common::GameCommand::PrepareKickoffOurs))
  {
    return std::numeric_limits<double>::max();
  }
  return std::numeric_limits<double>::quiet_NaN();
}

void OurKickoffPlay::reset()
{
  line_kick_skill_.Reset();
  defense_.reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> OurKickoffPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;
  std::vector<Robot> current_available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(current_available_robots, world);

  support_positions_ = {
    ateam_geometry::Point(-0.3, world.field.field_width / 3),
    ateam_geometry::Point(-0.3, -world.field.field_width / 3)
  };

  play_helpers::GroupAssignmentSet groups;

  std::vector<int> disallowed_kickers;
  if (world.double_touch_forbidden_id_) {
    disallowed_kickers.push_back(*world.double_touch_forbidden_id_);
  }
  groups.AddPosition("kicker", kicker_point_, disallowed_kickers);

  groups.AddGroup("defenders", defense_.getAssignmentPoints(world));

  groups.AddGroup("supports", support_positions_);

  const auto assignments = play_helpers::assignGroups(current_available_robots, groups);

  const auto maybe_kicker = assignments.GetPositionAssignment("kicker");
  if (maybe_kicker) {
    runKicker(world, *maybe_kicker, maybe_motion_commands);
  }

  defense_.runFrame(
    world, assignments.GetGroupFilledAssignments("defenders"),
    maybe_motion_commands);

  runSupportBots(world, assignments.GetGroupAssignments("supports"), maybe_motion_commands);

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
    line_kick_skill_.SetTargetPoint(
      ateam_geometry::Point(-0.3, world.field.field_width / 3));
    line_kick_skill_.SetKickSpeed(3.0);
    motion_commands.at(kicker.id) = line_kick_skill_.RunFrame(world, kicker);

    getPlayInfo()["State"] = "Kicking";
  }
}

void OurKickoffPlay::runSupportBots(
  const World & world, const std::vector<std::optional<Robot>> & support_bots,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & motion_commands)
{
  for (auto ind = 0ul; ind < support_bots.size(); ++ind) {
    const auto & maybe_robot = support_bots[ind];
    if (!maybe_robot) {
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
    easy_move_to.face_point(world.ball.pos);

    motion_commands.at(robot.id) = easy_move_to.runFrame(robot, world);
  }
}

}  // namespace ateam_kenobi::plays
