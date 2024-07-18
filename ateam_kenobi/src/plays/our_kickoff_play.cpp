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
#include <random>
#include "types/world.hpp"
#include "skills/goalie.hpp"
#include "play_helpers/robot_assignment.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{
OurKickoffPlay::OurKickoffPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  defense_(createChild<tactics::StandardDefense>("defense")),
  multi_move_to_(createChild<tactics::MultiMoveTo>("multi_move_To")),
  pass_(createChild<tactics::Pass>("pass"))
{
}

stp::PlayScore OurKickoffPlay::getScore(const World & world)
{
  const auto & cmd = world.referee_info.running_command;
  const auto & prev = world.referee_info.prev_command;

  stp::PlayScore score;
  if (cmd == ateam_common::GameCommand::NormalStart) {
    if (prev_frame_game_command_ == ateam_common::GameCommand::PrepareKickoffOurs) {
      // First frame of normal start, I should be considered
      score = 75.0;
    } else if (prev == ateam_common::GameCommand::PrepareKickoffOurs) {
      // already running normal start
      if (pass_.isDone()) {
        score = stp::PlayScore::Min();
      } else {
        // arbitrary value to compare against KickoffOnGoalPlay
        score = 75.0;
      }
    }
  } else {
    score = stp::PlayScore::NaN();
  }

  prev_frame_game_command_ = cmd;

  return score;
}

stp::PlayCompletionState OurKickoffPlay::getCompletionState()
{
  if (pass_.isDone()) {
    return stp::PlayCompletionState::Done;
  }
  return stp::PlayCompletionState::Busy;
}

void OurKickoffPlay::reset()
{
  defense_.reset();
  multi_move_to_.Reset();
  pass_.reset();
  pass_direction_chosen_ = false;
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> OurKickoffPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;
  std::vector<Robot> current_available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(current_available_robots, world);

  if (!pass_direction_chosen_) {
    static std::default_random_engine rand_eng(std::random_device{}());
    std::uniform_int_distribution<int> distribution(0, 1);
    pass_left_ = distribution(rand_eng);
    pass_direction_chosen_ = true;
  }

  multi_move_to_.SetTargetPoints(
    {
      ateam_geometry::Point(-0.3, (pass_left_ ? -1.0 : 1.0) * world.field.field_width / 3)
    });
  multi_move_to_.SetFacePoint(world.ball.pos);

  pass_.setTarget(
    ateam_geometry::Point(
      -0.3,
      (pass_left_ ? 1.0 : -1.0) * world.field.field_width / 3));

  play_helpers::GroupAssignmentSet groups;

  std::vector<int> disallowed_kickers;
  if (world.double_touch_forbidden_id_) {
    disallowed_kickers.push_back(*world.double_touch_forbidden_id_);
  }
  groups.AddPosition("kicker", pass_.getKickerAssignmentPoint(world), disallowed_kickers);
  groups.AddPosition("receiver", pass_.getReceiverAssignmentPoint());

  groups.AddGroup("defenders", defense_.getAssignmentPoints(world));

  groups.AddGroup("supports", multi_move_to_.GetAssignmentPoints());

  const auto assignments = play_helpers::assignGroups(current_available_robots, groups);

  const auto maybe_kicker = assignments.GetPositionAssignment("kicker");
  const auto maybe_receiver = assignments.GetPositionAssignment("receiver");
  if (maybe_kicker && maybe_receiver) {
    const auto & kicker = *maybe_kicker;
    const auto & receiver = *maybe_receiver;
    auto & kicker_command =
      *(maybe_motion_commands[kicker.id] = ateam_msgs::msg::RobotMotionCommand{});
    auto & receiver_command =
      *(maybe_motion_commands[receiver.id] = ateam_msgs::msg::RobotMotionCommand{});
    pass_.runFrame(world, kicker, receiver, kicker_command, receiver_command);
  }

  defense_.runFrame(
    world, assignments.GetGroupFilledAssignments("defenders"),
    maybe_motion_commands);

  multi_move_to_.RunFrame(
    world, assignments.GetGroupAssignments("supports"),
    maybe_motion_commands);

  return maybe_motion_commands;
}

}  // namespace ateam_kenobi::plays
