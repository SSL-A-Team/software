// Copyright 2025 A Team
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

#include "test_pass_play.hpp"
#include "core/types/world.hpp"
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"

namespace ateam_kenobi::plays
{
TestPassPlay::TestPassPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  pass_tactic_(createChild<tactics::Pass>("pass"))
{
  targets_ = {
    ateam_geometry::Point{-1, -1.5},
    ateam_geometry::Point{-1, 1.5}
  };
}

void TestPassPlay::enter()
{
  target_ind_ = 0;
  pass_tactic_.reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> TestPassPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  const auto is_done = pass_tactic_.isDone();
  if(is_done && !prev_done_) {
    done_time_ = world.current_time;
  }
  const auto done_time = std::chrono::duration_cast<std::chrono::seconds>(world.current_time -
      done_time_).count();
  if (is_done && done_time > 5) {
    target_ind_ = (target_ind_ + 1) % targets_.size();
    pass_tactic_.reset();
  }
  prev_done_ = is_done;

  pass_tactic_.setTarget(targets_[target_ind_]);

  play_helpers::GroupAssignmentSet groups;
  groups.AddPosition("kicker", pass_tactic_.getKickerAssignmentPoint(world));
  groups.AddPosition("receiver", pass_tactic_.getReceiverAssignmentPoint());

  const auto available_robots = play_helpers::getAvailableRobots(world);
  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  const auto kicker_assignment = assignments.GetPositionAssignment("kicker");
  const auto receiver_assignment = assignments.GetPositionAssignment("receiver");

  if(!kicker_assignment || !receiver_assignment) {
    return maybe_motion_commands;
  }

  const auto kicker = *kicker_assignment;
  const auto receiver = *receiver_assignment;

  getPlayInfo()["kicker_id"] = kicker.id;
  getPlayInfo()["receiver_id"] = receiver.id;
  getPlayInfo()["pass_done"] = is_done;
  getPlayInfo()["done_time"] = done_time;
  getPlayInfo()["target"]["x"] = targets_[target_ind_].x();
  getPlayInfo()["target"]["y"] = targets_[target_ind_].y();
  ForwardPlayInfo(pass_tactic_);

  auto & kicker_command = maybe_motion_commands.at(kicker.id).emplace();
  auto & receiver_command = maybe_motion_commands.at(receiver.id).emplace();
  if(!is_done) {
    pass_tactic_.runFrame(world, kicker, receiver, kicker_command, receiver_command);
  }

  return maybe_motion_commands;
}
}  // namespace ateam_kenobi::plays
