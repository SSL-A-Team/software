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
  first_frame_ = true;
  pass_tactic_.reset();
}

std::array<std::optional<RobotCommand>, 16> TestPassPlay::runFrame(
  const World & world)
{
  std::array<std::optional<RobotCommand>, 16> maybe_motion_commands;

  if(first_frame_) {
    const auto furthest_target_iter = std::max_element(targets_.begin(), targets_.end(),
        [&world](const auto & t1, const auto & t2) {
          return CGAL::squared_distance(world.ball.pos, t1) < CGAL::squared_distance(world.ball.pos,
          t2);
    });
    target_ind_ = std::distance(targets_.begin(), furthest_target_iter);
    first_frame_ = false;
  }

  const auto is_done = pass_tactic_.isDone();
  if(is_done && !prev_done_) {
    done_time_ = world.current_time;
  }
  const auto done_time = std::chrono::duration_cast<std::chrono::seconds>(world.current_time -
      done_time_).count();

  if (is_done && done_time > 5) {
    do {
      target_ind_ = (target_ind_ + 1) % targets_.size();
      pass_tactic_.reset();
    } while (ateam_geometry::norm(targets_[target_ind_] - world.ball.pos) < 0.5);
  }
  prev_done_ = is_done;

  pass_tactic_.setTarget(targets_[target_ind_]);

  play_helpers::GroupAssignmentSet groups;

  const auto kicker_assignment_point = pass_tactic_.getKickerAssignmentPoint(world);
  groups.AddPosition("kicker", kicker_assignment_point);
  getOverlays().drawCircle("kick_assignment_pt",
      ateam_geometry::makeCircle(kicker_assignment_point, 0.05), "#00000000", "LightGreen");

  const auto receiver_assignment_point = pass_tactic_.getReceiverAssignmentPoint();
  groups.AddPosition("receiver", receiver_assignment_point);
  getOverlays().drawCircle("receive_assignment_pt",
      ateam_geometry::makeCircle(receiver_assignment_point, 0.05), "#00000000", "red");

  const auto available_robots = play_helpers::getAvailableRobots(world);
  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  const auto kicker_assignment = assignments.GetPositionAssignment("kicker");
  const auto receiver_assignment = assignments.GetPositionAssignment("receiver");

  if(!kicker_assignment || !receiver_assignment) {
    return maybe_motion_commands;
  }

  const auto kicker = *kicker_assignment;
  const auto receiver = *receiver_assignment;

  getOverlays().drawCircle("kicker_halo",
      ateam_geometry::makeCircle(kicker.pos, kRobotRadius + 0.1), "LightGreen", "#00000000");
  getOverlays().drawCircle("receiver_halo",
      ateam_geometry::makeCircle(receiver.pos, kRobotRadius + 0.1), "red", "#00000000");

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
