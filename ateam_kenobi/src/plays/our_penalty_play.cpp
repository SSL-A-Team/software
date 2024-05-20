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


#include "our_penalty_play.hpp"
#include <limits>
#include "play_helpers/available_robots.hpp"
#include "play_helpers/window_evaluation.hpp"

namespace ateam_kenobi::plays
{

OurPenaltyPlay::OurPenaltyPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  goalie_skill_(createChild<skills::Goalie>("goalie")),
  line_kick_skill_(createChild<skills::LineKick>("line_kick")),
  move_tos_(createIndexedChildren<play_helpers::EasyMoveTo>("EasyMoveTo"))
{
}

double OurPenaltyPlay::getScore(const World & world)
{
  if (world.in_play) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto & cmd = world.referee_info.running_command;
  const auto & prev = world.referee_info.prev_command;
  if (cmd == ateam_common::GameCommand::PreparePenaltyOurs ||
    (cmd == ateam_common::GameCommand::NormalStart &&
    prev == ateam_common::GameCommand::PreparePenaltyOurs))
  {
    return std::numeric_limits<double>::max();
  }
  return std::numeric_limits<double>::quiet_NaN();
}

void OurPenaltyPlay::reset()
{
  for (auto & move_to : move_tos_) {
    move_to.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> OurPenaltyPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  goalie_skill_.runFrame(world, motion_commands);

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  if (available_robots.empty()) {
    return {};
  }

  auto kicking_robot = available_robots.front();
  available_robots.erase(available_robots.begin());

  getPlayInfo()["Kicker ID"] = kicking_robot.id;

  if (world.referee_info.running_command == ateam_common::GameCommand::NormalStart) {
    if (kick_time_ == std::chrono::steady_clock::time_point::max()) {
      kick_time_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    }
  } else {
    kick_time_ = std::chrono::steady_clock::time_point::max();
  }

  if (kick_time_ == std::chrono::steady_clock::time_point::max() ||
    kick_time_ > std::chrono::steady_clock::now())
  {
    // Stage for kick
    getPlayInfo()["State"] = "Preparing";
    line_kick_skill_.setTargetPoint(chooseKickTarget(world));
    const auto destination = line_kick_skill_.getAssignmentPoint(world);
    auto & move_to = move_tos_[kicking_robot.id];
    move_to.setTargetPosition(destination);
    move_to.face_point(world.ball.pos);
    move_to.setMaxVelocity(1.5);
    motion_commands[kicking_robot.id] = move_to.runFrame(kicking_robot, world);
  } else {
    // Kick ball
    getPlayInfo()["State"] = "Kicking";
    motion_commands[kicking_robot.id] = line_kick_skill_.runFrame(world, kicking_robot);
  }

  ateam_geometry::Point pattern_point(kRobotDiameter - (world.field.field_length / 2.0),
    kRobotDiameter - (world.field.field_width / 2.0));
  ateam_geometry::Vector pattern_step(kRobotDiameter + 0.2, 0.0);
  for (const auto & robot : available_robots) {
    auto & move_to = move_tos_[robot.id];
    move_to.setTargetPosition(pattern_point);
    move_to.setMaxVelocity(1.5);
    move_to.face_travel();
    motion_commands[robot.id] = move_to.runFrame(robot, world);
    pattern_point += pattern_step;
  }

  return motion_commands;
}


ateam_geometry::Point OurPenaltyPlay::chooseKickTarget(const World & world)
{
  const ateam_geometry::Segment goal_segment(ateam_geometry::Point(
      world.field.field_length / 2,
      -world.field.goal_width / 2),
    ateam_geometry::Point(world.field.field_length / 2, world.field.goal_width / 2));
  const auto their_robots = play_helpers::getVisibleRobots(world.their_robots);
  const auto windows = play_helpers::window_evaluation::getWindows(
    goal_segment, world.ball.pos, their_robots);
  play_helpers::window_evaluation::drawWindows(
    windows, world.ball.pos, getOverlays().getChild("Windows"));
  const auto target_window = play_helpers::window_evaluation::getLargestWindow(windows);
  if (target_window) {
    return CGAL::midpoint(*target_window);
  } else {
    return ateam_geometry::Point(world.field.field_length / 2, 0);
  }
}
}  // namespace ateam_kenobi::plays
