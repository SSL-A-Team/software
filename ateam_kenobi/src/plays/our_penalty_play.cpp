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
#include <vector>
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/window_evaluation.hpp"

namespace ateam_kenobi::plays
{

OurPenaltyPlay::OurPenaltyPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  goalie_skill_(createChild<skills::Goalie>("goalie")),
  kick_skill_(createChild<skills::UniversalKick>("kick")),
  multi_move_to_(createChild<tactics::MultiMoveTo>("multi_move_to"))
{
}

stp::PlayScore OurPenaltyPlay::getScore(const World & world)
{
  // if (world.in_play) {
  //   return stp::PlayScore::NaN();
  // }
  const auto & cmd = world.referee_info.running_command;
  const auto & prev = world.referee_info.prev_command;
  if (cmd == ateam_common::GameCommand::PreparePenaltyOurs ||
    (cmd == ateam_common::GameCommand::NormalStart &&
    prev == ateam_common::GameCommand::PreparePenaltyOurs))
  {
    return stp::PlayScore::Max();
  }
  return stp::PlayScore::NaN();
}

std::array<std::optional<RobotCommand>, 16> OurPenaltyPlay::runFrame(
  const World & world)
{
  std::array<std::optional<RobotCommand>, 16> motion_commands;

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
    kick_skill_.SetTargetPoint(chooseKickTarget(world));
    const auto destination = kick_skill_.GetAssignmentPoint(world);
    RobotCommand command;
    command.motion_intent.linear = motion::intents::linear::PositionIntent{destination};
    command.motion_intent.angular = motion::intents::angular::FacingIntent{world.ball.pos};
    command.motion_intent.motion_options.max_velocity = 1.5;
    motion_commands[kicking_robot.id] = command;
  } else {
    // Kick ball
    getPlayInfo()["State"] = "Kicking";
    if (kick_skill_.IsDone()) {
      kick_skill_.Reset();
    }
    motion_commands[kicking_robot.id] = kick_skill_.RunFrame(world, kicking_robot);
  }

  ateam_geometry::Point pattern_point(kRobotDiameter - (world.field.field_length / 2.0),
    kRobotDiameter - (world.field.field_width / 2.0));
  ateam_geometry::Vector pattern_step(kRobotDiameter + 0.2, 0.0);
  std::vector<ateam_geometry::Point> target_points;
  std::generate_n(std::back_inserter(target_points), available_robots.size(),
    [pos = pattern_point, step = pattern_step, &world]() mutable {
      auto current = pos;
      pos = pos + step;
      if (pos.x() > (world.field.field_length / 2.0) - kRobotDiameter) {
        pos = ateam_geometry::Point(
          kRobotDiameter - (world.field.field_length / 2.0),
          pos.y() + step.y());
      }
      return current;
    });
  multi_move_to_.SetTargetPoints(target_points);
  multi_move_to_.SetFaceTravel();
  for(auto & maybe_cmd : motion_commands) {
    if(!maybe_cmd) {continue;}
    maybe_cmd->motion_intent.motion_options.max_velocity = 1.5;
  }
  multi_move_to_.RunFrame(available_robots, motion_commands);

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
