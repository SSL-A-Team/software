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


#include "triangle_pass_play.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"

namespace ateam_kenobi::plays
{

TrianglePassPlay::TrianglePassPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  pass_tactic_(createChild<tactics::Pass>("pass"))
{
  createIndexedChildren<play_helpers::EasyMoveTo>(easy_move_tos_, "EasyMoveTo");
  positions.emplace_back(0.85, 0);
  const auto angle = angles::from_degrees(120);
  CGAL::Aff_transformation_2<ateam_geometry::Kernel> rotate_transform(CGAL::ROTATION, std::sin(
      angle), std::cos(angle));
  positions.push_back(positions.back().transform(rotate_transform));
  positions.push_back(positions.back().transform(rotate_transform));
  for (auto & p : positions) {
    p += ateam_geometry::Vector(0, 0);
  }
}

void TrianglePassPlay::reset()
{
  state_ = State::Setup;
  kick_target_ind_ = 0;
  for (auto & e : easy_move_tos_) {
    e.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> TrianglePassPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);

  if (available_robots.size() < 3) {
    return maybe_motion_commands;
  }

  if (available_robots.size() > 3) {
    available_robots.erase(available_robots.begin() + 3, available_robots.end());
  }

  switch (state_) {
    case State::Setup:
      runSetup(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Setup";
      break;
    case State::Passing:
      runPassing(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Passing";
      break;
    case State::BackOff:
      runBackOff(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "BackOff";
      break;
  }

  getOverlays().drawCircle(
    "target",
    ateam_geometry::makeCircle(positions[kick_target_ind_], kRobotRadius + 0.1), "orange");

  for (auto ind = 0ul; ind < maybe_motion_commands.size(); ++ind) {
    auto & motion_command = maybe_motion_commands[ind];
    if (!motion_command) {
      motion_command = ateam_msgs::msg::RobotMotionCommand{};
    }
  }

  return maybe_motion_commands;
}

bool TrianglePassPlay::isReady(const World & world)
{
  const auto available_robots = play_helpers::getAvailableRobots(world);
  auto dist_to_closest_bot = [&available_robots](const ateam_geometry::Point p) {
      return ateam_geometry::norm(play_helpers::getClosestRobot(available_robots, p).pos - p);
    };

  std::vector<double> distances;
  std::ranges::transform(positions, std::back_inserter(distances), dist_to_closest_bot);

  return std::ranges::all_of(distances, [](const double dist) {return dist < kRobotRadius;});
}

void TrianglePassPlay::runSetup(
  const std::vector<Robot> & robots,
  const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  if (isReady(world)) {
    state_ = State::Passing;
    pass_tactic_.reset();
  }

  for (auto pos_ind = 0ul; pos_ind < positions.size(); ++pos_ind) {
    const auto & position = positions[pos_ind];
    const auto & robot = robots[pos_ind];
    auto & emt = easy_move_tos_[robot.id];
    emt.setTargetPosition(position);
    emt.face_travel();
    motion_commands[robot.id] = emt.runFrame(robot, world);
  }
}

void TrianglePassPlay::runPassing(
  const std::vector<Robot> & robots,
  const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  if (pass_tactic_.isDone()) {
    state_ = State::BackOff;
  }

  pass_tactic_.setTarget(positions[kick_target_ind_]);

  const auto & receiver_bot = robots[kick_target_ind_];

  const auto idler_index = (kick_target_ind_ + 1) % robots.size();
  const auto & idler_bot = robots[idler_index];

  const auto & kicker_bot = robots[(kick_target_ind_ + 2) % robots.size()];

  getPlayInfo()["Kicker"] = std::to_string(kicker_bot.id);
  getPlayInfo()["Receiver"] = std::to_string(receiver_bot.id);

  auto & kicker_command = *(motion_commands[kicker_bot.id] = ateam_msgs::msg::RobotMotionCommand{});
  auto & receiver_command =
    *(motion_commands[receiver_bot.id] = ateam_msgs::msg::RobotMotionCommand{});

  pass_tactic_.runFrame(world, kicker_bot, receiver_bot, kicker_command, receiver_command);

  auto & idler_emt = easy_move_tos_[idler_bot.id];
  idler_emt.setTargetPosition(positions[idler_index]);
  idler_emt.face_point(world.ball.pos);
  motion_commands[idler_bot.id] = idler_emt.runFrame(idler_bot, world);
}

void TrianglePassPlay::runBackOff(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  auto byDistToBall = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.ball.pos, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto closest_robot_iter = std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);
  const auto & receiver_robot = *closest_robot_iter;
  getPlayInfo()["Receiver"] = std::to_string(receiver_robot.id);

  const auto ball_to_bot_vec = receiver_robot.pos - world.ball.pos;

  const auto dist_to_ball = ateam_geometry::norm(ball_to_bot_vec);

  if (dist_to_ball > 0.3) {
    state_ = State::Passing;
    kick_target_ind_++;
    kick_target_ind_ %= positions.size();
    pass_tactic_.reset();
  }

  const auto vel = ateam_geometry::normalize(ball_to_bot_vec) * 0.25;
  auto & motion_command = motion_commands[receiver_robot.id] =
    ateam_msgs::msg::RobotMotionCommand{};
  motion_command->twist.linear.x = vel.x();
  motion_command->twist.linear.y = vel.y();
}

}  // namespace ateam_kenobi::plays
