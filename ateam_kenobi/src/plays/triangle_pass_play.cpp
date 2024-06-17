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
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include "play_helpers/available_robots.hpp"
#include "play_helpers/robot_assignment.hpp"

namespace ateam_kenobi::plays
{

TrianglePassPlay::TrianglePassPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  line_kick_(createChild<skills::LineKick>("line_kick")),
  pass_receiver_(createChild<skills::PassReceiver>("pass_receiver")),
  easy_move_tos_(createIndexedChildren<play_helpers::EasyMoveTo>("EasyMoveTo"))
{
  positions.emplace_back(0.85, 0);
  const auto angle = angles::from_degrees(120);
  CGAL::Aff_transformation_2<ateam_geometry::Kernel> rotate_transform(CGAL::ROTATION, std::sin(
      angle), std::cos(angle));
  positions.push_back(positions.back().transform(rotate_transform));
  positions.push_back(positions.back().transform(rotate_transform));
  for (auto & p : positions) {
    p += ateam_geometry::Vector(0, 2);
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

  const auto ball_speed = ateam_geometry::norm(world.ball.vel);

  ball_vel_avg_ = (0.8 * ball_vel_avg_) + (0.2 * ball_speed);

  auto byDistToBall = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.ball.pos, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto closest_robot_iter = std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);
  const auto & closest_robot = *closest_robot_iter;
  const auto dist_to_ball =
    CGAL::approximate_sqrt(CGAL::squared_distance(world.ball.pos, closest_robot.pos));

  switch (state_) {
    case State::Setup:
      if (isReady(world)) {
        state_ = State::Kicking;
        pass_receiver_.reset();
      }
      runSetup(world, maybe_motion_commands);
      getPlayInfo()["State"] = "Setup";
      break;
    case State::Kicking:
      if (ball_speed > 0.5 * kKickSpeed) {
        state_ = State::Receiving;
        latch_receive_ = false;
      }
      runKicking(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Kicking";
      break;
    case State::Receiving:
      if (ball_vel_avg_ < 0.1) {
        state_ = State::BackOff;
      }
      getPlayInfo()["State"] = "Receiving";
      break;
    case State::BackOff:
      if (dist_to_ball > 0.3) {
        state_ = State::Kicking;
        kick_target_ind_++;
        kick_target_ind_ %= positions.size();
        pass_receiver_.reset();
      }
      runBackOff(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "BackOff";
      break;
  }

  getOverlays().drawCircle("target", ateam_geometry::makeCircle(positions[kick_target_ind_], kRobotRadius+0.1), "orange");

  if(state_ != State::Setup && state_ != State::BackOff) {
    pass_receiver_.setTarget(positions[kick_target_ind_]);
    const auto receiver_command_msg = pass_receiver_.runFrame(world);
    const auto receiver_robot_id = pass_receiver_.getRobotID();
    if (receiver_robot_id) {
      maybe_motion_commands[*receiver_robot_id] = receiver_command_msg;
      getPlayInfo()["Receiver"] = *receiver_robot_id;
    }
  }

  for (auto ind = 0ul; ind < available_robots.size(); ++ind) {
    const auto & robot = available_robots[ind];
    auto & motion_command = maybe_motion_commands[robot.id];

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

  return std::ranges::all_of(distances, [](const double dist){ return dist < kRobotRadius; });
}

void TrianglePassPlay::runSetup(
  const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  const auto available_robots = play_helpers::getAvailableRobots(world);
  const auto assignments = play_helpers::assignRobots(available_robots, positions);
  for(auto pos_ind = 0ul; pos_ind < positions.size(); ++pos_ind) {
    const auto & position = positions[pos_ind];
    const auto & maybe_bot = assignments.at(pos_ind);
    if(!maybe_bot) {
      continue;
    }
    auto & emt = easy_move_tos_[maybe_bot->id];
    emt.setTargetPosition(position);
    emt.face_travel();
    motion_commands[maybe_bot->id] = emt.runFrame(*maybe_bot, world);
  }
}

void TrianglePassPlay::runKicking(
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

  const auto & closest_robot = *closest_robot_iter;

  last_kicked_id_ = closest_robot.id;

  getPlayInfo()["Robots"][std::to_string(closest_robot.id)] = "Kicker";

  line_kick_.setTargetPoint(positions[kick_target_ind_]);
  line_kick_.setKickSpeed(kKickSpeed);
  auto & motion_command = motion_commands[closest_robot.id];
  motion_command = line_kick_.runFrame(world, closest_robot);
  // motion_command->dribbler_speed = 200;
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
  getPlayInfo()["Robots"][std::to_string(receiver_robot.id)] = "Receiver";

  const auto ball_to_bot_vec = receiver_robot.pos - world.ball.pos;
  const auto vel = ateam_geometry::normalize(ball_to_bot_vec) * 0.25;
  auto & motion_command = motion_commands[receiver_robot.id] =
    ateam_msgs::msg::RobotMotionCommand{};
  motion_command->twist.linear.x = vel.x();
  motion_command->twist.linear.y = vel.y();
}

}  // namespace ateam_kenobi::plays
