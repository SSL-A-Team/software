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

namespace ateam_kenobi::plays
{

TrianglePassPlay::TrianglePassPlay(stp::Options stp_options)
: stp::Play("TrianglePassPlay", stp_options),
  line_kick_(createChild<skills::LineKick>("line_kick"))
{
  play_helpers::EasyMoveTo::CreateArray(easy_move_tos_, getOverlays().getChild("EasyMoveTo"));

  positions.emplace_back(0.85, 0);
  const auto angle = angles::from_degrees(120);
  CGAL::Aff_transformation_2<ateam_geometry::Kernel> rotate_transform(CGAL::ROTATION, std::sin(
      angle), std::cos(angle));
  positions.push_back(positions.back().transform(rotate_transform));
  positions.push_back(positions.back().transform(rotate_transform));
}

void TrianglePassPlay::reset()
{
  for (auto & e : easy_move_tos_) {
    e.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> TrianglePassPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);

  if (available_robots.size() < 2) {
    return maybe_motion_commands;
  }

  if (available_robots.size() > 3) {
    available_robots.erase(available_robots.begin() + 3, available_robots.end());
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
    case State::Kicking:
      if (ball_speed > 0.2) {
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
      runReceiving(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Receiving";
      break;
    case State::BackOff:
      if (dist_to_ball > 0.3) {
        state_ = State::Kicking;
      }
      runBackOff(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "BackOff";
      break;
  }

  for (auto ind = 0ul; ind < available_robots.size(); ++ind) {
    const auto & robot = available_robots[ind];
    const auto & position = positions[ind];
    auto & motion_command = maybe_motion_commands[robot.id];

    if (!motion_command) {
      auto & emt = easy_move_tos_[robot.id];
      emt.setTargetPosition(position);
      emt.face_point(world.ball.pos);
      maybe_motion_commands[robot.id] = emt.runFrame(robot, world);
      getPlayInfo()["Robots"][std::to_string(robot.id)] = "-";
    }
  }

  return maybe_motion_commands;
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
  auto receiver_robot_iter = closest_robot_iter + 1;
  if (receiver_robot_iter >= available_robots.end()) {
    receiver_robot_iter = available_robots.begin();
  }

  const auto & closest_robot = *closest_robot_iter;
  const auto & receiver_robot = *receiver_robot_iter;

  last_kicked_id_ = closest_robot.id;

  getPlayInfo()["Robots"][std::to_string(closest_robot.id)] = "Kicker";
  getPlayInfo()["Robots"][std::to_string(receiver_robot.id)] = "Receiver";

  line_kick_.setTargetPoint(receiver_robot.pos);
  line_kick_.setKickSpeed(0.55);
  auto & motion_command = motion_commands[closest_robot.id];
  motion_command = line_kick_.runFrame(world, closest_robot);
  // motion_command->dribbler_speed = 200;
}

void TrianglePassPlay::runReceiving(
  std::vector<Robot> available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  const ateam_geometry::Ray ball_ray(world.ball.pos, world.ball.vel);

  play_helpers::removeRobotWithId(available_robots, last_kicked_id_);

  const auto ball_speed = ateam_geometry::norm(world.ball.vel);

  auto byDistToBallRay = [&ball_ray](const Robot & lhs, const Robot & rhs) {
      return CGAL::squared_distance(lhs.pos, ball_ray) <
             CGAL::squared_distance(rhs.pos, ball_ray);
    };

  auto byDistToBall = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.ball.pos, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  std::function<bool(const Robot &, const Robot &)> compFunc = byDistToBallRay;
  if (ball_speed < 0.2) {
    compFunc = byDistToBall;
  }

  const auto & receiver_robot = *std::min_element(
    available_robots.begin(),
    available_robots.end(), compFunc);

  getPlayInfo()["Robots"][std::to_string(receiver_robot.id)] = "Receiver";

  const auto target_point = ball_ray.supporting_line().projection(receiver_robot.pos);

  getOverlays().drawCircle("receiving_pos", ateam_geometry::makeCircle(target_point, kRobotRadius));

  auto & emt = easy_move_tos_[receiver_robot.id];
  emt.setTargetPosition(target_point);
  emt.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.use_default_obstacles = false;
  emt.setPlannerOptions(planner_options);
  auto & motion_command = motion_commands[receiver_robot.id];
  motion_command = emt.runFrame(receiver_robot, world);
  motion_command->dribbler_speed = 200;
  if (CGAL::approximate_sqrt(CGAL::squared_distance(receiver_robot.pos, world.ball.pos)) < 0.5) {
    ateam_geometry::Vector robot_vel(motion_command->twist.linear.x,
      motion_command->twist.linear.y);
    robot_vel += ateam_geometry::normalize(world.ball.vel) * 0.35;
    motion_command->twist.linear.x = robot_vel.x();
    motion_command->twist.linear.y = robot_vel.y();
  }
  const auto ball_close =
    CGAL::approximate_sqrt(CGAL::squared_distance(receiver_robot.pos, world.ball.pos)) < 0.11;
  if (latch_receive_ || ball_close) {
    motion_command->twist.linear.x = 0;
    motion_command->twist.linear.y = 0;
    latch_receive_ = true;
  }
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
