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


#include "waypoints_play.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <tuple>
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

WaypointsPlay::WaypointsPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options)
{
  createIndexedChildren<play_helpers::EasyMoveTo>(easy_move_tos_, "EasyMoveTo");
  addWaypoint(
    5000, {
      {0.00, 0, M_PI},
      {0.30, 0, M_PI},
      {0.60, 0, M_PI},
      {0.90, 0, M_PI},
      {1.20, 0, M_PI},
      {1.50, 0, M_PI}
    });

  addWaypoint(
    5000, {
      {-0.20, -0.11, M_PI},
      {-0.20, 0.11, M_PI},
      {0.00, -0.22, M_PI},
      {0.00, 0.22, M_PI},
      {0.20, -0.33, M_PI},
      {0.20, 0.33, M_PI}
    });
}

void WaypointsPlay::reset()
{
  waypoint_index_ = 0;
  next_transition_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(
    waypoints_.front().duration_ms);
  path_planning::PlannerOptions planner_options;
  planner_options.use_default_obstacles = false;
  planner_options.footprint_inflation = 0.02;
  for (auto & emt : easy_move_tos_) {
    emt.reset();
    emt.setPlannerOptions(planner_options);
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> WaypointsPlay::runFrame(
  const World & world)
{
  const auto now = std::chrono::steady_clock::now();
  if (now >= next_transition_time_) {
    waypoint_index_++;
    waypoint_index_ %= waypoints_.size();
    next_transition_time_ = now +
      std::chrono::milliseconds(waypoints_[waypoint_index_].duration_ms);
    RCLCPP_INFO_STREAM(getLogger(), "waypoint index = " << waypoint_index_);
  }

  const auto & waypoint = waypoints_[waypoint_index_];

  auto available_robots = play_helpers::getAvailableRobots(world);

  const auto num_robots = std::min(waypoint.poses.size(), available_robots.size());

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  for (auto robot_ind = 0ul; robot_ind < num_robots; ++robot_ind) {
    const auto & pose = waypoint.poses[robot_ind];
    const auto & robot = available_robots[robot_ind];
    easy_move_tos_[robot.id].setTargetPosition(pose.position);
    easy_move_tos_[robot.id].face_absolute(pose.heading);
    motion_commands[robot.id] = easy_move_tos_[robot.id].runFrame(robot, world);
  }

  return motion_commands;
}

void WaypointsPlay::addWaypoint(
  const int64_t duration_ms, const std::vector<std::tuple<double,
  double, double>> & poses)
{
  std::vector<Pose> waypoint_poses;
  std::ranges::transform(
    poses, std::back_inserter(waypoint_poses), [](const auto & p) {
      return Pose{ateam_geometry::Point(std::get<0>(p), std::get<1>(p)), std::get<2>(p)};
    });
  waypoints_.emplace_back(waypoint_poses, duration_ms);
}

}  // namespace ateam_kenobi::plays
