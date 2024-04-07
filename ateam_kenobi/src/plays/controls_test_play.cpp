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

#include "controls_test_play.hpp"
#include <angles/angles.h>
#include <ateam_geometry/types.hpp>
#include <ateam_geometry/make_circle.hpp>
#include "types/world.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{
ControlsTestPlay::ControlsTestPlay()
: BasePlay("ControlsTestPlay")
{
  play_helpers::EasyMoveTo::CreateArray(easy_move_tos_, overlays_.getChild("EasyMoveTo"));

  // Turn 180 deg in place
  // waypoints.push_back({ateam_geometry::Point(-1.5,0.0), AngleMode::face_absolute,  0.0, 3.0});
  // waypoints.push_back({ateam_geometry::Point(-1.5,0.0), AngleMode::face_absolute, M_PI, 3.0});

  // Drive in square
  waypoints = {
    {ateam_geometry::Point(-1.5, -1.0), AngleMode::face_absolute, 0.0, 3.0},
    {ateam_geometry::Point(-1.5, 1.0), AngleMode::face_absolute, 0.0, 3.0},
    {ateam_geometry::Point(0.0, 1.0), AngleMode::face_absolute, 0.0, 3.0},
    {ateam_geometry::Point(0.0, -1.0), AngleMode::face_absolute, 0.0, 3.0},
  };

  motion_controller_.v_max = 2;
  motion_controller_.t_max = 18;
}

void ControlsTestPlay::reset()
{
  motion_controller_.reset();
  goal_hit = false;
  goal_hit_time = std::chrono::steady_clock::now();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> ControlsTestPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;
  auto current_available_robots = play_helpers::getAvailableRobots(world);
  if (current_available_robots.empty()) {
    // No robots available to run
    return maybe_motion_commands;
  }

  const auto & robot = current_available_robots.front();

  if (goal_hit) {
    if ((std::chrono::steady_clock::now() - goal_hit_time) >
      std::chrono::duration<double>(waypoints[index].hold_time_sec))
    {
      index = (index + 1) % waypoints.size();
      goal_hit = false;
    }
  } else if (isGoalHit(robot)) {
    goal_hit = true;
    goal_hit_time = std::chrono::steady_clock::now();
  }


  motion_controller_.set_trajectory(std::vector<ateam_geometry::Point> {waypoints[index].position});
  switch (waypoints[index].angle_mode) {
    case AngleMode::face_absolute:
      motion_controller_.face_absolute(waypoints[index].heading);
      break;
    case AngleMode::face_travel:
      motion_controller_.face_travel();
      break;
    case AngleMode::face_point:
      // WARNING: face_point not supported in this play. Defaulting to no face.
      [[fallthrough]];
    case AngleMode::no_face:
      motion_controller_.no_face();
      break;
  }
  const auto current_time = std::chrono::duration_cast<std::chrono::duration<double>>(
    world.current_time.time_since_epoch()).count();
  maybe_motion_commands[robot.id] = motion_controller_.get_command(robot, current_time);

  const std::vector<ateam_geometry::Point> viz_path = {robot.pos, waypoints[index].position};
  overlays_.drawLine("controls_test_path", viz_path, "purple");

  play_info_["robot"]["id"] = robot.id;
  play_info_["robot"]["index"] = index;
  play_info_["robot"]["goal_hit"] = goal_hit;
  play_info_["robot"]["time_at_goal"] = std::chrono::duration_cast<std::chrono::duration<double>>(
    std::chrono::steady_clock::now() - goal_hit_time).count();
  play_info_["robot"]["target"]["x"] = waypoints[index].position.x();
  play_info_["robot"]["target"]["y"] = waypoints[index].position.y();
  play_info_["robot"]["target"]["angle_mode"] = waypoints[index].angle_mode;
  play_info_["robot"]["target"]["theta"] = waypoints[index].heading;
  play_info_["robot"]["pos"]["x"] = robot.pos.x();
  play_info_["robot"]["pos"]["y"] = robot.pos.y();
  play_info_["robot"]["pos"]["t"] = robot.theta;
  play_info_["robot"]["vel"]["x"] = robot.vel.x();
  play_info_["robot"]["vel"]["y"] = robot.vel.y();
  play_info_["robot"]["vel"]["t"] = robot.omega;

  for (std::size_t i = 0; i < waypoints.size(); i++) {
    overlays_.drawCircle(
      "controls_test_point" + std::to_string(i),
      ateam_geometry::makeCircle(waypoints[i].position, .05),
      "blue",
      "blue");
  }

  return maybe_motion_commands;
}

bool ControlsTestPlay::isGoalHit(const Robot & robot)
{
  const bool position_goal_hit = ateam_geometry::norm(waypoints[index].position - robot.pos) < 0.05;
  const bool heading_goal_hit = [&]() {
      if (waypoints[index].angle_mode == AngleMode::face_absolute) {
        return std::abs(
          angles::shortest_angular_distance(
            waypoints[index].heading,
            robot.theta)) < angles::from_degrees(5);
      } else {
        return true;
      }
    }();
  return position_goal_hit && heading_goal_hit;
}

}  // namespace ateam_kenobi::plays
