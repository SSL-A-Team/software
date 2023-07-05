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
#include "types/world.hpp"
#include "skills/goalie.hpp"
#include "robot_assignment.hpp"

namespace ateam_kenobi::plays
{
OurKickoffPlay::OurKickoffPlay(visualization::OverlayPublisher & overlay_publisher)
: BasePlay(overlay_publisher)
{
}

void OurKickoffPlay::reset()
{
  motion_controller_.reset();
  positions_to_assign_.clear();
  
  // Get a kicker
  ateam_geometry::Point kicker_point = ateam_geometry::Point(-0.55, 0);
  positions_to_assign_.push_back(kicker_point);
  // Get 4 defenders
  positions_to_assign_.push_back(ateam_geometry::Point(-0.3, -1.5));
  positions_to_assign_.push_back(ateam_geometry::Point(-0.3, 1.5));
  positions_to_assign_.push_back(ateam_geometry::Point(-2, 2));
  positions_to_assign_.push_back(ateam_geometry::Point(-2, -2));
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> OurKickoffPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  for (const auto & maybe_robot : world.our_robots) {
    if (maybe_robot && maybe_robot.value().id != world.referee_info.our_goalie_id) {
      available_robots_.push_back(maybe_robot.value());
    }
  }

  // Get a goalie - set, not assigned
  int our_goalie_id = world.referee_info.our_goalie_id;
  ateam_geometry::Point goalie_point = ateam_kenobi::skills::get_goalie_defense_point(world);

  const auto goalie_path = path_planner_.getPath(
    world.our_robots.at(our_goalie_id).value().pos, 
    goalie_point, world, {});
  motion_controller_.set_trajectory(goalie_path);

  const auto current_time = std::chrono::duration_cast<std::chrono::duration<double>>(
      world.current_time.time_since_epoch()).count();
  const auto goalie_robot = world.our_robots.at(our_goalie_id);
  maybe_motion_commands.at(our_goalie_id) = motion_controller_.get_command(
    goalie_robot.value(), current_time);

  const auto & robot_assignments = robot_assignment::assign(available_robots_, positions_to_assign_);

  for (const auto [robot_id, pos_ind] : robot_assignments) {
    const auto & maybe_assigned_robot = world.our_robots.at(robot_id);
    if (!maybe_assigned_robot) {
      // TODO Log this
      // Assigned non-available robot
      continue;
    }
    const auto & robot = maybe_assigned_robot.value();
    const auto & destination = positions_to_assign_.at(pos_ind);
    const auto path = path_planner_.getPath(robot.pos, destination, world, {});
    if (path.empty()) {
      overlay_publisher_.drawCircle(
        "highlight_test_robot",
        ateam_geometry::makeCircle(robot.pos, 0.2), "red", "transparent");
      return {};
    }
    motion_controller_.set_trajectory(path);
    prev_assigned_id_ = robot.id;
    const auto current_time = std::chrono::duration_cast<std::chrono::duration<double>>(
      world.current_time.time_since_epoch()).count();
    maybe_motion_commands.at(robot_id) = motion_controller_.get_command(robot, current_time);
  }

  return maybe_motion_commands;
}
}  // namespace ateam_kenobi::plays
