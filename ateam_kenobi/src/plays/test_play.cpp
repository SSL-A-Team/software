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

#include "test_play.hpp"
#include "ateam_geometry/types.hpp"
#include "types/world.hpp"
#include "skills/goalie.hpp"
#include "robot_assignment.hpp"

namespace ateam_kenobi::plays
{
TestPlay::TestPlay(visualization::OverlayPublisher & overlay_publisher)
: BasePlay(overlay_publisher)
{
}

void TestPlay::reset()
{
  motion_controller_.reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> TestPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  std::vector<Robot> available_robots;
  for (const auto & maybe_robot : world.our_robots) {
    if (maybe_robot) {
      available_robots.push_back(maybe_robot.value());
    }
  }

  std::vector<ateam_geometry::Point> test_positions;
  test_positions.push_back(ateam_kenobi::skills::get_goalie_defense_point(world));

  const auto & robot_assignments = robot_assignment::assign(available_robots, test_positions);

  for (const auto [robot_id, pos_ind] : robot_assignments) {
    const auto & maybe_assigned_robot = world.our_robots.at(robot_id);
    if (!maybe_assigned_robot) {
      // TODO Log this
      // Assigned non-available robot
      continue;
    }
    const auto & robot = maybe_assigned_robot.value();
    const auto & destination = test_positions.at(pos_ind);
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

    overlay_publisher_.drawLine("test_path", path, "purple");
    overlay_publisher_.drawCircle(
      "highlight_test_robot", ateam_geometry::makeCircle(
        robot.pos,
        0.2), "purple",
      "transparent");
  }

  return maybe_motion_commands;
}
}  // namespace ateam_kenobi::plays
