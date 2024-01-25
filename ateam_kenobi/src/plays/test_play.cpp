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
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{
TestPlay::TestPlay(
  visualization::OverlayPublisher & overlay_publisher,
  visualization::PlayInfoPublisher & play_info_publisher)
: BasePlay(overlay_publisher, play_info_publisher),
  goalie_skill_(overlay_publisher, play_info_publisher)
{
  play_helpers::EasyMoveTo::CreateArray(easy_move_tos_, overlay_publisher);
}

void TestPlay::reset()
{
  for (auto & move_to : easy_move_tos_) {
    move_to.reset();
  }
  goalie_skill_.reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> TestPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;
  auto current_available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(current_available_robots, world);

  if (current_available_robots.size() > 0) {
    const auto & robot = current_available_robots[0];
    int robot_id = robot.id;
    auto & easy_move_to = easy_move_tos_.at(robot_id);

    easy_move_to.setTargetPosition(world.ball.pos + ateam_geometry::Vector(-.2, 0));
    easy_move_to.face_point(world.ball.pos);
    maybe_motion_commands.at(robot_id) = easy_move_to.runFrame(robot, world);
  }

  goalie_skill_.runFrame(world, maybe_motion_commands);

  play_info_publisher_.send_play_message("Test Play");
  return maybe_motion_commands;
}
}  // namespace ateam_kenobi::plays
