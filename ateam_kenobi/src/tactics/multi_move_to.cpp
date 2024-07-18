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

#include "multi_move_to.hpp"
#include <ateam_common/robot_constants.hpp>

namespace ateam_kenobi::tactics
{

MultiMoveTo::MultiMoveTo(stp::Options stp_options)
: stp::Tactic(stp_options),
  easy_move_tos_(createIndexedChildren<play_helpers::EasyMoveTo>("easy_move_to"))
{}

void MultiMoveTo::Reset()
{
  for (auto & emt : easy_move_tos_) {
    emt.reset();
  }
}

std::vector<ateam_geometry::Point> MultiMoveTo::GetAssignmentPoints()
{
  return target_points_;
}

void MultiMoveTo::RunFrame(
  const World & world,
  const std::vector<std::optional<Robot>> & robots,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & motion_commands)
{
  for (auto ind = 0ul; ind < robots.size(); ++ind) {
    const auto & maybe_robot = robots[ind];
    if (!maybe_robot) {
      continue;
    }
    const auto & robot = *maybe_robot;
    const auto & target_position = target_points_[ind];

    auto & easy_move_to = easy_move_tos_.at(robot.id);

    auto viz_circle = ateam_geometry::makeCircle(target_position, kRobotRadius);
    getOverlays().drawCircle(
      "destination_" + std::to_string(
        robot.id), viz_circle, "blue", "transparent");

    easy_move_to.setTargetPosition(target_position);
    easy_move_to.face_point(world.ball.pos);

    motion_commands.at(robot.id) = easy_move_to.runFrame(robot, world);
  }
}

}  // namespace ateam_kenobi::tactics
