// Copyright 2025 A Team
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


#include "corner_lineup_play.hpp"
#include <ateam_common/robot_constants.hpp>
#include "core/types/robot.hpp"
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

CornerLineupPlay::CornerLineupPlay(stp::Options stp_options, double x_mult, double y_mult)
: stp::Play(kPlayName, stp_options),
  x_mult_(x_mult),
  y_mult_(y_mult)
{
  createIndexedChildren<play_helpers::EasyMoveTo>(easy_move_tos_, "EasyMoveTo");
  setEnabled(false);
}

stp::PlayScore CornerLineupPlay::getScore(const World &)
{
  return stp::PlayScore::NaN();
}

void CornerLineupPlay::reset()
{
  for (auto & move_to : easy_move_tos_) {
    move_to.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> CornerLineupPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  auto current_available_robots = play_helpers::getAvailableRobots(world);

  const ateam_geometry::Point start_point(x_mult_ * world.field.field_length / 2.0,
    y_mult_ * world.field.field_width / 2.0);
  const auto x_dir = -1.0 * std::copysign(1.0, x_mult_);
  const ateam_geometry::Vector direction(x_dir * 2 * kRobotDiameter, 0.0);

  for (auto ind = 0ul; ind < current_available_robots.size(); ++ind) {
    const auto & robot = current_available_robots[ind];
    auto & easy_move_to = easy_move_tos_.at(robot.id);
    easy_move_to.setTargetPosition(start_point + direction * ind);
    easy_move_to.face_absolute(0.0);
    maybe_motion_commands.at(robot.id) = easy_move_to.runFrame(robot, world);
  }

  return maybe_motion_commands;
}

}  // namespace ateam_kenobi::plays
