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
#include "core/types/state_types.hpp"
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

CornerLineupPlay::CornerLineupPlay(stp::Options stp_options, double x_mult, double y_mult)
: stp::Play(stp_options.name, stp_options),
  x_mult_(x_mult),
  y_mult_(y_mult),
  multi_move_to_(createChild<tactics::MultiMoveTo>("MultiMoveTo"))
{
  setEnabled(false);
}

stp::PlayScore CornerLineupPlay::getScore(const World &)
{
  return stp::PlayScore::NaN();
}

std::array<std::optional<RobotCommand>, 16> CornerLineupPlay::runFrame(
  const World & world)
{
  std::array<std::optional<RobotCommand>, 16> maybe_motion_commands;

  const ateam_geometry::Point start_point(x_mult_ * world.field.field_length / 2.0,
    y_mult_ * world.field.field_width / 2.0);
  const auto x_dir = -1.0 * std::copysign(1.0, x_mult_);
  const ateam_geometry::Vector direction(x_dir * 2 * kRobotDiameter, 0.0);

  const std::vector<Robot> available_robots = play_helpers::getAvailableRobots(world);

  std::vector<ateam_geometry::Point> target_points;
  std::generate_n(std::back_inserter(target_points), available_robots.size(),
    [spot = start_point, direction, n = 0]() mutable {
      return spot + (direction * n++);
    });

  multi_move_to_.SetTargetPoints(target_points);
  multi_move_to_.SetFaceAbsolue(0.0);
  multi_move_to_.RunFrame(available_robots, maybe_motion_commands);

  return maybe_motion_commands;
}

}  // namespace ateam_kenobi::plays
