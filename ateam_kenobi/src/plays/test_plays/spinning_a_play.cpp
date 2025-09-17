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


#include "spinning_a_play.hpp"
#include <algorithm>
#include <ateam_common/robot_constants.hpp>
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

SpinningAPlay::SpinningAPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  multi_move_to_(createChild<tactics::MultiMoveTo>("multi_move_to"))
{
  base_shape_ = {
    ateam_geometry::Point(0, 0),
    ateam_geometry::Point(-0.44, -0.44),
    ateam_geometry::Point(-0.22, 0),
    ateam_geometry::Point(0, 0.44),
    ateam_geometry::Point(0.22, 0),
    ateam_geometry::Point(0.44, -0.44),
  };
}

void SpinningAPlay::reset()
{
  angle_ = 0.0;
}

std::array<std::optional<RobotCommand>,
  16> SpinningAPlay::runFrame(const World & world)
{
  const auto available_robots = play_helpers::getAvailableRobots(world);

  const auto num_robots = std::min(available_robots.size(), base_shape_.size());

  CGAL::Aff_transformation_2<ateam_geometry::Kernel> rotate_transform(CGAL::ROTATION,
    std::sin(angle_), std::cos(angle_));

  std::array<std::optional<RobotCommand>, 16> motion_commands;

  std::vector<ateam_geometry::Point> targets;
  std::transform(
    base_shape_.begin(), base_shape_.begin() + num_robots, std::back_inserter(targets),
    [&rotate_transform](const auto & point) {
      return point.transform(rotate_transform);
    });
  multi_move_to_.SetTargetPoints(targets);
  multi_move_to_.SetFaceAbsolue(-M_PI_2 + angle_);
  multi_move_to_.RunFrame(available_robots, motion_commands);
  for(auto & maybe_motion_command : motion_commands) {
    if(!maybe_motion_command) continue;
    maybe_motion_command->motion_intent.planner_options.footprint_inflation = 0.1;
  }

  if (angle_ < kNumRotations * 2 * M_PI) {
    angle_ += kAngleSpeed;
  }

  return motion_commands;
}

}  // namespace ateam_kenobi::plays
