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


#ifndef BALLSENSE_EMULATOR_HPP_
#define BALLSENSE_EMULATOR_HPP_

#include <angles/angles.h>
#include <ranges>
#include <array>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/normalize.hpp>
#include "core/types/world.hpp"

namespace ateam_kenobi
{

class BallSenseEmulator
{
public:
  BallSenseEmulator()
  {
    std::ranges::fill(prev_ball_detecteds_, false);
  }

  void Update(World & world)
  {
    std::ranges::for_each(
      world.our_robots, [this, &world](Robot & robot) {
        EmulateForRobot(robot, world.ball.pos);
      });
  }

private:
  static constexpr double kDistanceThreshold = 0.08 + kBallRadius;
  static constexpr double kAngleThreshold = 0.15;
  static constexpr double kHysteresisMultiplier = 2.0;

  std::array<bool, 16> prev_ball_detecteds_;

  void EmulateForRobot(Robot & robot, const ateam_geometry::Point & ball_pos)
  {
    float hysteresis = 1.0;
    if (prev_ball_detecteds_.at(robot.id)) {
      hysteresis = kHysteresisMultiplier;
    }

    const auto robot_to_ball = ball_pos - robot.pos;
    const auto distance_to_ball = ateam_geometry::norm(robot.pos, ball_pos);
    const auto robot_to_ball_angle = std::atan2(robot_to_ball.y(), robot_to_ball.x());
    const auto close_to_ball = distance_to_ball < (kDistanceThreshold + kBallRadius) * hysteresis;
    const auto facing_ball =
      angles::shortest_angular_distance(
      robot.theta,
      robot_to_ball_angle) < (kAngleThreshold * hysteresis);
    robot.breakbeam_ball_detected = close_to_ball && facing_ball;
  }
};

}  // namespace ateam_kenobi

#endif  // BALLSENSE_EMULATOR_HPP_
