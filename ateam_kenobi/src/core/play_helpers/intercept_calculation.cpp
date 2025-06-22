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

#include "intercept_calculation.hpp"
#include <angles/angles.h>
#include <ateam_geometry/angles.hpp>
#include <ateam_common/robot_constants.hpp>

namespace ateam_kenobi::play_helpers
{

InterceptResults calculateIntercept(const World & world, const Robot & robot)
{

  InterceptResults result;

  // ball velocity
  const double vb = ateam_geometry::norm(world.ball.vel);

  // TODO: pass this as an argument or something?
  // max robot velocity (slightly decreased to give robot time to prepare)
  // const double vr = max_speed_ * 0.85;
  const double vr = 1.5;

  // Need to get there a bit in front of the ball
  const double offset_ball_dist = kBallRadius + kRobotRadius + 0.05;
  const auto offset_ball_pos = world.ball.pos + (offset_ball_dist * ateam_geometry::normalize(world.ball.vel));
  const auto ball_to_robot = robot.pos - offset_ball_pos;

  // robot distance to ball projected onto the balls trajectory
  const double d = (world.ball.vel * ball_to_robot) /
    ateam_geometry::norm(world.ball.vel);

  const auto robot_proj_ball = world.ball.vel * d /
    ateam_geometry::norm(world.ball.vel);

  const auto robot_perp_ball = ball_to_robot - robot_proj_ball;

  // robot distance tangent to the balls trajectory
  const double h = ateam_geometry::norm(robot_perp_ball);
  
  result.robot_proj_ball = robot_proj_ball;
  result.robot_perp_ball = robot_perp_ball;
  result.d = d;
  result.h = h;

  /*
    Initial Equation:
      d - vb*t = sin(acos(h/(vr*t))) * vr*t
    Solve for t:
      t = (d * vb - sqrt((d^2*vr^2) + (h^2 * vr^2) - (h^2 * vb^2)) / (vb^2 - vr^2)
  */

  double discriminant = (d * d * vr * vr) + (h * h * vr * vr) - (h * h * vb * vb);
  double denominator = vb * vb - vr * vr;

  // Imaginary result means we will never catch the ball
  if (discriminant < 0) {
    return result;
  }

  // If the robot and ball speeds are very similar then the denominator can go to 0
  if (abs(denominator) < 0.01) {
    // Shift the denominator to represent a slightly faster robot since:
    //  a. we already assume a slower than actual robot speed
    //  b. the ball will decelerate as it rolls
    denominator = -0.1;
  }

  // Time to intercept moving towards the ball
  double t = (d * vb - sqrt(discriminant)) / denominator;

  // TODO(chachmu): add better checks to see if these times are possible to
  // respond to
  if (t > 0.0) {
    result.t = t;
    result.equation_sign = -1.0;
    result.intercept_pos = std::make_optional(offset_ball_pos + (t * world.ball.vel));

    return result;
  }

  // Time to intercept moving away from the ball
  t = (d * vb + sqrt(discriminant)) / denominator;
  
  if (t > 0.0) {
    result.t = t;
    result.equation_sign = 1.0;
    result.intercept_pos = std::make_optional(offset_ball_pos + (t * world.ball.vel));

    return result;
  }

  return result;
}
}  // namespace ateam_kenobi::play_helpers
