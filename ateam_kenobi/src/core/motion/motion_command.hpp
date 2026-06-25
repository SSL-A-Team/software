// Copyright 2026 A Team
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

#ifndef CORE__MOTION__MOTION_COMMAND_HPP_
#define CORE__MOTION__MOTION_COMMAND_HPP_

#include <cstdint>

namespace ateam_kenobi::motion
{

struct Twist2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

enum class ControlMode
{
  Off = 0,
  GlobalPosition = 1,
  GlobalVelocity = 2,
  LocalVelocity = 3,
  GlobalAccel = 4,
  LocalAccel = 5,
  HeadingPivot = 6,
  PointPivot = 7
};

struct MotionCommand
{
  ControlMode control_mode;
  Twist2D pose;
  Twist2D velocity;
  Twist2D acceleration;
  double limit_vel_linear = 0.0;
  double limit_vel_angular = 0.0;
  double limit_acc_linear = 0.0;
  double limit_acc_angular = 0.0;

  double pivot_target_x = 0.0;
  double pivot_target_y = 0.0;
  double pivot_global_theta = 0.0;
  double pivot_orbit_radius = 0.0;
  double pivot_inset_angle = 0.0;

  uint8_t pivot_direction = 0;
  bool pivot_commpute_inset_angle = false;
};

}  // namespace ateam_kenobi::motion

#endif  // CORE__MOTION__MOTION_COMMAND_HPP_
