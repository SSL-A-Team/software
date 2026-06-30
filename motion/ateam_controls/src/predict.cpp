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


#include "ateam_controls_cpp/predict.hpp"
#include "ateam_controls/ateam_controls.h"
#include "ateam_controls_cpp/exceptions.hpp"

namespace ateam_controls_cpp::predict
{

ateam_geometry::Point PositionAtT(const Robot & robot, const modes::Off &, const double)
{
  return robot.pos;
}

ateam_geometry::Point PositionAtT(const Robot & robot, const modes::EStopBrake &, const double)
{
  return robot.pos;
}

ateam_geometry::Point PositionAtT(
  const Robot & robot, const modes::GlobalPosition & params,
  const double t)
{
  const auto default_c_params = ateam_controls_default_traj_params();
  TrajectoryParams c_params{
    .max_vel_linear = params.max_linear_vel != 0.0f ? params.max_linear_vel : default_c_params.max_vel_linear,
    .max_vel_angular = params.max_angular_vel != 0.0f ? params.max_angular_vel : default_c_params.max_vel_angular,
    .max_accel_linear = params.max_linear_acc != 0.0f ? params.max_linear_acc : default_c_params.max_accel_linear,
    .max_accel_angular = params.max_angular_acc != 0.0f ? params.max_angular_acc : default_c_params.max_accel_angular
  };
  Vector6C c_start_state{
    .data = {
      static_cast<float>(robot.pos.x()),
      static_cast<float>(robot.pos.y()),
      static_cast<float>(robot.theta),
      static_cast<float>(robot.vel.x()),
      static_cast<float>(robot.vel.y()),
      static_cast<float>(robot.omega)
    }
  };
  Vector3C c_goal_state{
    .x = params.global_x,
    .y = params.global_y,
    .z = params.global_theta
  };
  BangBangTraj3D trajectory;
  if(const auto err = ateam_controls_traj_from_target_pose(c_start_state, c_goal_state, c_params,
      &trajectory); err != ATEAM_CONTROLS_OK)
  {
    throw ControlsException(err);
  }
  Vector6C state;
  if(const auto err = ateam_controls_traj_state_at(trajectory, t, &state);
    err != ATEAM_CONTROLS_OK)
  {
    throw ControlsException(err);
  }
  return ateam_geometry::Point{state.data[0], state.data[1]};
}

ateam_geometry::Point PositionAtT(
  const Robot & robot, const modes::GlobalVelocity & params,
  const double t)
{
  (void)params;
  (void)t;
  return robot.pos;
}

ateam_geometry::Point PositionAtT(
  const Robot & robot, const modes::LocalVelocity & params,
  const double t)
{
  (void)params;
  (void)t;
  return robot.pos;
}

ateam_geometry::Point PositionAtT(
  const Robot & robot, const modes::GlobalAccel & params,
  const double t)
{
  (void)params;
  (void)t;
  return robot.pos;
}

ateam_geometry::Point PositionAtT(
  const Robot & robot, const modes::LocalAccel & params,
  const double t)
{
  (void)params;
  (void)t;
  return robot.pos;
}

ateam_geometry::Point PositionAtT(
  const Robot & robot, const modes::HeadingPivot & params,
  const double t)
{
  (void)params;
  (void)t;
  return robot.pos;
}

ateam_geometry::Point PositionAtT(
  const Robot & robot, const modes::PointPivot & params,
  const double t)
{
  (void)params;
  (void)t;
  return robot.pos;
}

ateam_geometry::Point PositionAtT(
  const Robot & robot, const modes::HeadingLine & params,
  const double t)
{
  (void)params;
  (void)t;
  return robot.pos;
}

ateam_geometry::Point PositionAtT(
  const Robot & robot, const modes::PointLine & params,
  const double t)
{
  (void)params;
  (void)t;
  return robot.pos;
}

}  // namespace ateam_controls_cpp::predict
