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

#include "controls_lib_adapters.hpp"
#include <ateam_controls/ateam_controls.h>
#include <algorithm>

namespace ateam_path_planning
{

Vector6C_t Vector6FromRobot(const ateam_game_state::Robot & robot)
{
  return Vector6C_t{
    static_cast<float>(robot.pos.x()),
    static_cast<float>(robot.pos.y()),
    static_cast<float>(robot.theta),
    static_cast<float>(robot.vel.x()),
    static_cast<float>(robot.vel.y()),
    static_cast<float>(robot.omega)
  };
}


Vector3C_t Vector3FromPose(const Pose & pose)
{
  return Vector3C_t{
    static_cast<float>(pose.position.x()),
    static_cast<float>(pose.position.y()),
    static_cast<float>(pose.heading)
  };
}

Vector6C_t Vector6FromPose(const Pose & pose)
{
  return Vector6C_t{
    static_cast<float>(pose.position.x()),
    static_cast<float>(pose.position.y()),
    static_cast<float>(pose.heading),
    0.0f,
    0.0f,
    0.0f
  };
}

Pose PoseFromVector3(const Vector3C_t & vector)
{
  return Pose{
    .position = ateam_geometry::Point(vector.x, vector.y),
    .heading = vector.z
  };
}

Pose PoseFromVector6(const Vector6C_t & vector)
{
  return Pose{
    .position = ateam_geometry::Point(vector.data[0], vector.data[1]),
    .heading = vector.data[2]
  };
}

double GetBangBangTrajectoryDuration(const BangBangTraj3D_t & trajectory)
{
  return std::max(std::max(trajectory.x.t4, trajectory.y.t4), trajectory.z.t4);
}


Vector6C_t GetStateAtT(const BangBangTraj3D_t & trajectory, const double t)
{
  Vector6C_t state_at_t;
  if(const auto err =
    ateam_controls_traj_state_at(trajectory, t, &state_at_t);
    err != ATEAM_CONTROLS_OK)
  {
    throw ControlsException(err);
  }
  return state_at_t;
}

TrajectoryParams_t BuildTrajectoryParams(const Limits & limits)
{
  auto params = ateam_controls_default_traj_params();
  if(limits.angular_acceleration != 0.0f) {
    params.max_accel_angular = limits.angular_acceleration;
  }
  if(limits.angular_velocity != 0.0f) {
    params.max_vel_angular = limits.angular_velocity;
  }
  if(limits.linear_acceleration != 0.0f) {
    params.max_accel_linear = limits.linear_acceleration;
  }
  if(limits.linear_velocity != 0.0f) {
    params.max_vel_linear = limits.linear_velocity;
  }
  return params;
}

BangBangTraj3D_t GenerateTrajectory(
  const Vector6C_t & init_state, const Vector3C_t & target_pose,
  const TrajectoryParams_t params)
{
  BangBangTraj3D_t trajectory;
  if(const auto err =
    ateam_controls_traj_from_target_pose(init_state, target_pose, params,
          &trajectory); err != ATEAM_CONTROLS_OK)
  {
    throw ControlsException(err);
  }
  return trajectory;
}

const char * ControlsException::what() const noexcept
{
  switch(raw_) {
    case ATEAM_CONTROLS_OK:
      return "OK";
    case ATEAM_CONTROLS_INVALID_INPUT:
      return "Invalid Input";
    case ATEAM_CONTROLS_SINGULAR:
      return "Singular";
    case ATEAM_CONTROLS_NO_SOLUTION:
      return "No Solution";
    case ATEAM_CONTROLS_INVALID_TIME:
      return "Invalid Time";
    case ATEAM_CONTROLS_EXCEEDS_LIMIT:
      return "Exceeds Limit";
    default:
      return "Unknown";
  }
}

}  // namespace ateam_path_planning
