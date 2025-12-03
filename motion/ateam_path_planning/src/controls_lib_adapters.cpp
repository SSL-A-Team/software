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

#include "ateam_path_planning/controls_lib_adapters.hpp"

namespace ateam_path_planning
{

RigidBodyState RigidBodyStateFromRobot(const ateam_game_state::Robot & robot)
{
  RigidBodyState state;
  state.pose.position.x = robot.pos.x();
  state.pose.position.y = robot.pos.y();
  state.pose.position.z = 0.0;
  double half_theta = robot.theta / 2.0;
  state.pose.orientation.x = 0.0;
  state.pose.orientation.y = 0.0;
  state.pose.orientation.z = sin(half_theta);
  state.pose.orientation.w = cos(half_theta);
  state.twist.linear.x = robot.vel.x();
  state.twist.linear.y = robot.vel.y();
  state.twist.linear.z = 0.0;
  state.twist.angular.x = 0.0;
  state.twist.angular.y = 0.0;
  state.twist.angular.z = robot.omega;
  return state;
}


RigidBodyState RigidBodyStateFromPose(const Pose & pose)
{
  RigidBodyState state;
  state.pose.position.x = pose.position.x();
  state.pose.position.y = pose.position.y();
  state.pose.position.z = 0.0;
  double half_theta = pose.heading / 2.0;
  state.pose.orientation.x = 0.0;
  state.pose.orientation.y = 0.0;
  state.pose.orientation.z = sin(half_theta);
  state.pose.orientation.w = cos(half_theta);
  state.twist.linear.x = 0.0;
  state.twist.linear.y = 0.0;
  state.twist.linear.z = 0.0;
  state.twist.angular.x = 0.0;
  state.twist.angular.y = 0.0;
  state.twist.angular.z = 0.0;
  return state;
}

double GetBangBangTrajectoryDuration(const BangBangTraj3D & trajectory)
{
  return std::max(std::max(trajectory.x_traj.t4, trajectory.y_traj.t4), trajectory.z_traj.t4);
}

}  // namespace ateam_path_planning
