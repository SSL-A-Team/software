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

Vector6C_t RigidBodyStateFromRobot(const ateam_game_state::Robot & robot)
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


Vector3C_t RigidBodyStateFromPose(const Pose & pose)
{
  return Vector3C_t{
    static_cast<float>(pose.position.x()),
    static_cast<float>(pose.position.y()),
    static_cast<float>(pose.heading)
  };
}

double GetBangBangTrajectoryDuration(const BangBangTraj3D & trajectory)
{
  return std::max(std::max(trajectory.x.t4, trajectory.y.t4), trajectory.z.t4);
}

}  // namespace ateam_path_planning
