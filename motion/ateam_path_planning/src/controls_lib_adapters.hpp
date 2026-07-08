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

#ifndef CONTROLS_LIB_ADAPTERS_HPP_
#define CONTROLS_LIB_ADAPTERS_HPP_

#include <ateam_controls/ateam_controls.h>
#include <ateam_game_state/robot.hpp>
#include "ateam_path_planning/pose.hpp"
#include "ateam_path_planning/planner_options.hpp"

namespace ateam_path_planning
{

Vector6C_t Vector6FromRobot(const ateam_game_state::Robot & robot);

Vector3C_t Vector3FromPose(const Pose & pose);

Vector6C_t Vector6FromPose(const Pose & pose);

Pose PoseFromVector3(const Vector3C_t & vector);

Pose PoseFromVector6(const Vector6C_t & vector);

double GetBangBangTrajectoryDuration(const BangBangTraj3D_t & trajectory);

Vector6C_t GetStateAtT(const BangBangTraj3D_t & trajectory, const double t);

TrajectoryParams_t BuildTrajectoryParams(const Limits & limits);

BangBangTraj3D_t GenerateTrajectory(
  const Vector6C_t & init_state, const Vector3C_t & target_pose,
  const TrajectoryParams_t params);

class ControlsException : public std::exception
{
public:
  explicit ControlsException(int32_t err)
  : raw_(err) {}

  ~ControlsException() override = default;

  const char * what() const noexcept override;

  int32_t GetRaw() const
  {
    return raw_;
  }

private:
  int32_t raw_;
};

}  // namespace ateam_path_planning

#endif  // CONTROLS_LIB_ADAPTERS_HPP_
