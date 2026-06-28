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

#include "ateam_path_planning/trajectory_spline.hpp"
#include <ateam_controls/ateam_controls.h>
#include "ateam_path_planning/controls_lib_adapters.hpp"

namespace ateam_path_planning
{

std::vector<ateam_geometry::Point> TrajectorySpline::ToPoints(double delta_t) const
{
  std::vector<ateam_geometry::Point> points;
  if (segments.empty()) {
    return points;
  }

  Vector6C_t current_state {
    static_cast<float>(start_pose.position.x()),
    static_cast<float>(start_pose.position.y()),
    static_cast<float>(start_pose.heading),
    static_cast<float>(start_velocity.x()),
    static_cast<float>(start_velocity.y()),
    0.0f
  };

  const auto trajectory_params = ateam_controls_default_traj_params();

  for(const auto & segment : segments) {
    BangBangTraj3D_t trajectory;
    if(const auto err =
      ateam_controls_traj_from_target_pose(current_state, Vector3FromPose(segment.target),
        trajectory_params, &trajectory); err != ATEAM_CONTROLS_OK)
    {
      throw ControlsException(err);
    }
    for(double t = 0.0; t < segment.duration; t += delta_t) {
      Vector6C_t state_at_t;
      if(const auto err =
        ateam_controls_traj_state_at(trajectory, t, &state_at_t);
        err != ATEAM_CONTROLS_OK)
      {
        throw ControlsException(err);
      }
      points.push_back(ateam_geometry::Point(
        state_at_t.data[0],
        state_at_t.data[1]));
    }
    if(const auto err =
      ateam_controls_traj_state_at(trajectory, segment.duration,
        &current_state); err != ATEAM_CONTROLS_OK)
    {
      throw ControlsException(err);
    }
  }

  return points;
}

std::vector<std::vector<ateam_geometry::Point>> TrajectorySpline::ToPointsBySegment(
  double delta_t) const
{
  std::vector<std::vector<ateam_geometry::Point>> points;
  if (segments.empty()) {
    return points;
  }

  Vector6C_t current_state {
    static_cast<float>(start_pose.position.x()),
    static_cast<float>(start_pose.position.y()),
    static_cast<float>(start_pose.heading),
    static_cast<float>(start_velocity.x()),
    static_cast<float>(start_velocity.y()),
    0.0f
  };

  const auto trajectory_params = ateam_controls_default_traj_params();

  for(const auto & segment : segments) {
    BangBangTraj3D_t trajectory;
    if(const auto err =
      ateam_controls_traj_from_target_pose(current_state, Vector3FromPose(segment.target),
        trajectory_params, &trajectory); err != ATEAM_CONTROLS_OK)
    {
      throw ControlsException(err);
    }
    auto & segment_points = points.emplace_back();
    for(double t = 0.0; t < segment.duration; t += delta_t) {
      Vector6C_t state_at_t;
      if(const auto err =
        ateam_controls_traj_state_at(trajectory, t, &state_at_t);
        err != ATEAM_CONTROLS_OK)
      {
        throw ControlsException(err);
      }
      segment_points.push_back(ateam_geometry::Point(
        state_at_t.data[0],
        state_at_t.data[1]));
    }
    if(const auto err =
      ateam_controls_traj_state_at(trajectory, segment.duration,
        &current_state); err != ATEAM_CONTROLS_OK)
    {
      throw ControlsException(err);
    }
  }

  return points;
}

}  // namespace ateam_path_planning
