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

  RigidBodyState current_state = RigidBodyStateFromPose(start_pose);
  current_state.twist.linear.x = start_velocity.x();
  current_state.twist.linear.y = start_velocity.y();

  for(const auto & segment : segments) {
    const auto trajectory = ateam_controls_compute_optimal_bangbang_traj_3d(
      current_state, RigidBodyStateFromPose(segment.target));
    for(double t = 0.0; t < segment.duration; t += delta_t) {
      const auto state_at_t = ateam_controls_compute_bangbang_traj_3d_state_at_t(
        trajectory, current_state, 0.0, t);
      points.push_back(ateam_geometry::Point(
        state_at_t.pose.position.x,
        state_at_t.pose.position.y));
    }
    current_state = ateam_controls_compute_bangbang_traj_3d_state_at_t(
      trajectory, current_state, 0.0, segment.duration);
  }

  return points;
}

std::vector<std::vector<ateam_geometry::Point>> TrajectorySpline::ToPointsBySegment(double delta_t) const
{
  std::vector<std::vector<ateam_geometry::Point>> points;
  if (segments.empty()) {
    return points;
  }

  RigidBodyState current_state = RigidBodyStateFromPose(start_pose);
  current_state.twist.linear.x = start_velocity.x();
  current_state.twist.linear.y = start_velocity.y();

  for(const auto & segment : segments) {
    const auto trajectory = ateam_controls_compute_optimal_bangbang_traj_3d(
      current_state, RigidBodyStateFromPose(segment.target));
    auto & segment_points = points.emplace_back();
    for(double t = 0.0; t < segment.duration; t += delta_t) {
      const auto state_at_t = ateam_controls_compute_bangbang_traj_3d_state_at_t(
        trajectory, current_state, 0.0, t);
      segment_points.push_back(ateam_geometry::Point(
        state_at_t.pose.position.x,
        state_at_t.pose.position.y));
    }
    current_state = ateam_controls_compute_bangbang_traj_3d_state_at_t(
      trajectory, current_state, 0.0, segment.duration);
  }

  return points;
}

}  // namespace ateam_path_planning
