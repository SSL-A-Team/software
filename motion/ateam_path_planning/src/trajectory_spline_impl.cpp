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

#include "trajectory_spline_impl.hpp"
#include <ateam_controls/ateam_controls.h>
#include <vector>
#include "controls_lib_adapters.hpp"

namespace ateam_path_planning
{

std::optional<Pose> TrajectorySplineImpl::GetStateAtT(double t) const
{
  if(segments.empty()) {
    return PoseFromVector6(start_state);
  }
  if(t < 0.0) {
    return PoseFromVector6(start_state);
  }
  double path_t = 0.0;
  for(const auto & segment : segments) {
    if (t > path_t + segment.duration) {
      path_t += segment.duration;
      continue;
    }
    Vector6C_t state;
    if(const auto err =
      ateam_controls_traj_state_at(segment.trajectory, t - path_t,
        &state); err != ATEAM_CONTROLS_OK)
    {
      throw ControlsException(err);
    }
    return PoseFromVector6(state);
  }
  return segments.back().target;
}

std::optional<Pose> TrajectorySplineImpl::GetStateAt(
  const std::chrono::steady_clock::time_point & time_point) const
{
  const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(time_point -
      start_time).count();
  return GetStateAtT(elapsed);
}

std::optional<Pose> TrajectorySplineImpl::GetTargetAtT(double t) const
{
  if(segments.empty()) {
    return PoseFromVector6(start_state);
  }
  double path_t = 0.0;
  for(const auto & segment : segments) {
    path_t += segment.duration;
    if (t < path_t) {
      return segment.target;
    }
  }
  return segments.back().target;
}

std::optional<Pose> TrajectorySplineImpl::GetTargetAtNow() const
{
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time);
  return GetTargetAtT(elapsed.count());
}

std::vector<ateam_geometry::Point> TrajectorySplineImpl::ToPoints(double delta_t) const
{
  std::vector<ateam_geometry::Point> points;
  if (segments.empty()) {
    return points;
  }

  for(const auto & segment : segments) {
    for(double t = 0.0; t < segment.duration; t += delta_t) {
      Vector6C_t state_at_t;
      if(const auto err =
        ateam_controls_traj_state_at(segment.trajectory, t, &state_at_t);
        err != ATEAM_CONTROLS_OK)
      {
        throw ControlsException(err);
      }
      points.push_back(ateam_geometry::Point(
        state_at_t.data[0],
        state_at_t.data[1]));
    }
  }

  return points;
}

std::vector<std::vector<ateam_geometry::Point>> TrajectorySplineImpl::ToPointsBySegment(
  double delta_t) const
{
  std::vector<std::vector<ateam_geometry::Point>> points;
  if (segments.empty()) {
    return points;
  }

  for(const auto & segment : segments) {
    auto & segment_points = points.emplace_back();
    for(double t = 0.0; t < segment.duration; t += delta_t) {
      Vector6C_t state_at_t;
      if(const auto err =
        ateam_controls_traj_state_at(segment.trajectory, t, &state_at_t);
        err != ATEAM_CONTROLS_OK)
      {
        throw ControlsException(err);
      }
      segment_points.push_back(ateam_geometry::Point(
        state_at_t.data[0],
        state_at_t.data[1]));
    }
  }

  return points;
}

Pose TrajectorySplineImpl::GetStartPose() const
{
  return PoseFromVector6(start_state);
}

Pose TrajectorySplineImpl::GetEndPose() const
{
  if(segments.empty()) {
    return PoseFromVector6(start_state);
  }
  return segments.back().target;
}

size_t TrajectorySplineImpl::GetSegmentCount() const
{
  return segments.size();
}

std::chrono::steady_clock::time_point TrajectorySplineImpl::GetStartTime() const
{
  return start_time;
}

TrajectorySpline MakeTrajectorySpline(TrajectorySplineImpl & impl)
{
  return TrajectorySpline(impl);
}

}  // namespace ateam_path_planning
