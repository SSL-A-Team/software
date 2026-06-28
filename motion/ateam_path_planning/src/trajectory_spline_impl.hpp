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

#ifndef TRAJECTORY_SPLINE_IMPL_HPP_
#define TRAJECTORY_SPLINE_IMPL_HPP_

#include <ateam_controls/ateam_controls.h>
#include <optional>
#include <vector>
#include <ateam_geometry/types.hpp>
#include "ateam_path_planning/pose.hpp"
#include "ateam_path_planning/trajectory_spline.hpp"

namespace ateam_path_planning
{

struct TrajectorySplineSegment
{
  double duration;
  Pose target;
  BangBangTraj3D_t trajectory;
};

struct TrajectorySplineImpl
{
  std::chrono::steady_clock::time_point start_time;
  Vector6C_t start_state;
  TrajectoryParams_t trajectory_params;
  std::vector<TrajectorySplineSegment> segments;

  std::optional<Pose> GetStateAtT(double t) const;

  std::optional<Pose> GetStateAt(const std::chrono::steady_clock::time_point & time_point) const;

  std::optional<Pose> GetTargetAtT(double t) const;

  std::optional<Pose> GetTargetAtNow() const;

  std::vector<ateam_geometry::Point> ToPoints(double delta_t = 0.1) const;

  std::vector<std::vector<ateam_geometry::Point>> ToPointsBySegment(double delta_t = 0.1) const;

  Pose GetStartPose() const;

  Pose GetEndPose() const;

  size_t GetSegmentCount() const;

  std::chrono::steady_clock::time_point GetStartTime() const;
};

TrajectorySpline MakeTrajectorySpline(TrajectorySplineImpl &);

}  // namespace ateam_path_planning

#endif  // TRAJECTORY_SPLINE_IMPL_HPP_
