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

#include "ateam_path_planning/trajectory_spline.hpp"
#include "trajectory_spline_impl.hpp"

namespace ateam_path_planning
{

TrajectorySpline::TrajectorySpline(const TrajectorySpline & other)
: impl_(std::make_unique<TrajectorySplineImpl>(*other.impl_))
{
}

TrajectorySpline::TrajectorySpline(TrajectorySpline && other)
: impl_(std::move(other.impl_))
{
}

TrajectorySpline::~TrajectorySpline() = default;

TrajectorySpline & TrajectorySpline::operator=(const TrajectorySpline & other)
{
  impl_ = std::make_unique<TrajectorySplineImpl>(*other.impl_);
  return *this;
}

TrajectorySpline & TrajectorySpline::operator=(TrajectorySpline && other)
{
  impl_ = std::move(other.impl_);
  return *this;
}

std::optional<Pose> TrajectorySpline::GetStateAtT(double t) const
{
  return impl_->GetStateAtT(t);
}

std::optional<Pose> TrajectorySpline::GetStateAt(
  const std::chrono::steady_clock::time_point & time_point) const
{
  return impl_->GetStateAt(time_point);
}

std::optional<Pose> TrajectorySpline::GetTargetAtT(double t) const
{
  return impl_->GetTargetAtT(t);
}

std::optional<Pose> TrajectorySpline::GetTargetAtNow() const
{
  return impl_->GetTargetAtNow();
}

std::vector<ateam_geometry::Point> TrajectorySpline::ToPoints(double delta_t) const
{
  return impl_->ToPoints(delta_t);
}

std::vector<std::vector<ateam_geometry::Point>> TrajectorySpline::ToPointsBySegment(
  double delta_t) const
{
  return impl_->ToPointsBySegment(delta_t);
}

Pose TrajectorySpline::GetStartPose() const
{
  return impl_->GetStartPose();
}

Pose TrajectorySpline::GetEndPose() const
{
  return impl_->GetEndPose();
}

size_t TrajectorySpline::GetSegmentCount() const
{
  return impl_->GetSegmentCount();
}

double TrajectorySpline::GetTotalDuration() const
{
  return impl_->GetTotalDuration();
}

std::chrono::steady_clock::time_point TrajectorySpline::GetStartTime() const
{
  return impl_->GetStartTime();
}

ateam_geometry::Point TrajectorySpline::GetFirstTransitionPoint() const
{
  return impl_->GetFirstTransitionPoint();
}

TrajectorySpline::TrajectorySpline(TrajectorySplineImpl & impl)
: impl_(std::make_unique<TrajectorySplineImpl>(impl))
{
}


}  // namespace ateam_path_planning
