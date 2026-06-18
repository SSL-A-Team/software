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

#ifndef UTILS__PATH_ENDS_AT_HPP_
#define UTILS__PATH_ENDS_AT_HPP_

#include <angles/angles.h>
#include <gmock/gmock.h>
#include <ateam_geometry/types.hpp>
#include <ateam_geometry/comparisons.hpp>
#include <ateam_geometry/epsilon.hpp>
#include "ateam_path_planning/trajectory_spline.hpp"

class PathEndsAtMatcher
  : public ::testing::MatcherInterface<const ateam_path_planning::TrajectorySpline &>
{
public:
  explicit PathEndsAtMatcher(
    const ateam_path_planning::Pose & target,
    double position_threshold = ateam_geometry::kDistanceEpsilon,
    double heading_threshold = ateam_geometry::kAngleEpsilon)
  : target_(target), position_threshold_(position_threshold), heading_threshold_(heading_threshold)
  {
  }

  bool MatchAndExplain(
    const ateam_path_planning::TrajectorySpline & trajectory,
    ::testing::MatchResultListener * listener) const override
  {
    const auto & end_segment = trajectory.segments.back();
    *listener << "ends at (" << end_segment.target.position.x() << ", " <<
      end_segment.target.position.y() << ", " << end_segment.target.heading << ')';
    return ateam_geometry::nearEqual(end_segment.target.position, target_.position,
      position_threshold_) &&
           std::abs(angles::shortest_angular_distance(end_segment.target.heading,
      target_.heading)) < heading_threshold_;
  }

  void DescribeTo(std::ostream * os) const override
  {
    *os << "ends at (" << target_.position.x() << ", " << target_.position.y() << ", "
        << target_.heading << ")";
  }

  void DescribeNegationTo(std::ostream * os) const override
  {
    *os << "does not end at (" << target_.position.x() << ", " << target_.position.y() << ", "
        << target_.heading << ")";
  }

private:
  const ateam_path_planning::Pose target_;
  const double position_threshold_;
  const double heading_threshold_;
};

inline ::testing::Matcher<const ateam_path_planning::TrajectorySpline &> PathEndsAt(
  const ateam_path_planning::Pose & target,
  double position_threshold = ateam_geometry::kDistanceEpsilon,
  double heading_threshold = ateam_geometry::kAngleEpsilon)
{
  return ::testing::MakeMatcher(new PathEndsAtMatcher(target, position_threshold,
    heading_threshold));
}

#endif  // UTILS__PATH_ENDS_AT_HPP_
