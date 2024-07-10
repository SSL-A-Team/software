// Copyright 2024 A Team
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

#ifndef ATEAM_GEOMETRY_TESTING__POINT_IS_NEAR_HPP_
#define ATEAM_GEOMETRY_TESTING__POINT_IS_NEAR_HPP_

#include <gmock/gmock.h>
#include <CGAL/squared_distance_2.h>
#include <tuple>
#include <ateam_geometry/types.hpp>
#include <ateam_geometry/comparisons.hpp>

// Style deviations in this file to match Google Test styling

class PointIsNearMatcher : public ::testing::MatcherInterface<const ateam_geometry::Point &>
{
public:
  explicit PointIsNearMatcher(const ateam_geometry::Point target, double threshold = 0.01)
  : target_(target), threshold_(threshold) {}

  bool MatchAndExplain(
    const ateam_geometry::Point & p,
    ::testing::MatchResultListener *) const override
  {
    return ateam_geometry::nearEqual(p, target_, threshold_);
  }

  void DescribeTo(std::ostream * os) const override
  {
    *os << "is near (" << target_.x() << ", " << target_.y() << ")";
  }

  void DescribeNegationTo(std::ostream * os) const override
  {
    *os << "is not near (" << target_.x() << ", " << target_.y() << ")";
  }

private:
  const ateam_geometry::Point target_;
  const double threshold_;
};

inline ::testing::Matcher<const ateam_geometry::Point &> PointIsNear(
  const ateam_geometry::Point & target,
  double threshold = 0.01)
{
  return ::testing::MakeMatcher(new PointIsNearMatcher(target, threshold));
}

#endif  // ATEAM_GEOMETRY_TESTING__POINT_IS_NEAR_HPP_
