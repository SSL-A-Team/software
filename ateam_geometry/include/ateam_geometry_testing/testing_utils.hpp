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

#ifndef ATEAM_GEOMETRY_TESTING__TESTING_UTILS_HPP_
#define ATEAM_GEOMETRY_TESTING__TESTING_UTILS_HPP_

#include <gmock/gmock.h>
#include <CGAL/squared_distance_2.h>
#include <tuple>
#include <ateam_geometry/types.hpp>

class PointIsNearMatcher : public ::testing::MatcherInterface<ateam_geometry::Point>
{
public:
  explicit PointIsNearMatcher(const ateam_geometry::Point target, double threshold = 0.01)
  : target_(target), threshold_squared_(threshold * threshold) {}

  bool MatchAndExplain(ateam_geometry::Point p, ::testing::MatchResultListener *) const override
  {
    return CGAL::squared_distance(p, target_) < threshold_squared_;
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
  const double threshold_squared_;
};

inline ::testing::Matcher<ateam_geometry::Point> PointIsNear(
  const ateam_geometry::Point & target,
  double threshold = 0.01)
{
  return ::testing::MakeMatcher(new PointIsNearMatcher(target, threshold));
}

class PointsAreNearMatcher : public ::testing::MatcherInterface<std::tuple<ateam_geometry::Point,
    ateam_geometry::Point>>
{
public:
  explicit PointsAreNearMatcher(double threshold = 0.01)
  : threshold_squared_(threshold * threshold) {}

  bool MatchAndExplain(
    std::tuple<ateam_geometry::Point, ateam_geometry::Point> points,
    ::testing::MatchResultListener *) const override
  {
    return CGAL::squared_distance(std::get<0>(points), std::get<1>(points)) < threshold_squared_;
  }

  void DescribeTo(std::ostream * os) const override
  {
    *os << "are near";
  }

  void DescribeNegationTo(std::ostream * os) const override
  {
    *os << "are not near";
  }

private:
  const double threshold_squared_;
};

inline ::testing::Matcher<std::tuple<ateam_geometry::Point, ateam_geometry::Point>> PointsAreNear(
  double threshold = 0.01)
{
  return ::testing::MakeMatcher<std::tuple<ateam_geometry::Point, ateam_geometry::Point>>(
    new PointsAreNearMatcher(
      threshold));
}

#endif  // ATEAM_GEOMETRY_TESTING__TESTING_UTILS_HPP_
