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

#ifndef ATEAM_GEOMETRY_TESTING__POINTS_ARE_NEAR_HPP_
#define ATEAM_GEOMETRY_TESTING__POINTS_ARE_NEAR_HPP_

#include <gmock/gmock.h>
#include <CGAL/squared_distance_2.h>
#include <tuple>
#include <ateam_geometry/types.hpp>
#include <ateam_geometry/comparisons.hpp>

// Style deviations in this file to match Google Test styling

class PointsAreNearMatcher : public ::testing::MatcherInterface<std::tuple<ateam_geometry::Point,
    ateam_geometry::Point>>
{
public:
  explicit PointsAreNearMatcher(double threshold = 0.01)
  : threshold_(threshold) {}

  bool MatchAndExplain(
    std::tuple<ateam_geometry::Point, ateam_geometry::Point> points,
    ::testing::MatchResultListener *) const override
  {
    return ateam_geometry::nearEqual(std::get<0>(points), std::get<1>(points), threshold_);
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
  const double threshold_;
};

inline ::testing::Matcher<std::tuple<ateam_geometry::Point, ateam_geometry::Point>> PointsAreNear(
  double threshold = 0.01)
{
  return ::testing::MakeMatcher<std::tuple<ateam_geometry::Point, ateam_geometry::Point>>(
    new PointsAreNearMatcher(
      threshold));
}

#endif  // ATEAM_GEOMETRY_TESTING__POINTS_ARE_NEAR_HPP_
