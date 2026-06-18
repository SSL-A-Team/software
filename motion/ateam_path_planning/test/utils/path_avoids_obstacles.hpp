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

#ifndef UTILS__PATH_AVOIDS_OBSTACLES_HPP_
#define UTILS__PATH_AVOIDS_OBSTACLES_HPP_

#include <angles/angles.h>
#include <gmock/gmock.h>
#include <vector>
#include <ateam_geometry/types.hpp>
#include <ateam_geometry/comparisons.hpp>
#include <ateam_geometry/epsilon.hpp>
#include <ateam_geometry/do_intersect.hpp>
#include <ateam_common/robot_constants.hpp>
#include "ateam_path_planning/trajectory_spline.hpp"
#include "ateam_path_planning/obstacle.hpp"

class PathAvoidsObstaclesMatcher
  : public ::testing::MatcherInterface<const ateam_path_planning::TrajectorySpline &>
{
public:
  explicit PathAvoidsObstaclesMatcher(
    const std::vector<ateam_path_planning::Obstacle> & obstacles, const double resolution = 0.01)
  : obstacles_(obstacles), resolution_(resolution)
  {
  }

  bool MatchAndExplain(
    const ateam_path_planning::TrajectorySpline & trajectory,
    ::testing::MatchResultListener * listener) const override
  {
    const auto points = trajectory.ToPoints(resolution_);
    for(const auto & point : points) {
      const auto robot_footprint = ateam_geometry::makeDisk(point, kRobotRadius);
      for (const auto & obstacle : obstacles_) {
        if(ateam_geometry::doIntersect(robot_footprint,
            obstacle.shape))
        {
          *listener << "collides with obstacle: " << obstacle.shape;
          return false;
        }
      }
    }
    *listener << "avoids given obstacles";
    return true;
  }

  void DescribeTo(std::ostream * os) const override
  {
    *os << "avoids given obstacles";
  }

  void DescribeNegationTo(std::ostream * os) const override
  {
    *os << "hits one or more obstacles";
  }

private:
  const std::vector<ateam_path_planning::Obstacle> obstacles_;
  const double resolution_;
};

inline ::testing::Matcher<const ateam_path_planning::TrajectorySpline &> PathAvoidsObstacles(
  const std::vector<ateam_path_planning::Obstacle> & obstacles, const double resolution = 0.01)
{
  return ::testing::MakeMatcher(new PathAvoidsObstaclesMatcher(obstacles, resolution));
}

#endif  // UTILS__PATH_AVOIDS_OBSTACLES_HPP_
