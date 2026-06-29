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

#ifndef UTILS__PATHS_COLLIDE_HPP_
#define UTILS__PATHS_COLLIDE_HPP_

#include <gmock/gmock.h>
#include <algorithm>
#include <array>
#include <optional>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/types.hpp>
#include <ateam_geometry/creation_helpers.hpp>
#include <ateam_geometry/do_intersect.hpp>
#include "ateam_path_planning/trajectory_spline.hpp"

class PathsCollideMatcher
  : public ::testing::MatcherInterface<
    const std::array<std::optional<ateam_path_planning::PathPlanResult>, 16> &>
{
public:
  explicit PathsCollideMatcher(const double resolution = 0.1, const double horizon = 3.0)
  : resolution_(resolution), horizon_(horizon)
  {
  }

  bool MatchAndExplain(
    const std::array<std::optional<ateam_path_planning::PathPlanResult>, 16> & results,
    ::testing::MatchResultListener * listener) const override
  {
    std::vector<std::vector<ateam_geometry::Point>> path_points;
    size_t max_path_length = 0;
    for (const auto & result : results) {
      if (!result.has_value()) {
        continue;
      }
      path_points.push_back(result->path.ToPoints(resolution_));
      max_path_length = std::max(max_path_length, path_points.back().size());
    }
    const auto horizon_path_length = static_cast<size_t>(horizon_ / resolution_);
    const auto loop_limit = std::min(max_path_length, horizon_path_length);
    for (auto i = 0ul; i < loop_limit; ++i) {
      std::vector<std::optional<ateam_geometry::Disk>> disks;
      for (const auto & path : path_points) {
        if(path.size() > i) {
          disks.push_back(ateam_geometry::makeDisk(path[i], kRobotRadius));
        } else {
          disks.push_back(std::nullopt);
        }
      }
      for(auto a = 0ul; a < disks.size() - 1; ++a) {
        for(auto b = a + 1; b < disks.size(); ++b) {
          if (!disks[a].has_value() || !disks[b].has_value()) {
            continue;
          }
          if (ateam_geometry::doIntersect(*disks[a], *disks[b])) {
            const auto collision_time = i * resolution_;
            const auto distance = CGAL::approximate_sqrt(CGAL::squared_distance(disks[a]->center(),
              disks[b]->center()));
            *listener << "paths " << a << " and " << b << " collide at " << collision_time <<
              "s. Positions: " << disks[a]->center() << " and " << disks[b]->center() <<
              " (Distance: " << distance << ")  (Time: " << (i * resolution_) << ')';
            return true;
          }
        }
      }
    }

    *listener << "paths do not collide";
    return false;
  }

  void DescribeTo(std::ostream * os) const override
  {
    *os << "paths collide";
  }

  void DescribeNegationTo(std::ostream * os) const override
  {
    *os << "paths do not collide";
  }

private:
  const double resolution_;
  const double horizon_;
};

inline ::testing::Matcher<const std::array<std::optional<ateam_path_planning::PathPlanResult>,
  16> &> PathsCollide(
  double resolution = 0.1, double horizon = 3.0)
{
  return ::testing::MakeMatcher(new PathsCollideMatcher(resolution, horizon));
}

#endif  // UTILS__PATHS_COLLIDE_HPP_
