// Copyright 2021 A Team
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


#ifndef CORE__PATH_PLANNING__PATH_PLANNER_HPP_
#define CORE__PATH_PLANNING__PATH_PLANNER_HPP_

#include <vector>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/any_shape.hpp>
#include <ateam_geometry/types.hpp>
#include "core/types/world.hpp"
#include "core/stp/base.hpp"
#include "path.hpp"
#include "planner_options.hpp"

namespace ateam_kenobi::path_planning
{

class PathPlanner : public stp::Base
{
public:
  explicit PathPlanner(stp::Options stp_options = {});

  Path getPath(
    const Position & start, const Position & goal, const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options = PlannerOptions());

  bool usedCachedPath() const
  {
    return used_cached_path_;
  }

  bool didTimeOut() const
  {
    return timed_out_;
  }

  bool isPathTruncated() const
  {
    return cached_path_truncated_;
  }

private:
  visualization::Overlays overlays_;
  bool cached_path_valid_ = false;
  bool cached_path_truncated_ = false;
  bool timed_out_ = false;
  Path cached_path_;
  Position cached_path_goal_;
  bool used_cached_path_ = false;
  std::chrono::steady_clock::time_point start_time_;

  bool isTimeUp(const PlannerOptions & options);

  void removeCollidingObstacles(
    std::vector<ateam_geometry::AnyShape> & obstacles,
    const ateam_geometry::Point & point, const PlannerOptions & options);

  enum class BoundaryStrategy
  {
    Strict,
    Ignore,
    OffsetIn,
    OffsetOut
  };

  bool isStateValid(
    const ateam_geometry::Point & state,
    const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options, const BoundaryStrategy bounds_strat = BoundaryStrategy::Strict);

  std::optional<ateam_geometry::Point> getCollisionPoint(
    const ateam_geometry::Point & p1, const ateam_geometry::Point & p2,
    const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options);

  struct SplitResult
  {
    bool split_needed;
    bool split_succeeded;
  };

  SplitResult splitSegmentIfNecessary(
    Path & path, const std::size_t ind1, const std::size_t ind2,
    const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options);

  void removeSkippablePoints(
    Path & path, const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options);

  void removeLoops(Path & path);

  void trimPathAfterCollision(
    Path & path, const World & world,
    std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options);

  std::optional<ateam_geometry::Point> findLastCollisionFreePoint(
    const ateam_geometry::Point & start, const ateam_geometry::Point & goal, const World & world,
    std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options);

  void drawObstacles(const std::vector<ateam_geometry::AnyShape> & obstacles);

  bool shouldReplan(
    const Position & start, const Position & goal, const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options);

  void smoothCorners(
    Path & path, const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles, const PlannerOptions & options);

  bool smoothCorner(
    Path & path, const size_t corner_point_index, const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles, const PlannerOptions & options);
};

}  // namespace ateam_kenobi::path_planning

#endif  // CORE__PATH_PLANNING__PATH_PLANNER_HPP_
