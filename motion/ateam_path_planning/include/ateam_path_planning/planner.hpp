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

#ifndef ATEAM_PATH_PLANNING__PLANNER_HPP_
#define ATEAM_PATH_PLANNING__PLANNER_HPP_

#include <array>
#include <optional>
#include <vector>
#include <ateam_game_state/world.hpp>
#include <ateam_geometry/types.hpp>
#include "obstacle.hpp"
#include "pose.hpp"
#include "trajectory_spline.hpp"
#include "planner_options.hpp"
#include "path_plan_result.hpp"

namespace ateam_path_planning
{

class Planner {
public:
  Planner() = default;

  std::array<std::optional<PathPlanResult>, 16> PlanPathsForAllBots(
    const std::array<std::optional<Pose>, 16> & targets, const std::array<uint8_t, 16> & priorities,
    const ateam_game_state::World & world, const std::vector<Obstacle> & global_obstacles,
    const std::array<std::vector<Obstacle>, 16> & per_bot_obstacles,
    const std::array<PlannerOptions, 16> & options = {});

  std::array<std::optional<ateam_geometry::Point>, 16>
  GetExpectedLocations(const std::array<PlannerOptions, 16> & options = {});

private:
  struct CacheEntry
  {
    TrajectorySpline trajectory;
    PlannerOptions options;
    Pose target;
  };

  std::array<std::optional<CacheEntry>, 16> cache_;

  std::optional<PathPlanResult> PlanPath(
    const ateam_game_state::Robot & robot, const Pose & target,
    const std::vector<Obstacle> & obstacles, const PlannerOptions & options,
    const ateam_game_state::World & world);

  bool ShouldReplan(
    const ateam_game_state::Robot & robot, const Pose & target,
    const std::vector<Obstacle> & obstacles, const PlannerOptions & options,
    const ateam_game_state::World & world);

  std::optional<Pose> GetTruncatedTarget(
    const ateam_game_state::Robot & robot, const Pose & target,
    const std::vector<Obstacle> & obstacles, const PlannerOptions & options,
    const ateam_game_state::World & world);

  void RemoveInitialCollidingObstacles(
    std::vector<Obstacle> & obstacles,
    const ateam_game_state::Robot & robot);

  /**
   * Compares candidate paths via multiple metrics.
   * @returns @c std::strong_ordering::less if path_l is worse than path_r,
   * @c std::strong_ordering::greater if path_l is better than path_r, and @c std::strong_ordering::equal otherwise
   */
  std::partial_ordering ComparePaths(
    const TrajectorySpline & path_l, const CollisionStats & stats_l,
    const TrajectorySpline & path_r, const CollisionStats & stats_r);
};
}  // namespace ateam_path_planning

#endif  // ATEAM_PATH_PLANNING__PLANNER_HPP_
