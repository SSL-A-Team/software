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


#ifndef PATH_PLANNING__PATH_PLANNER_HPP_
#define PATH_PLANNING__PATH_PLANNER_HPP_

#include <vector>
#include <ateam_geometry/types.hpp>
#include "types/world.hpp"
#include "visualization/overlays.hpp"

namespace ateam_kenobi::path_planning
{

struct PlannerOptions
{
  /**
   * @brief Max time before planner will give up searching for a path
   */
  double search_time_limit = 2e-3;  // seconds

  /**
   * @brief If true, the planner treats the ball as an obstacle.
   */
  bool avoid_ball = true;

  /**
   * @brief The size by which the radius of the robot will be augmented during collision checking
   *
   */
  double footprint_inflation = 0.06;

  double collision_check_resolution = 0.05;

  bool use_default_obstacles = true;

  /**
   * @brief If true, any obstacles touching the start point will be ignored for all planning.
   *
   * Useful if you want to plan a path to escape a virtual obstacle like a keep out zone.
   */
  bool ignore_start_obstacle = true;

  bool draw_obstacles = false;
};

class PathPlanner
{
public:
  using Position = ateam_geometry::Point;
  using Path = std::vector<Position>;

  PathPlanner(visualization::Overlays overlays = {});

  Path getPath(
    const Position & start, const Position & goal, const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options = PlannerOptions());

private:
  visualization::Overlays overlays_;

  void removeCollidingObstacles(
    std::vector<ateam_geometry::AnyShape> & obstacles,
    const ateam_geometry::Point & point, const PlannerOptions & options);

  bool isStateInBounds(const ateam_geometry::Point & state, const World & world);

  bool isStateValid(
    const ateam_geometry::Point & state,
    const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options);

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

  void addRobotsToObstacles(
    const World & world, const ateam_geometry::Point & start_pos,
    std::vector<ateam_geometry::AnyShape> & obstacles);

  void addDefaultObstacles(const World & world, std::vector<ateam_geometry::AnyShape> & obstacles);

  void removeSkippablePoints(
    Path & path, const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options);

  void removeLoops(Path & path);

  void trimPathAfterCollision(
    Path & path, const World & world,
    std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options);

  void drawObstacles(const std::vector<ateam_geometry::AnyShape> & obstacles);
};

}  // namespace ateam_kenobi::path_planning

#endif  // PATH_PLANNING__PATH_PLANNER_HPP_
