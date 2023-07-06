#ifndef PATH_PLANNING__PATH_PLANNER_HPP_
#define PATH_PLANNING__PATH_PLANNER_HPP_

#include <ateam_geometry/types.hpp>
#include "types/world.hpp"

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
  bool avoid_ball = false;

  /**
   * @brief The size by which the radius of the robot will be augmented during collision checking
   *
   */
  double footprint_inflation = 0.05;

  double collision_check_resolution = 0.05;
};

class PathPlanner
{
public:
  using Position = ateam_geometry::Point;
  using Path = std::vector<Position>;

  PathPlanner();

  Path getPath(
    const Position & start, const Position & goal, const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles,
    const PlannerOptions & options = PlannerOptions());

private:
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

};

}  // namespace ateam_kenobi::path_planning;

#endif  // PATH_PLANNING__PATH_PLANNER_HPP_
