#ifndef PATH_PLANNING__PATH_PLANNER_HPP_
#define PATH_PLANNING__PATH_PLANNER_HPP_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ateam_geometry/types.hpp>
#include "types/world.hpp"

namespace ateam_kenobi::path_planning
{

struct RrtOptions
{
  /**
   * @brief Max time before planner will give up searching for a path
   */
  double search_time_limit = 2e-3;  // seconds

  /**
   * @brief Max time the planner will spend trying to simplify the path
   *
   */
  double simplification_time_limit = 1e-3;  // seconds

  /**
   * @brief Length of segments in the RRT tree
   */
  double step_size = 0.1;  // meters

  /**
   * @brief If true, the planner treats the ball as an obstacle.
   */
  bool avoid_ball = false;

  /**
   * @brief The size by which the radius of the robot will be augmented during collision checking
   *
   */
  double footprint_inflation = 0.05;
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
    const RrtOptions & options = RrtOptions());

private:
  std::shared_ptr<ompl::base::SE2StateSpace> state_space_;
  ompl::geometric::SimpleSetup simple_setup_;
  ompl::base::PlannerPtr planner_;

  bool isStateValid(
    const ompl::base::State * state,
    const std::vector<ateam_geometry::AnyShape> & obstacles,
    const RrtOptions & options);

  Path convertOmplPathToGeometryPoints(ompl::geometric::PathGeometric & path);

};

}  // namespace ateam_kenobi::path_planning;

#endif  // PATH_PLANNING__PATH_PLANNER_HPP_
