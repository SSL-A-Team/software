#include "path_planner.hpp"
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/Console.h>
#include <ranges>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/ateam_geometry.hpp>

namespace ateam_kenobi::path_planning
{

PathPlanner::PathPlanner()
: state_space_(std::make_shared<ompl::base::SE2StateSpace>()),
  simple_setup_(state_space_),
  planner_(std::make_shared<ompl::geometric::RRT>(simple_setup_.getSpaceInformation()))
{
  // TODO connect to ros logging
  ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
  simple_setup_.setPlanner(planner_);
}

PathPlanner::Path PathPlanner::getPath(
  const Position & start, const Position & goal, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles,
  const RrtOptions & options)
{
  using ScopedState = ompl::base::ScopedState<ompl::base::SE2StateSpace>;

  simple_setup_.clear();

  planner_->as<ompl::geometric::RRT>()->setRange(options.step_size);

  std::vector<ateam_geometry::AnyShape> obstacles_and_robots(obstacles);
  auto obstacle_from_robot = [](const std::optional<Robot> & robot) {
      return ateam_geometry::AnyShape(
        ateam_geometry::makeCircle(robot.value().pos, kRobotRadius));
    };

  auto not_current_robot = [&start](const std::optional<Robot> & robot) {
      // Assume any robot close enough to the start pos is the robot trying to navigate
      return (robot.value().pos - start).squared_length() > std::pow(kRobotRadius,2);
    };

  auto our_robot_obstacles = world.our_robots |
    std::views::filter(std::mem_fn(&std::optional<Robot>::has_value)) |
    std::views::filter(not_current_robot) |
    std::views::transform(obstacle_from_robot);
  obstacles_and_robots.insert(
    obstacles_and_robots.end(),
    our_robot_obstacles.begin(),
    our_robot_obstacles.end());

  auto their_robot_obstacles = world.their_robots |
    std::views::filter(std::mem_fn(&std::optional<Robot>::has_value)) |
    std::views::transform(obstacle_from_robot);
  obstacles_and_robots.insert(
    obstacles_and_robots.end(),
    their_robot_obstacles.begin(),
    their_robot_obstacles.end());

  if(options.avoid_ball) {
    obstacles_and_robots.push_back(ateam_geometry::makeCircle(world.ball.pos, 0.04267/2));
  }

  ScopedState start_state(state_space_);
  start_state->setXY(start.x(), start.y());

  ScopedState goal_state(state_space_);
  goal_state->setXY(goal.x(), goal.y());

  simple_setup_.setStartAndGoalStates(start_state, goal_state);

  ompl::base::RealVectorBounds bounds(2);
  bounds.setLow(0, (-0.5 * world.field.field_length) - world.field.boundary_width);
  bounds.setHigh(0, (0.5 * world.field.field_length) + world.field.boundary_width);
  bounds.setLow(1, (-0.5 * world.field.field_width) - world.field.boundary_width);
  bounds.setHigh(1, (0.5 * world.field.field_width) + world.field.boundary_width);
  auto bounds_sizes = bounds.getDifference();
  // arbitrary threshold to make sure field info is valid
  if(bounds_sizes[0] < 0.1 || bounds_sizes[1] < 0.1) {
    // No path exists because field size is not valid
    return {};
  }
  state_space_->setBounds(bounds);

  simple_setup_.setStateValidityChecker(
    [this, &obstacles_and_robots, &options](const ompl::base::State * s) {
      return isStateValid(s, obstacles_and_robots, options);
    });

  auto planner_status = simple_setup_.solve(options.search_time_limit);

  if (planner_status != ompl::base::PlannerStatus::EXACT_SOLUTION) {
    // no solution found
    return {};
  }

  simple_setup_.simplifySolution(options.simplification_time_limit);

  return convertOmplPathToGeometryPoints(simple_setup_.getSolutionPath());
}

bool PathPlanner::isStateValid(
  const ompl::base::State * state,
  const std::vector<ateam_geometry::AnyShape> & obstacles,
  const RrtOptions & options)
{
  const auto * se2_state = state->as<ompl::base::SE2StateSpace::StateType>();
  auto robot_footprint = ateam_geometry::makeCircle(
    ateam_geometry::Point(se2_state->getX(), se2_state->getY()),
    kRobotRadius + options.footprint_inflation
  );

  return std::ranges::none_of(
    obstacles, [&robot_footprint](const ateam_geometry::AnyShape & obstacle) {
      return ateam_geometry::variantDoIntersect(robot_footprint, obstacle);
    });
}

PathPlanner::Path PathPlanner::convertOmplPathToGeometryPoints(ompl::geometric::PathGeometric & path)
{
  using State = ompl::base::SE2StateSpace::StateType;
  Path geometry_path;
  const auto & states = path.getStates();
  geometry_path.reserve(states.size());
  std::transform(states.begin(), states.end(), std::back_inserter(geometry_path), [](ompl::base::State * s) {
    auto state = s->as<State>();
    return Position(state->getX(), state->getY());
  });
  return geometry_path;
}

} // namespace ateam_kenobi::path_planning
