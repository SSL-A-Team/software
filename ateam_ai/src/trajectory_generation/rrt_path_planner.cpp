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

#include "rrt_path_planner.hpp"

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>
#include <ranges>
#include <algorithm>
#include <ateam_geometry/ateam_geometry.hpp>

namespace ateam_ai::trajectory_generation
{
RrtPathPlanner::RrtPathPlanner()
: state_space_(std::make_shared<ompl::base::SE2StateSpace>()),
  simple_setup_(state_space_),
  planner_(std::make_shared<ompl::geometric::RRTConnect>(simple_setup_.getSpaceInformation()))
{
  // ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
  simple_setup_.setPlanner(planner_);
}

RrtPathPlanner::Path RrtPathPlanner::generatePath(
  const World & world, const std::vector<ateam_geometry::AnyShape> & obstacles,
  const Position & start_pos, const Position & goal_pos, const RrtOptions & options)
{
  using ScopedState = ompl::base::ScopedState<ompl::base::SE2StateSpace>;

  simple_setup_.clear();

  planner_->as<ompl::geometric::RRTConnect>()->setRange(options.step_size);

  std::vector<ateam_geometry::AnyShape> obstacles_and_robots(obstacles);
  auto obstacle_from_robot = [](const std::optional<Robot> & robot) {
      return ateam_geometry::AnyShape(
        ateam_geometry::makeCircle(ateam_geometry::EigenToPoint(robot.value().pos), 0.09));
    };

  // auto not_current_robot = [&start_pos](const std::optional<Robot> & robot) {
  //     // Assume any robot close enough to the start pos is the robot trying to navigate
  //     return (robot.value().pos.head(2) - start_pos).norm() > 0.090;  // threshold is robot radius
  //   };

  // auto our_robot_obstacles = world.our_robots |
  //   std::views::filter(std::mem_fn(&std::optional<Robot>::has_value)) |
  //   std::views::filter(not_current_robot) |
  //   std::views::transform(obstacle_from_robot);
  // obstacles_and_robots.insert(
  //   obstacles_and_robots.end(),
  //   our_robot_obstacles.begin(),
  //   our_robot_obstacles.end());

  // auto their_robot_obstacles = world.their_robots |
  //   std::views::filter(std::mem_fn(&std::optional<Robot>::has_value)) |
  //   std::views::transform(obstacle_from_robot);
  // obstacles_and_robots.insert(
  //   obstacles_and_robots.end(),
  //   their_robot_obstacles.begin(),
  //   their_robot_obstacles.end());

  ScopedState start(state_space_);
  start->setXY(start_pos.x(), start_pos.y());

  ScopedState goal(state_space_);
  goal->setXY(goal_pos.x(), goal_pos.y());

  simple_setup_.setStartAndGoalStates(start, goal);

  ompl::base::RealVectorBounds bounds(2);
  bounds.setLow(0, (-0.5 * world.field.field_length) - world.field.boundary_width);
  bounds.setHigh(0, (0.5 * world.field.field_length) + world.field.boundary_width);
  bounds.setLow(1, (-0.5 * world.field.field_width) - world.field.boundary_width);
  bounds.setHigh(1, (0.5 * world.field.field_width) + world.field.boundary_width);
  state_space_->setBounds(bounds);

  simple_setup_.setStateValidityChecker(
    [this, &obstacles_and_robots](const ompl::base::State * s) {
      return isStateValid(s, obstacles_and_robots);
    });

  auto planner_status = simple_setup_.solve(options.time_limit);

  std::cerr << "Planner status: " << planner_status.asString() << '\n';

  if (planner_status != ompl::base::PlannerStatus::EXACT_SOLUTION) {
    // no solution found
    return {};
  }

  simple_setup_.simplifySolution();

  return convertOmplPathToEigen(simple_setup_.getSolutionPath());
}

bool RrtPathPlanner::isStateValid(
  const ompl::base::State * state,
  const std::vector<ateam_geometry::AnyShape> & obstacles)
{
  const auto * se2_state = state->as<ompl::base::SE2StateSpace::StateType>();
  auto robot_footprint = ateam_geometry::makeCircle(
    ateam_geometry::Point(se2_state->getX(), se2_state->getY()),
    0.09
  );

  return std::ranges::none_of(
    obstacles, [&robot_footprint](const ateam_geometry::AnyShape & obstacle) {
      return ateam_geometry::variantDoIntersect(robot_footprint, obstacle);
    });
}

RrtPathPlanner::Path RrtPathPlanner::convertOmplPathToEigen(ompl::geometric::PathGeometric & path)
{
  using State = ompl::base::SE2StateSpace::StateType;

  Path eigen_path;
  const auto & states = path.getStates();
  eigen_path.reserve(states.size());
  std::transform(
    states.begin(), states.end(), std::back_inserter(eigen_path), [](
      ompl::base::State * s) {
      auto state = s->as<State>();
      return Position(state->getX(), state->getY());
    });
  return eigen_path;
}

}  // namespace ateam_ai::trajectory_generation
