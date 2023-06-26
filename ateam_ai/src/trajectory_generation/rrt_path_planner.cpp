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


/* 
  Paths forward:
  - Treat other robots as obstacles
  - Allow for passing in additional obstacle shapes
  - Allow option for "robot faces along path"
  - 

*/

namespace ateam_ai::trajectory_generation
{
RrtPathPlanner::RrtPathPlanner()
: state_space_(std::make_shared<ompl::base::SE2StateSpace>()),
  simple_setup_(state_space_),
  planner_(std::make_shared<ompl::geometric::RRTConnect>(simple_setup_.getSpaceInformation()))
{
  ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
  simple_setup_.setPlanner(planner_);
  simple_setup_.setStateValidityChecker(
    [this](const ompl::base::State * s) {
      return isStateValid(s);
    });
}

Trajectory RrtPathPlanner::generatePath(
  const World & /*world*/, const Eigen::Vector3d & start_pos, 
  const Eigen::Vector3d & goal_pos)
{
  using ScopedState = ompl::base::ScopedState<ompl::base::SE2StateSpace>;

  ScopedState start(state_space_);
  start->setXY(start_pos.x(), start_pos.y());
  start->setYaw(start_pos.z());

  ScopedState goal(state_space_);
  goal->setXY(goal_pos.x(), goal_pos.y());
  goal->setYaw(goal_pos.z());

  simple_setup_.setStartAndGoalStates(start, goal);

  ompl::base::RealVectorBounds bounds(2);
  bounds.setLow(-10);
  bounds.setHigh(10);
  state_space_->setBounds(bounds);

  // TODO(barulicm) set state space bounds based on field boundaries?

  auto planner_status = simple_setup_.solve(1.0 /* time limit in s */);

  if (!planner_status) {
    // no solution found
    return {};
  }

  simple_setup_.simplifySolution();

  return convertOmplPathToTrajectory(simple_setup_.getSolutionPath());
}

bool RrtPathPlanner::isStateValid(const ompl::base::State * /*state*/)
{
  // TODO(barulicm) actually check state validity
  /* TODO(barulicm) how to get World object into this without extra copying? Maybe add parameter
   * and bind w/ lambda
   */
  return true;
}

Trajectory RrtPathPlanner::convertOmplPathToTrajectory(ompl::geometric::PathGeometric & path)
{
  using State = ompl::base::SE2StateSpace::StateType;

  Trajectory trajectory;
  const auto & states = path.getStates();
  trajectory.samples.reserve(states.size());
  std::transform(
    states.begin(), states.end(), std::back_inserter(trajectory.samples), [](
      ompl::base::State * s) {
      auto state = s->as<State>();
      Sample3d sample;
      sample.pose.x() = state->getX();
      sample.pose.y() = state->getY();
      sample.pose.z() = state->getYaw();
      return sample;
    });
  return trajectory;
}

}  // namespace ateam_ai::trajectory_generation
