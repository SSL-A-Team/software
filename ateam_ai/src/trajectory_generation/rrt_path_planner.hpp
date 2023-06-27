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

#ifndef TRAJECTORY_GENERATION__RRT_PATH_PLANNER_HPP_
#define TRAJECTORY_GENERATION__RRT_PATH_PLANNER_HPP_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include <ateam_geometry/types.hpp>
#include "types/world.hpp"

namespace ateam_ai::trajectory_generation
{

class RrtPathPlanner
{
public:
  using Position = Eigen::Vector2d;
  using Path = std::vector<Position>;

  RrtPathPlanner();

  Path generatePath(
    const World & world, const std::vector<ateam_geometry::AnyShape> & obstacles,
    const Position & start_pos, const Position & goal_pos);

private:
  std::shared_ptr<ompl::base::SE2StateSpace> state_space_;
  ompl::geometric::SimpleSetup simple_setup_;
  ompl::base::PlannerPtr planner_;

  bool isStateValid(
    const ompl::base::State * state,
    const std::vector<ateam_geometry::AnyShape> & obstacles);

  Path convertOmplPathToEigen(ompl::geometric::PathGeometric & path);
};

}  // namespace ateam_ai::trajectory_generation

#endif  // TRAJECTORY_GENERATION__RRT_PATH_PLANNER_HPP_
