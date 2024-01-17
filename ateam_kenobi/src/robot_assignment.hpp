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


#ifndef ROBOT_ASSIGNMENT_HPP_
#define ROBOT_ASSIGNMENT_HPP_

#include <cmath>

#include <algorithm>
#include <map>
#include <set>
#include <utility>
#include <vector>
#include <iostream>
#include <limits>
#include <unordered_map>

#include <ateam_geometry/types.hpp>

#include "types/robot.hpp"
#include "ateam_common/assignment.hpp"

namespace ateam_kenobi::robot_assignment
{

// #define USE_HACKY_ASSIGNMENT

inline std::unordered_map<size_t, size_t> assign(
  const std::vector<Robot> & available_robots,
  const std::vector<ateam_geometry::Point> & goal_positions)
{
#ifdef USE_HACKY_ASSIGNMENT

  /*
   * Hacky version that just assigns robots in order
   */
  std::unordered_map<size_t, size_t> assignments;
  auto num_assignments = std::min(available_robots.size(), goal_positions.size());
  for (auto index = 0ul; index < num_assignments; ++index) {
    assignments[available_robots[index].id] = index;
  }
  return assignments;

#else

  /*
   * Hungarian-algorithm-based assignment implementation
   */
  Eigen::MatrixXd costs = Eigen::MatrixXd::Constant(
    available_robots.size(),
    goal_positions.size(), 0);  // NOT INFINITY
  for (size_t i = 0; i < available_robots.size(); i++) {
    for (size_t j = 0; j < goal_positions.size(); j++) {
      costs(i, j) = sqrt(CGAL::squared_distance(available_robots.at(i).pos, goal_positions.at(j)));
    }
  }

  // row to column matching assignments
  std::unordered_map assignment = ateam_common::assignment::optimize_assignment(costs);

  std::unordered_map<size_t, size_t> original_indexes_map {};
  for (auto & grid_assignment : assignment) {
    original_indexes_map.emplace(
      available_robots.at(
        grid_assignment.first).id, grid_assignment.second);
  }
  // Map of original robot indexes (1-16) and what of the n goals each is assigned to
  return original_indexes_map;

#endif
}

}  // namespace ateam_kenobi::robot_assignment

#endif  // ROBOT_ASSIGNMENT_HPP_
