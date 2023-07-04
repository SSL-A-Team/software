
#ifndef ATEAM_KENOBI__ROBOT_ASSIGNMENT_HPP_
#define ATEAM_KENOBI__ROBOT_ASSIGNMENT_HPP_

#include <cmath>

#include <algorithm>
#include <map>
#include <set>
#include <utility>
#include <vector>
#include <iostream>
#include <limits>

#include "types/robot.hpp"
#include "ateam_common/assignment.hpp"

namespace ateam_kenobi::robot_assignment
{

std::unordered_map<size_t, size_t> assign(const std::vector<Robot> & available_robots, const std::vector<ateam_geometry::Point> & goal_positions)
{
  Eigen::MatrixXd costs = Eigen::MatrixXd::Constant(
    available_robots.size(),
    goal_positions.size(), std::numeric_limits<double>::max());  // NOT INFINITY
  for (size_t i = 0; i < available_robots.size(); i++) {
    for (size_t j = 0; j < goal_positions.size(); j++) {
      costs(i, j) = sqrt(cgal::squared_distance(available_robots.at(i).pos(), goal_positions.at(j)));
    }
  }

  // row to column matching assignments
  std::unordered_map assignment = ateam_common::assignment::optimize_assignment(costs);

  std::map<Robot> original_indexes_map {};
  for(size_t i = 0; i < assignment.size(); i++) {
    auto grid_assignment = assignment.at(i);
    original_indexes_map.emplace_back(available_robots.at(grid_assignment->first).id, grid_assignment->second);
  }
  // Map of original robot indexes (1-16) and what of the n goals each is assigned to
  return original_indexes_map;
}

}  // namespace ateam_kenobi::robot_assignment

#endif  // ATEAM_KENOBI__ROBOT_ASSIGNMENT_HPP_
