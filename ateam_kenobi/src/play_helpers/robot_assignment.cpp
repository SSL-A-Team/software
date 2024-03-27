// Copyright 2024 A Team
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


#include "robot_assignment.hpp"

#include <algorithm>
#include <limits>

namespace ateam_kenobi::play_helpers
{

std::vector<std::optional<Robot>> assignRobots(
  const std::vector<Robot> & available_robots,
  const std::vector<ateam_geometry::Point> & positions,
  const std::vector<std::vector<int>> & disallowed_robot_ids)
{
  assert(disallowed_robot_ids.empty() || disallowed_robot_ids.size() == positions.size());

  std::vector<std::optional<Robot>> assignments(available_robots.size(), std::nullopt);

  if (available_robots.empty() || positions.empty()) {
    return assignments;
  }

  Eigen::MatrixXd costs = Eigen::MatrixXd::Constant(
    positions.size(),
    available_robots.size(), std::numeric_limits<double>::max());
  for (size_t i = 0; i < positions.size(); i++) {
    for (size_t j = 0; j < available_robots.size(); j++) {
      costs(i, j) = sqrt(CGAL::squared_distance(available_robots.at(j).pos, positions.at(i)));
      if (!disallowed_robot_ids.empty()) {
        const auto is_disallowed =
          std::ranges::find(
          disallowed_robot_ids[i],
          available_robots[j].id) != disallowed_robot_ids[i].end();
        if (is_disallowed) {
          costs(i, j) = std::numeric_limits<double>::max();
        }
      }
    }
  }

  const auto num_assignments = std::min(available_robots.size(), positions.size());

  for (auto row = 0ul; row < num_assignments; ++row) {
    Eigen::MatrixXd::Index min_index;
    if (costs.row(row).minCoeff(&min_index) == std::numeric_limits<double>::max()) {
      // Don't assign 'infinite' cost robots.
      // Should only be hit if no bots are available.
      continue;
    }
    assignments[row] = available_robots[min_index];
    costs.col(min_index).fill(std::numeric_limits<double>::max());
  }

  return assignments;
}

}  // namespace ateam_kenobi::play_helpers
