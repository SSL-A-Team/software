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
#include <map>
#include <ateam_common/km_assignment.hpp>

namespace ateam_kenobi::play_helpers
{

std::vector<std::optional<Robot>> assignRobots(
  const std::vector<Robot> & available_robots,
  const std::vector<ateam_geometry::Point> & positions,
  const std::vector<std::vector<int>> & disallowed_robot_ids)
{
  assert(disallowed_robot_ids.empty() || disallowed_robot_ids.size() == positions.size());

  std::vector<std::optional<Robot>> assignments(positions.size(), std::nullopt);

  if (available_robots.empty() || positions.empty()) {
    return assignments;
  }

  Eigen::MatrixXd costs = Eigen::MatrixXd::Constant(
    available_robots.size(),
    positions.size(), std::numeric_limits<double>::max());
  for (size_t i = 0; i < available_robots.size(); i++) {
    for (size_t j = 0; j < positions.size(); j++) {
      costs(i, j) = sqrt(CGAL::squared_distance(positions.at(j), available_robots.at(i).pos));
    }
  }

  std::map<int, std::vector<int>> forbidden;

  for (auto disallowed_task = 0ul; disallowed_task < disallowed_robot_ids.size();
    ++disallowed_task)
  {
    const auto & ids = disallowed_robot_ids[disallowed_task];
    for (auto id_ind = 0ul; id_ind < ids.size(); ++id_ind) {
      const auto robot_iter =
        std::ranges::find_if(
        available_robots, [target_id = ids[id_ind]](const Robot & r) {
          return r.id == target_id;
        });
      if (robot_iter == available_robots.end()) {
        continue;
      }
      const auto robot_ind = std::distance(available_robots.begin(), robot_iter);
      forbidden[robot_ind].push_back(disallowed_task);
    }
  }

  const auto assigned_tasks = ateam_common::km_assignment::km_assignment(
    costs,
    ateam_common::km_assignment::AssignmentType::MinCost,
    forbidden);

  for (auto robot_ind = 0ul; robot_ind < assigned_tasks.size(); ++robot_ind) {
    const auto task_ind = assigned_tasks[robot_ind];
    if (task_ind >= 0) {
      assignments.at(task_ind) = available_robots[robot_ind];
    }
  }

  return assignments;
}

}  // namespace ateam_kenobi::play_helpers
