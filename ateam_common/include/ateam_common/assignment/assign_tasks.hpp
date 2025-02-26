// Copyright 2025 A Team
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

#ifndef ATEAM_COMMON__ASSIGNMENT__ASSIGN_TASKS_HPP_
#define ATEAM_COMMON__ASSIGNMENT__ASSIGN_TASKS_HPP_

#include <Eigen/Dense>
#include <vector>
#include <map>

namespace ateam_common::assignment
{

enum class AssignmentType
{
  MaxCost,
  MinCost
};

/**
 * @brief Find the optimal assignment of a bipartite graph (i.e. robots and field positions)
 * represented by a cost matrix.
 *
 * @param cost_matrix Costs of assigning an agent (row) to a task (col)
 * @param max_or_min
 * @param forbidden_assignments Maps an agent to a set of tasks that agent cannot be assigned to
 * @return std::vector<int> A vector of the assigned task (col) for each agent (row)
 */
std::vector<int> AssignTasks(
  const Eigen::MatrixXd & cost_matrix,
  AssignmentType max_or_min = AssignmentType::MaxCost,
  std::map<int, std::vector<int>> forbidden_assignments = std::map<int, std::vector<int>>()
);

}  // namespace ateam_common::assignment

#endif  // ATEAM_COMMON__ASSIGNMENT__ASSIGN_TASKS_HPP_
