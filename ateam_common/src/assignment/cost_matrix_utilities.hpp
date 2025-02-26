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

#ifndef ASSIGNMENT__COST_MATRIX_UTILITIES_HPP_
#define ASSIGNMENT__COST_MATRIX_UTILITIES_HPP_


#include <Eigen/Dense>
#include <map>
#include <vector>
#include "ateam_common/assignment/assign_tasks.hpp"

namespace ateam_common::assignment
{

/*
  Ensure the input cost matrix is non-negative. If not "is_max_cost",
  multiply the matrix by -1 so the maximum is a minimum and vice versa
  (since this implementation of the Hungarian algorithm finds the
  MAX value matching).

  Returns a new version of the input matrix scaled from 0 to inf.
*/
Eigen::MatrixXd ScaleCostMatrix(
  const Eigen::MatrixXd & cost_matrix,
  AssignmentType max_or_min);

/*
  Ensure the input cost matrix is square. If needed, add rows or
  columns of a constant cost below the minimum, so they will
  only be chosen if there are an excess of actors to assign to tasks
  (excess of rows compared to columns).

  Returns a new matrix, containing the original n x m matrix and any
  added rows or columns of 0s starting at n + 1 / m + 1 needed to make
  the matrix square.
*/
Eigen::MatrixXd MakeSquareCostMatrix(
  const Eigen::MatrixXd & cost_matrix);

/*
  Replace any nan costs in the cost matrix with 0s.
*/
Eigen::MatrixXd ReplaceNanCostsWithZeros(
  const Eigen::MatrixXd & matrix);

/*
  Replace any nan costs in the cost matrix with a specified value
*/
Eigen::MatrixXd ReplaceNanCostsWithValue(
  const Eigen::MatrixXd & matrix,
  double value);

/*
  Replace costs of forbidden assignments with zeros AFTER scaling the cost matrix
  to ensure the tasks are not assigned.

  The forbidden_assignments map agents (rows) to vectors of forbidden tasks (a vector of column indices).
*/
Eigen::MatrixXd ReplaceForbiddenCostsWithZeros(
  const Eigen::MatrixXd & matrix,
  std::map<int, std::vector<int>> forbidden_assignments);

Eigen::MatrixXd ReplaceForbiddenCostsWithValue(
  const Eigen::MatrixXd & matrix,
  std::map<int, std::vector<int>> forbidden_assignments,
  double value);

}  // namespace ateam_common::assignment

#endif  // ASSIGNMENT__COST_MATRIX_UTILITIES_HPP_
