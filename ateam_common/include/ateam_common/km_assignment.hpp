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
#ifndef ATEAM_COMMON__KM_ASSIGNMENT_HPP_
#define ATEAM_COMMON__KM_ASSIGNMENT_HPP_

#include <Eigen/Dense>
#include <vector>

namespace ateam_common::km_assignment
{
/*
  Ensure the input cost matrix is non-negative. If not "is_max_cost",
  multiply the matrix by -1 so the maximum is a minimum and vice versa
  (since this implementation of the Hungarian algorithm finds the
  MAX value matching).

  Returns a new version of the input matrix scaled from 0 to inf.
*/
Eigen::MatrixXd scale_cost_matrix(
  const Eigen::MatrixXd & cost_matrix,
  bool is_max_cost);

/*
  Ensure the input cost matrix is square. If needed, add rows or
  columns of a constant cost below the minimum, so they will
  only be chosen if there are an excess of actors to assign to tasks
  (excess of rows compared to columns).

  Returns a new matrix, containing the original n x m matrix and any
  added rows or columns of 0s starting at n + 1 / m + 1 needed to make
  the matrix square.
*/
Eigen::MatrixXd make_square_cost_matrix(
  const Eigen::MatrixXd & cost_matrix);

/*
  Replace any nan costs in the cost matrix with 0s.
*/
Eigen::MatrixXd replace_nan_costs_with_zeros(
  const Eigen::MatrixXd & matrix);

/*
  Replace any nan costs in the cost matrix with a specified value
*/
Eigen::MatrixXd replace_nan_costs_with_value(
  const Eigen::MatrixXd & matrix,
  double value);

Eigen::MatrixXd replace_forbidden_costs_with_zeros(
  const Eigen::MatrixXd & matrix,
  std::vector<int> forbidden_x, 
  std::vector<int> forbidden_y);
/*
Calculate slack used to update the labeling (weight) of nodes.

- Initialized before starting the algorithm.
- Called every time we add an augmenting path.
- Called after each time we update labels using the minimum slack
value.
*/
void compute_slack(
  const int x,
  std::vector<double> & slack,
  std::vector<double> & slackx,
  const Eigen::MatrixXd & cost,
  const Eigen::VectorXd & lx,
  const Eigen::VectorXd & ly
);

/*
  Find the maximum cost assignment of a bipartite graph (i.e. robots and field positions)
  using the Hungarian (Kuhn-Munkres) Maximum Matching Algorithm.

  You can find the min cost assignment by setting "max_cost" to false.

  Returns a vector of the assigned column index (task) for each row (agent) in the matrix.

  ----------------------------------------------------------------------

  This implementation is largely based off of the one in dlib but allows
  us to use our own types and types from Eigen rather than the dlib ones
  as well as use fp (and not just int) values.
  https://github.com/davisking/dlib/blob/master/dlib/optimization/max_cost_assignment.h

  This resource can be used for reference:
  (two different links, but the content is identical)
      https://cse.hkust.edu.hk/~golin/COMP572/Notes/Matching.pdf
      https://www.columbia.edu/~cs2035/courses/ieor6614.S16/GolinAssignmentNotes.pdf

  This resource is a bit more clear and has a nice written C implementation
  but I unfortunately found it after the dlib implementation:
    https://cp-algorithms.com/graph/hungarian-algorithm.html#implementation-of-the-hungarian-algorithm
*/
std::vector<int> max_cost_assignment(
  const Eigen::MatrixXd & cost_matrix,
  bool max_cost = true,
  std::vector<int> forbidden_x = std::vector<int>(),
  std::vector<int> forbidden_y = std::vector<int>()
);

std::vector<int> min_cost_assignment(
  const Eigen::MatrixXd & cost_matrix,
  std::vector<int> forbidden_x = std::vector<int>(),
  std::vector<int> forbidden_y = std::vector<int>()
);

}  // namespace ateam_common::km_assignment
#endif  // ATEAM_COMMON__KM_ASSIGNMENT_HPP_
