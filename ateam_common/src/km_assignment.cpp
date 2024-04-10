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

#include <Eigen/Dense>
#include <limits>
#include <cmath>
#include <vector>
#include <deque>
#include "ateam_common/km_assignment.hpp"

namespace ateam_common::km_assignment
{
const float EPSILON = 1e-10;

Eigen::MatrixXd scale_cost_matrix(
  const Eigen::MatrixXd & matrix, bool is_max_cost)
{
  Eigen::MatrixXd new_matrix = matrix;
  if (!is_max_cost) {
    new_matrix *= -1;
  }
  // If any of the coefficients are NaN, ensure we ignore them using
  // Eigen's NaNPropagation template:
  // https://eigen.tuxfamily.org/index.php?title=3.4#Improvement_to_NaN_propagation
  double matMin = new_matrix.template minCoeff<Eigen::PropagateNumbers>();

  // If there are no negative values, no rescaling needed.
  if (matMin >= 0.0) {
    return new_matrix;
  }
  // Otherwise, shift so the entire matrix is moved to be > 0
  new_matrix -= (Eigen::MatrixXd::Ones(
      new_matrix.rows(), new_matrix.cols()) * matMin);
  return new_matrix;
}

Eigen::MatrixXd make_square_cost_matrix(
  const Eigen::MatrixXd & matrix)
{
  if (matrix.rows() == matrix.cols()) {
    return matrix;
  }

  std::size_t new_dim = std::max(matrix.rows(), matrix.cols());

  //  The value of any non-existant cost is 0, less than any assigable
  //  real cost.
  Eigen::MatrixXd new_mat =
    Eigen::MatrixXd::Constant(new_dim, new_dim, 0);

  new_mat.block(0, 0, matrix.rows(), matrix.cols()) = matrix;

  return new_mat;
}

Eigen::MatrixXd replace_nan_costs_with_zeros(
  const Eigen::MatrixXd & matrix)
{
  Eigen::MatrixXd zero_mask = Eigen::MatrixXd::Zero(matrix.rows(), matrix.cols());
  Eigen::MatrixXd new_matrix = (matrix.array().isNaN()).select(zero_mask, matrix);
  return new_matrix;
}

Eigen::MatrixXd replace_nan_costs_with_value(
  const Eigen::MatrixXd & matrix,
  double value)
{
  Eigen::MatrixXd value_mask = Eigen::MatrixXd::Ones(matrix.rows(), matrix.cols()) * value;
  Eigen::MatrixXd new_matrix = (matrix.array().isNaN()).select(value_mask, matrix);
  return new_matrix;
}

Eigen::MatrixXd replace_forbidden_costs_with_zeros(
  const Eigen::MatrixXd & matrix,
  std::vector<int> forbidden_x,
  std::vector<int> forbidden_y) 
{
  Eigen::MatrixXd new_matrix = matrix;
  // For each x in forbidden x, set the row to 0
  for (const auto& x : forbidden_x){
    auto zero_vector = Eigen::MatrixXd::Zero(1, matrix.cols());
    new_matrix.block(x, 0, x, matrix.cols()) = zero_vector;
  }
  // For each y in forbidden y, set the column to 0
  for (const auto& y : forbidden_y){
    auto zero_vector = Eigen::MatrixXd::Zero(1, matrix.cols());
    new_matrix.block(0, y, matrix.rows(), y) = zero_vector;
  }
  return new_matrix;
};

void compute_slack(
  const int x,
  std::vector<double> & slack,
  std::vector<double> & slackx,
  const Eigen::MatrixXd & cost,
  const Eigen::VectorXd & lx,
  const Eigen::VectorXd & ly
)
/*
The slack used to update the labeling (weight) of nodes.

- Initialized before starting the algorithm.
- Called every time we add an augmenting path.
- Called after each time we update labels using the minimum slack
value.
*/
{
  for (size_t y = 0; y < static_cast<size_t>(cost.cols()); ++y) {
    if (lx[x] + ly[y] - cost(x, y) < slack[y]) {
      slack[y] = lx[x] + ly[y] - cost(x, y);
      slackx[y] = x;
    }
  }
}

std::vector<int> max_cost_assignment(
  const Eigen::MatrixXd & cost_matrix,
  bool max_cost, 
  std::vector<int> forbidden_x, 
  std::vector<int> forbidden_y
)
{
  /*
  Find the maximum cost assignment (aka min cost if you invert all the signs) of
  a bipartite graph (i.e. robots and field positions) using the Hungarian (Kuhn-Munkres)
  Maximum Matching Algorithm.

  This implementation is largely based off of the one in dlib but allows
  us to use our own types and types from Eigen rather than the dlib ones
  as well as use fp (and not just int) values.
  https://github.com/davisking/dlib/blob/master/dlib/optimization/max_cost_assignment.h

  This resource can be used for reference:
  (two different links, but the content is identical)
      https://cse.hkust.edu.hk/~golin/COMP572/Notes/Matching.pdf
      https://www.columbia.edu/~cs2035/courses/ieor6614.S16/GolinAssignmentNotes.pdf

  This resource is a bit more clear but I unfortunately found it after the two above:
    https://cp-algorithms.com/graph/hungarian-algorithm.html#implementation-of-the-hungarian-algorithm
  */

  // Make sure our matrix fits our algorithms requirements
  // (it is square and non-negative).
  auto cost = cost_matrix;
  cost = scale_cost_matrix(cost, max_cost);
  if (cost.cols() != cost.rows()) {
    cost = make_square_cost_matrix(cost);
  }
  cost = replace_nan_costs_with_zeros(cost);
  cost = replace_forbidden_costs_with_zeros(cost, forbidden_x, forbidden_y);

  // Step 1: Create an initial feasible labeling,
  // clear out sets S and T, and reset our slack values.

  // Size of each set of nodes in the bipartite graph
  size_t cost_size = static_cast<size_t>(cost.cols());

  // Initial labeling.
  // We set all ly = 0, all lx to their max value.
  Eigen::VectorXd ly = Eigen::VectorXd().setConstant(cost_size, 0);
  Eigen::VectorXd lx = cost.rowwise().maxCoeff();

  // Boolean indicator for whether node is a member of set S or T
  // S includes only nodes in X, T only nodes in Y
  std::vector<char> S, T;

  // Min l(x) + l(y) - cost(x,y) given the current S and T
  std::vector<double> slack;
  // The corresponding edge for each slack value (matching of y to x)
  std::vector<double> slackx;

  // Contains the indices of the augmenting path
  std::vector<int> aug_path;

  /*
  Vertex x is matched to vertex xy[x] and vertex y is matched to vertex yx[y].
  We start with a graph where nothing is matched (all matches set to -1)
  X corresponds to rows of the cost matrix and y corresponds to the
  columns of the cost matrix.
  */
  // Edges of X to Y
  std::vector<int> xy(cost_size, -1);
  // Edges of Y to X
  std::vector<int> yx(cost_size, -1);

  // Now grow the match set by picking edges from the equality subgraph until
  // we have a complete matching.

  for (size_t match_size = 0; match_size < cost_size; ++match_size) {
    // Contains free y nodes that would create a possible augmenting path
    std::deque<int> free_xs;

    // Empty out the S and T sets
    S.assign(cost_size, false);
    T.assign(cost_size, false);

    // Set all the initial slack to max; if something is unassignable,
    // it should stay this maximum cost.
    slack.assign(cost_size, std::numeric_limits<double>::max());
    slackx.resize(cost_size);
    aug_path.assign(cost_size, -1);

    // Step 2: Check if we have a perfect matching (if every vertex
    // in the graph has a match). If so, GOTO end.
    //
    // Otherwise we pick a free vertex in X and add it to S.
    // Next we will either pick a corresponding match y in T
    // OR IF sum(l(x)) == sum(l(y)), update our labeling
    // so sum(l(y)) != sum(l(x)) using the min slack value.

    for (size_t x = 0; x < cost_size; ++x) {
      // If x is not matched to anything
      if (xy[x] == -1) {
        free_xs.push_back(x);
        S[x] = true;

        // Compute slack in preparation for step 3
        compute_slack(x, slack, slackx, cost, lx, ly);
        break;
      }
    }

    int x_start = 0;
    int y_start = 0;

    // Step 4: Pick a y and get an augmenting path
    bool found_augmenting_path = false;
    while (!found_augmenting_path) {
      while (free_xs.size() > 0 && !found_augmenting_path) {
        const int x = free_xs.front();
        free_xs.pop_front();
        for (size_t y = 0; y < cost_size; ++y) {
          // Need to replace with within an epsilon
          if (std::abs(cost(x, y) - (lx[x] + ly[y])) < EPSILON && !T[y]) {
            // if vertex y isn't matched with anything
            // then we have an augmenting path
            if (yx[y] == -1) {
              y_start = y;
              x_start = x;
              found_augmenting_path = true;
              break;
            }

            // Else, then we need to extend an alternating
            // tree AND there is no augmenting path;
            // add y to T and x to S
            T[y] = true;
            free_xs.push_back(yx[y]);

            aug_path[yx[y]] = x;
            S[yx[y]] = true;
            compute_slack(yx[y], slack, slackx, cost, lx, ly);
          }
        }
      }

      if (found_augmenting_path) {
        break;
      }

      // Step 3: Updating labeling if all matched y in T
      // Since we didn't find an augmenting path we need to improve the
      // feasible labeling stored in lx and ly.  We also need to keep the
      // slack updated accordingly.
      // (The delta will be the min slack value)
      double delta = std::numeric_limits<double>::max();
      for (size_t i = 0; i < T.size(); ++i) {
        if (!T[i]) {
          delta = std::min(delta, slack[i]);
        }
      }
      for (size_t i = 0; i < T.size(); ++i) {
        if (S[i]) {
          lx[i] -= delta;
        }

        if (T[i]) {
          ly[i] += delta;
        } else {
          // We do this b/c the slack relies on the labeling of x.
          // This allows us to avoid fully recalculating the slack.
          slack[i] -= delta;
        }
      }

      // Reset the queue for when we need to augment the path
      free_xs.clear();
      for (size_t y = 0; y < cost_size; ++y) {
        if (!T[y] && slack[y] < EPSILON) {
          // if vertex y isn't matched with anything
          if (yx[y] == -1) {
            x_start = slackx[y];
            y_start = y;
            found_augmenting_path = true;
            break;
          } else {
            T[y] = true;
            if (!S[yx[y]]) {
              free_xs.push_back(yx[y]);

              aug_path[yx[y]] = slackx[y];
              S[yx[y]] = true;
              compute_slack(yx[y], slack, slackx, cost, lx, ly);
            }
          }
        }
      }
    }         // end while (!found_augmenting_path)

    // Flip the edges along the augmenting path.  This means we will add one more
    // item to our matching.
    for (int cx = x_start, cy = y_start, ty;
      cx != -1;
      cx = aug_path[cx], cy = ty)
    {
      ty = xy[cx];
      yx[cy] = cx;
      xy[cx] = cy;
    }
  }

  for (size_t x = 0; x < cost.rows(); ++x) {
    // If we assign an x a y that did not exist in the original,
    // (or vice versa) set it to not assigned (-1)
    if (xy[x] >= static_cast<int>(cost_matrix.cols())) {
      xy[x] = -1;
    }
  }

  // Return our perfect matching
  // based on our feasible labeling
  return xy;
}

std::vector<int> min_cost_assignment(
  const Eigen::MatrixXd & cost_matrix,
  std::vector<int> forbidden_x, 
  std::vector<int> forbidden_y
){
  return max_cost_assignment(cost_matrix, false, forbidden_x, forbidden_y);
};

}  // namespace ateam_common::km_assignment
