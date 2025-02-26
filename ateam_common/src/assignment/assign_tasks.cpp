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

#include <Eigen/Dense>
#include <limits>
#include <cmath>
#include <vector>
#include <deque>
#include <iostream>
#include "ateam_common/assignment/assign_tasks.hpp"
#include "cost_matrix_utilities.hpp"

namespace ateam_common::assignment
{
const float EPSILON = 1e-10;

/**
 * @brief Make sure the cost matrix fits the algorithm requirements.
 */
Eigen::MatrixXd PrepareCostMatrix(
  const Eigen::MatrixXd & input_costs, AssignmentType max_or_min,
  std::map<int, std::vector<int>> forbidden_assignments)
{
  auto cost = input_costs;
  cost = ScaleCostMatrix(cost, max_or_min);
  if (cost.cols() != cost.rows()) {
    cost = MakeSquareCostMatrix(cost);
  }

  cost += Eigen::MatrixXd::Ones(cost.rows(), cost.cols());

  // cost = ReplaceNanCostsWithValue(cost, std::numeric_limits<double>::lowest());
  cost = ReplaceNanCostsWithValue(cost, 0.0);
  if (!forbidden_assignments.empty()) {
    // cost = ReplaceForbiddenCostsWithValue(cost, forbidden_assignments,
        // std::numeric_limits<double>::lowest());
    cost = ReplaceForbiddenCostsWithValue(cost, forbidden_assignments, 0.0);
  }

  return cost;
}

/**
 * @brief Calculate slack used to update the labeling (weight) of nodes.
 *
 * Initialized before starting the algorithm.
 * Called every time we add an augmenting path.
 * Called after each time we update labels using the minimum slack value.
 *
 * @param x
 * @param slack
 * @param slackx
 * @param cost
 * @param lx
 * @param ly
 */
void ComputeSlack(
  const int x,
  std::vector<double> & slack,
  std::vector<std::size_t> & slackx,
  const Eigen::MatrixXd & cost,
  const Eigen::VectorXd & lx,
  const Eigen::VectorXd & ly)
{
  for (size_t y = 0; y < static_cast<size_t>(cost.cols()); ++y) {
    if (lx[x] + ly[y] - cost(x, y) < slack[y]) {
      slack[y] = lx[x] + ly[y] - cost(x, y);
      slackx[y] = x;
    }
  }
}

/**
 * @brief Pick a free vertex in X and add it to S
 *
 */
std::size_t PickAFreeX(
  const std::vector<int> & xy, std::deque<int> & free_xs,
  std::vector<char> & S)
{
  for (size_t x = 0; x < xy.size(); ++x) {
    // If x is not matched to anything
    if (xy[x] == -1) {
      free_xs.push_back(x);
      S[x] = true;
      return x;
    }
  }
  throw std::runtime_error("No nodes in X were free. This should not happen.");
}

bool FindAugmentingPath(
  int & x_start, int & y_start, std::deque<int> & free_xs, const Eigen::MatrixXd & cost,
  Eigen::VectorXd & lx, Eigen::VectorXd & ly, std::vector<char> & S, std::vector<char> & T,
  std::vector<double> & slack, std::vector<std::size_t> & slackx,
  std::vector<std::size_t> & aug_path, std::vector<int> & yx)
{
  std::size_t cost_size = static_cast<std::size_t>(cost.cols());
  while (free_xs.size() > 0) {
    const int x = free_xs.front();
    free_xs.pop_front();
    for (size_t y = 0; y < cost_size; ++y) {
      if (std::abs(cost(x, y) - (lx[x] + ly[y])) < EPSILON && !T[y]) {
        // If vertex y isn't matched with anything, we have an augmenting path
        if (yx[y] == -1) {
          y_start = y;
          x_start = x;
          return true;
        }

        // Else, we need to extend an alternating tree AND there is no augmenting path
        // Add y to T and x to S
        T[y] = true;
        free_xs.push_back(yx[y]);

        aug_path[yx[y]] = x;
        S[yx[y]] = true;
        ComputeSlack(yx[y], slack, slackx, cost, lx, ly);
      }
    }
  }
  return false;
}

// TODO(barulicm): I don't actually understand what this function does
bool FindAugmentingPath2(
  int & x_start, int & y_start, std::deque<int> & free_xs, const Eigen::MatrixXd & cost,
  Eigen::VectorXd & lx, Eigen::VectorXd & ly, std::vector<char> & S, std::vector<char> & T,
  std::vector<double> & slack, std::vector<std::size_t> & slackx,
  std::vector<std::size_t> & aug_path, std::vector<int> & yx)
{
  std::size_t cost_size = static_cast<std::size_t>(cost.cols());
  free_xs.clear();
  for (size_t y = 0; y < cost_size; ++y) {
    if (!T[y] && std::abs(slack[y]) < EPSILON) {
        // if vertex y isn't matched with anything
      if (yx[y] == -1) {
        x_start = slackx[y];
        y_start = y;
        return true;
      } else {
        T[y] = true;
        if (!S[yx[y]]) {
          free_xs.push_back(yx[y]);

          aug_path[yx[y]] = slackx[y];
          S[yx[y]] = true;
          ComputeSlack(yx[y], slack, slackx, cost, lx, ly);
        }
      }
    }
  }
  return false;
}

void UpdateLabels(
  Eigen::VectorXd & lx, Eigen::VectorXd & ly, std::vector<char> & S,
  std::vector<char> & T, std::vector<double> & slack)
{
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
      slack[i] -= delta;
    }
  }
}

void FlipEdgesFromAugmentingPath(
  std::vector<int> & xy, std::vector<int> & yx, const int x_start,
  const int y_start, const std::vector<std::size_t> & aug_path)
{
  int cx = x_start;
  int cy = y_start;
  int ty = 0;

  while(cx != -1) {
    ty = xy[cx];
    yx[cy] = cx;
    xy[cx] = cy;

    cx = aug_path[cx];
    cy = ty;
  }
}

void AugmentMatch(
  const Eigen::MatrixXd & cost, Eigen::VectorXd & lx, Eigen::VectorXd & ly,
  std::vector<char> & S, std::vector<char> & T, std::vector<double> & slack,
  std::vector<std::size_t> & slackx, std::vector<std::size_t> & aug_path, std::vector<int> & xy,
  std::vector<int> & yx)
{
  std::size_t cost_size = static_cast<std::size_t>(cost.cols());

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

  const auto selected_x = PickAFreeX(xy, free_xs, S);
  ComputeSlack(selected_x, slack, slackx, cost, lx, ly);

  int x_start = 0;
  int y_start = 0;

  while (true) {
    if(FindAugmentingPath(x_start, y_start, free_xs, cost, lx, ly, S, T, slack, slackx, aug_path,
        yx))
    {
      break;
    }

    UpdateLabels(lx, ly, S, T, slack);

    if(FindAugmentingPath2(x_start, y_start, free_xs, cost, lx, ly, S, T, slack, slackx, aug_path,
        yx))
    {
      break;
    }
  }

  FlipEdgesFromAugmentingPath(xy, yx, x_start, y_start, aug_path);
}

void RemoveAssignmentsToFillerTasks(std::vector<int> & assignments, const int num_real_tasks)
{
  for(auto & task : assignments) {
    if(task >= num_real_tasks) {
      task = -1;
    }
  }
}

void RemoveFillerAgents(std::vector<int> & assignments, const int num_real_agents)
{
  assignments.resize(num_real_agents);
}

void RemoveForbiddenAssignments(
  std::vector<int> & assignments,
  const std::map<int, std::vector<int>> & forbidden_assignments)
{
  for (const auto & [key, val] : forbidden_assignments) {
    for (const auto & v : val) {
      if (assignments.at(key) == v) {
        assignments[key] = -1;
      }
    }
  }
}

std::vector<int> AssignTasks(
  const Eigen::MatrixXd & cost_matrix,
  AssignmentType max_or_min,
  std::map<int, std::vector<int>> forbidden_assignments
)
{
  /*
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

  if(cost_matrix.rows() == 0 || cost_matrix.cols() == 0) {
    return {};
  }

  const auto cost = PrepareCostMatrix(cost_matrix, max_or_min, forbidden_assignments);

  std::cerr << "Cost matrix:\n";
  std::cerr << cost << std::endl;

  // Size of each set of nodes in the bipartite graph
  std::size_t cost_size = static_cast<std::size_t>(cost.cols());

  // Initial labeling.
  // We set all ly = 0, all lx to their max value.
  Eigen::VectorXd ly = Eigen::VectorXd().setConstant(cost_size, 0);
  Eigen::VectorXd lx = cost.rowwise().maxCoeff();

  // Boolean indicator for whether node is a member of set S or T
  // S includes only nodes in X, T only nodes in Y
  std::vector<char> S;
  std::vector<char> T;

  // Min l(x) + l(y) - cost(x,y) given the current S and T
  std::vector<double> slack;
  // The corresponding edge for each slack value (matching of y to x)
  std::vector<std::size_t> slackx;

  // Contains the indices of the augmenting path
  std::vector<std::size_t> aug_path;

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

  for (std::size_t match_size = 0; match_size < cost_size; ++match_size) {
    AugmentMatch(cost, lx, ly, S, T, slack, slackx, aug_path, xy, yx);
  }

  RemoveAssignmentsToFillerTasks(xy, cost_matrix.cols());

  RemoveFillerAgents(xy, cost_matrix.rows());

  RemoveForbiddenAssignments(xy, forbidden_assignments);

  return xy;
}

}  // namespace ateam_common::assignment
