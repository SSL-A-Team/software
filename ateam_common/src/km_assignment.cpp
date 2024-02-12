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
#include <vector>
#include <deque>
#include "ateam_common/km_assignment.hpp"

namespace ateam_common::km_assignment
{
const float EPSILON = 1e-10;

const Eigen::MatrixXd make_square_cost_matrix(
  const Eigen::MatrixXd & matrix)
{
  if (matrix.rows() == matrix.cols()) {
    return matrix;
  }

  std::size_t new_dim = std::max(matrix.rows(), matrix.cols());

  //  The value of any non-existant cost is 0 (since we are
  //  trying to find a MAX cost matching)
  Eigen::MatrixXd new_mat =
    Eigen::MatrixXd::Constant(new_dim, new_dim, std::numeric_limits<double>::min());

  new_mat.block(0, 0, matrix.rows(), matrix.cols()) = matrix;

  return new_mat;
}

inline void compute_slack(
  const int x,
  std::vector<double> & slack,
  std::vector<double> & slackx,
  const Eigen::MatrixXd & cost,
  const Eigen::VectorXd & lx,
  const Eigen::VectorXd & ly
)
{
  for (size_t y = 0; y < static_cast<size_t>(cost.cols()); ++y) {
    if (lx[x] + ly[y] - cost(x, y) < slack[y]) {
      slack[y] = lx[x] + ly[y] - cost(x, y);
      slackx[y] = x;
    }
  }
}

std::vector<int> max_cost_assignment(
  const Eigen::MatrixXd & cost_matrix
)
{
  auto cost = cost_matrix;
  if (cost_matrix.cols() != cost_matrix.rows()) {
    cost = make_square_cost_matrix(cost_matrix);
  }
  /*
      The dlib implementation of this algorithm is based on info available below...
      although the links were broken when I tried them.
          http://www.math.uwo.ca/~mdawes/courses/344/kuhn-munkres.pdf
          http://www.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm

      I used this reference instead:
          https://cse.hkust.edu.hk/~golin/COMP572/Notes/Matching.pdf
      */

  Eigen::VectorXd lx, ly;
  std::vector<int> xy;
  std::vector<int> yx;
  std::vector<char> S, T;
  std::vector<double> slack;
  std::vector<double> slackx;
  std::vector<int> aug_path;

  size_t cost_size = static_cast<size_t>(cost.cols());

  // Initially, nothing is matched.
  xy.assign(cost_size, -1);
  yx.assign(cost_size, -1);
  /*
      We maintain the following invariant:
          Vertex x is matched to vertex xy[x] and
          vertex y is matched to vertex yx[y].

          A value of -1 means a vertex isn't matched to anything.  Moreover,
          x corresponds to rows of the cost matrix and y corresponds to the
          columns of the cost matrix.  So we are matching X to Y.
      */

  // Create an initial feasible labeling.  Moreover, in the following
  // code we will always have:
  //     for all valid x and y:  lx[x] + ly[y] >= cost(x,y)
  // We set all ly = 0
  ly.setConstant(cost_size, 0);
  // We set all lx to their max value
  lx = cost.rowwise().maxCoeff();

  // Now grow the match set by picking edges from the equality subgraph until
  // we have a complete matching.
  for (size_t match_size = 0; match_size < cost_size; ++match_size) {
    std::deque<int> q;

    // Empty out the S and T sets
    S.assign(cost_size, false);
    T.assign(cost_size, false);

    // clear out old slack values
    slack.assign(cost_size, std::numeric_limits<double>::max());
    slackx.resize(cost_size);
    /*
        slack and slackx are maintained such that we always
        have the following (once they get initialized by compute_slack() below):
            - for all y:
                - let x == slackx[y]
                - slack[y] == lx[x] + ly[y] - cost(x,y)
        */

    aug_path.assign(cost_size, -1);

    // Step 2: Check if we have a perfect matching
    for (size_t x = 0; x < cost_size; ++x) {
      // If x is not matched to anything
      if (xy[x] == -1) {
        q.push_back(x);
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
      while (q.size() > 0 && !found_augmenting_path) {
        const int x = q.front();
        q.pop_front();
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
            q.push_back(yx[y]);

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
      q.clear();
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
              q.push_back(yx[y]);

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
}  // namespace ateam_common::km_assignment
