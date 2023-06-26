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

#include "trajectory_generation/b_spline.hpp"

// #include <algorithm>
#include <iostream>

namespace BSpline {

void sample_spline(const InternalState & state) {
  std::cout << "Samples" << std::endl;
  std::vector<Eigen::Vector2d> samples;
  for (int i = 0; i <= 10; i++) {
    samples.push_back(de_boors_algorithm(i * 0.1, 3, state.control_points, state.knot_sequence_with_multiplicity));

    std::cout << "(" << i * 0.1 << ") " << samples.back().x() << " " << samples.back().y() << std::endl;
  }
}

InternalState convert_to_spline(const Input & input) {
  // See secction 9.4 page 154 (pdf page 173)
  // p_i represents the data points
  // d_i represents the control points
  // tau_i represents the knot points
  //
  // L = K + 2 where K is the number of data points

  // Know d_0 == p_0 and d_L = p_k
  // d_1 = d_0 + (tau_1 - tau_0) / 3 * intial_vel
  // d_l-1 = d_L - (tau_k - tau_k-1) / 3 * end_vel

  // Need to calculate d_2 through d_l-2 per eq 9.12

  std::vector<double> tau = get_knot_sequence(input.data_points);
  std::vector<double> tau_with_multiplicity = apply_multiplicity(tau, 3);

  // 0 ... K is k+1 data points
  std::size_t k = input.data_points.size() - 1;
  std::size_t L = k + 2;
  std::cout << "k = " << k << "\tL = " << L << "\ttau = " << tau.size() << "\ttauwm = " << tau_with_multiplicity.size() << std::endl;

  // Front and back data point must match exactly
  Eigen::Vector2d d_0 = input.data_points.at(0);
  Eigen::Vector2d d_L = input.data_points.at(k);

  // Second and second from last data points determine initial/end speeds
  Eigen::Vector2d d_1 = d_0 + (tau.at(1) - tau.at(0)) / 3 * input.initial_vel;
  Eigen::Vector2d d_L_1 = d_L - (tau.at(k) - tau.at(k - 1)) / 3 * input.end_vel;

  // Knowing the first and second (and last and second to last) data points result a little more
  // specific equations in the least squares
  Eigen::Vector2d r_s = input.data_points.at(1) - d_1 * basis_function(1, 3, tau.at(1), tau_with_multiplicity);
  Eigen::Vector2d r_e = input.data_points.at(k - 1) - d_L_1 * basis_function(L - 2, 3, tau.at(k - 1), tau_with_multiplicity);

  // Eq 9.11, don't use 9.12 as it's difficult to figure out 
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(k - 1, k - 1);
  A(0, 0) = basis_function(2, 3, tau.at(1), tau_with_multiplicity);
  A(0, 1) = basis_function(3, 3, tau.at(1), tau_with_multiplicity);
  for (std::size_t i = 2; i <= k - 2; i++) {
    for (std::size_t p = 0; p < 3; p++) {
      // Get the p==2 on the diagonal
      int col_idx = p + i - 2;
      int row_idx = i - 1;
      
      // First and law row start and end outside bounds
      if (col_idx < 0 || col_idx >= A.rows()) {
        continue;
      }
      A(row_idx, col_idx) = basis_function(i + p, 3, tau.at(i), tau_with_multiplicity);
    }
  }
  A(k - 2, k - 3) = basis_function(L - 3, 3, tau.at(k - 1), tau_with_multiplicity);
  A(k - 2, k - 2) = basis_function(L - 2, 3, tau.at(k - 1), tau_with_multiplicity);
  std::cout << std::endl << A << std::endl << std::endl;

  // See EQ 9.12, vector b of the Ax=b
  Eigen::VectorXd b = Eigen::VectorXd::Zero(k - 1);
  for (int i = 0; i < b.rows(); i++) {
    if (i == 0) {
      // i = 0
      b(i) = r_s.x();
    } else if (i == b.rows() - 1) {
      // i = K - 1
      b(i) = r_e.x();
    } else {
      // i = (1 ... K - 2)
      b(i) = input.data_points.at(i + 1).x(); // 2 .. K-2
    }
  }

  std::cout << std::endl << b << std::endl << std::endl;

  // Output is the x coordinates of the control points 2..L-2
  // TODO(jneiger): replace with a true "tridiagonal" solver like the following
  // https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm
  Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

  for (int i = 0; i < b.rows(); i++) {
    if (i == 0) {
      // i = 0
      b(i) = r_s.y();
    } else if (i == b.rows() - 1) {
      // i = K - 1
      b(i) = r_e.y();
    } else {
      // i = (1 ... K - 2)
      b(i) = input.data_points.at(i + 1).y(); // 2 .. K-2
    }
  }
  Eigen::VectorXd y = A.colPivHouseholderQr().solve(b);
  
  std::vector<Eigen::Vector2d> control_points;
  control_points.push_back(d_0);
  control_points.push_back(d_1);
  for (int i = 0; i < x.rows(); i++) {
    control_points.push_back(Eigen::Vector2d{x(i), y(i)});
  }
  control_points.push_back(d_L_1);
  control_points.push_back(d_L);

  for (const auto & p : control_points) {
    std::cout << p.x() << " " << p.y() << std::endl;
  }


  InternalState out;
  out.control_points = control_points;
  out.knot_sequence = tau;
  out.knot_sequence_with_multiplicity = tau_with_multiplicity;
  return out;
}

std::vector<double> apply_multiplicity(const std::vector<double> & knot_sequence, const std::size_t multiplicity) {
  std::vector<double> out;
  for (std::size_t i = 0; i < multiplicity; i++) {
    out.push_back(knot_sequence.front());
  }
  for (std::size_t i = 1; i < knot_sequence.size() - 1; i++) {
    out.push_back(knot_sequence.at(i));
  }
  for (std::size_t i = 0; i < multiplicity; i++) {
    out.push_back(knot_sequence.back());
  }

  return out;
}

std::vector<double> get_knot_sequence(const std::vector<Eigen::Vector2d> & control_points) {
  return constant_spacing(control_points);
}

std::vector<double> constant_spacing(const std::vector<Eigen::Vector2d> & control_points) {
  std::vector<double> out;
  double spacing = 1.0 / (control_points.size() - 1);
  for (int i = 0; i < control_points.size(); i++) {
    out.push_back(i * spacing);
  }

  return out;
}

std::vector<double> chord_length_parametrization_spacing(const std::vector<Eigen::Vector2d> & control_points) {
  double total_length = 0;
  for (int i = 1; i < control_points.size(); i++) {
    total_length += (control_points.at(i) - control_points.at(i - 1)).norm();
  }

  std::vector<double> out;

  // First point is always 0
  out.push_back(0);
  for (int i = 1; i < control_points.size(); i++) {
    out.push_back(out.back() + (control_points.at(i) - control_points.at(i - 1)).norm() / total_length);
  }
  // Last point should always be 1

  return out;
}


std::vector<double> centripetal_spacing(const std::vector<Eigen::Vector2d> & control_points) {
  double total_length = 0;
  for (int i = 1; i < control_points.size(); i++) {
    total_length += std::sqrt((control_points.at(i) - control_points.at(i - 1)).norm());
  }

  std::vector<double> out;

  // First point is always 0
  out.push_back(0);
  for (int i = 1; i < control_points.size(); i++) {
    out.push_back(out.back() + std::sqrt((control_points.at(i) - control_points.at(i - 1)).norm()) / total_length);
  }
  // Last point should always be 1

  return out;
}

double basis_function(const std::size_t i, const std::size_t p, const double u, const std::vector<double> & knot_sequence) {
  if (p == 0) {
    if (knot_sequence.at(i) <= u && u < knot_sequence.at(i + 1)) {
      return 1;
    } else {
      return 0;
    }
  }

  // For the sake of calculation, 0/0 = 0
  // See Knots with Positive Multiplicity https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-ex-1.html
  auto coeff = [&knot_sequence] (double u, std::size_t i, std::size_t p) {
    if (knot_sequence.at(i + p) == knot_sequence.at(i)) {
      return 0.0;
    }
    return (u - knot_sequence.at(i)) / (knot_sequence.at(i + p) - knot_sequence.at(i));
  };

  // See Definition https://en.wikipedia.org/wiki/B-spline
  return coeff(u, i, p) * basis_function(i, p - 1, u, knot_sequence) +
   (1 - coeff(u, i + 1, p)) * basis_function(i + 1, p - 1, u, knot_sequence);
}

std::vector<Eigen::Vector2d> knot_points(const std::size_t degree, const std::vector<Eigen::Vector2d> & control_points, const std::vector<double> & knot_sequence) {
  std::vector<Eigen::Vector2d> out;
  for (const auto & u : knot_sequence) {
    Eigen::Vector2d knot_point{0, 0};

    for (int i = 0; i < control_points.size(); i++) {
      knot_point += basis_function(i, degree, u, knot_sequence) * control_points.at(i);
    }
    out.push_back(knot_point);
  }

  return out;
}

Eigen::Vector2d de_boors_algorithm(const double u, const std::size_t degree, const std::vector<Eigen::Vector2d> & control_points, const std::vector<double> & knot_sequence) {
  // Using variables from https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/de-Boor.html

  // Find the range [u_k, u_k+1) at which u fits within the squence
  std::size_t k;
  std::size_t equal_count = 0;
  for (k = 0; k < knot_sequence.size() - 1; k++) {
    if (u >= knot_sequence.at(k) && u < knot_sequence.at(k + 1)) {
      break;
    }
  }

  // Knots are strictly ascending so any that match will be grouped
  for (const auto & knot : knot_sequence) {
    if (u == knot) {
      equal_count++;
    }
  }

  auto clip_idx = [](const auto & vector, const int & idx) {
    return std::min(std::max(idx, 0), static_cast<int>(vector.size()) - 1);
  };

  // Number of times to insert the control point in the squence
  // Want equal to degree such that the curve moves through the specific control point
  // The total number can include a number of points already within control point list
  const std::size_t h = std::max(degree - equal_count, 0ul);
  const std::size_t s = equal_count;

  // If we already have enough multiplicity, just return the control point
  if (degree == s) {
    return control_points.at(clip_idx(control_points, k));
  }

  // Copy the control points that detail the curve at this point
  std::unordered_map<int, std::unordered_map<int, Eigen::Vector2d>> affected_control_points;
  for (int i = k - s; i >= static_cast<int>(k) - static_cast<int>(degree); i--) {
    affected_control_points[i][0] = control_points.at(clip_idx(control_points, i));
  }

  // Use a corner cutting process to figure out the location on the curve
  for (std::size_t r = 1; r <= h; r++) {
    for (std::size_t i = k - degree + r; i <= k - s; i++) {
      double a_i_r = (u - knot_sequence.at(i)) / (knot_sequence.at(i + degree - r + 1) - knot_sequence.at(i));

      affected_control_points[i][r] = (1 - a_i_r) * affected_control_points[i - 1][r - 1] + a_i_r * affected_control_points[i][r - 1];
    }
  }

  return affected_control_points[k - s][degree - s];
}

}  // BSpline
