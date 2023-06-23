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
#include <iostream>
namespace BSpline {
std::vector<Eigen::Vector2d> apply_multiplicity(const std::vector<Eigen::Vector2d> & control_points, std::size_t multiplicity) {
  std::vector<Eigen::Vector2d> out;
  for (int i = 0; i < multiplicity; i++) {
    out.push_back(control_points.front());
  }
  for (int i = 1; i < control_points.size() - 1; i++) {
    out.push_back(control_points.at(i));
  }
  for (int i = 0; i < multiplicity; i++) {
    out.push_back(control_points.back());
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

  return (u - knot_sequence.at(i)) / (knot_sequence.at(i + p) - knot_sequence.at(i)) * basis_function(i, p - 1, u, knot_sequence) +
   (knot_sequence.at(i + p + 1) - u) / (knot_sequence.at(i + p + 1) - knot_sequence.at(i + 1)) * basis_function(i + 1, p - 1, u, knot_sequence);
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

}  // BSpline
