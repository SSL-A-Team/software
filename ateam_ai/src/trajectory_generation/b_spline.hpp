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

#ifndef TRAJECTORY_GENERATION__B_SPLINE_HPP_
#define TRAJECTORY_GENERATION__B_SPLINE_HPP_

#include <Eigen/Dense>

#include <unordered_map>
#include <vector>

/**
 * See https://www.cin.ufpe.br/~mdlm/files/Farin-5a_edicao.pdf as reference
*/
namespace BSpline
{

struct Input {
  std::vector<Eigen::Vector2d> control_points;
  Eigen::Vector2d initial_vel;
  Eigen::Vector2d end_vel;
};

/**
 * Repeats the first and last control point |multiplicity| times to force end conditions
 * on the start and end control point
 *
 * |multiplicity| = 3 with velocity constraints produce splines that start/end at the end control_points with the defined velocity
*/
std::vector<Eigen::Vector2d> apply_multiplicity(const std::vector<Eigen::Vector2d> & control_points, std::size_t multiplicity);

/**
 * Given a sequence of control points, find the spacing of knots.
 * This is pre multiplicity.
 * 
 * Spacing is on a scale of 0 ... 1 inclusive
*/
std::vector<double> get_knot_sequence(const std::vector<Eigen::Vector2d> & control_points);

/**
 * Applies linear spacing of knots
*/
std::vector<double> constant_spacing(const std::vector<Eigen::Vector2d> & control_points);

/**
 * Applies a distance proportional spacing of knots
 *
 * Eq 9.16, page 163
*/
std::vector<double> chord_length_parametrization_spacing(const std::vector<Eigen::Vector2d> & control_points);

/**
 * Applies a sqrt of the distance proportional spacing of knots
 *
 * Eq 9.17, page 163
*/
std::vector<double> centripetal_spacing(const std::vector<Eigen::Vector2d> & control_points);

/**
 * Calculates the basis function N_i,p(u)
 *
 * The basis function is a box car step functions
*/
double basis_function(const std::size_t i, const std::size_t p, const double u, const std::vector<double> & knot_sequence);

/**
 * Calculates the knot points
 *
 * Knot points are on the B spline curve that split the curve into segments
*/
std::vector<Eigen::Vector2d> knot_points(const std::size_t degree, const std::vector<Eigen::Vector2d> & control_points, const std::vector<double> & knot_sequence);
}  // namespace BSpline

#endif  // TRAJECTORY_GENERATION__B_SPLINE_HPP_
