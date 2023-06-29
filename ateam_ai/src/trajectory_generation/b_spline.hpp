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
 *
 * Data points refer to the waypoint inputs
 * Control points refer to the points in the spline definition that pull the spline in different directions
 * Knot points refer to how to disbribute the basis functions across the length of the curve
 * Basis functions deal with the specific weighting of the control points to determine the location of the point on the curve at u=X
*/
namespace BSpline
{

struct Input {
  std::vector<Eigen::Vector2d> data_points;
  Eigen::Vector2d initial_vel;
  Eigen::Vector2d end_vel;

  double max_accel;
  double max_vel;
};

struct InternalState {
  std::vector<Eigen::Vector2d> control_points;
  std::vector<double> knot_sequence;
  std::vector<double> knot_sequence_with_multiplicity;
};

struct Output {
  struct Sample2d {
    Eigen::Vector2d p;
    double v;
    double a;
  };
  // Samples are a function of position
  std::vector<Sample2d> samples;
};

/**
 * Build and sample the spline |num_samples| number of times
*/
Output build_and_sample_spline(const Input & input, const std::size_t num_samples);

/**
 * Given the internal control and knot sequences.
 * Produce a series of equally sampled points along the spline
*/
std::vector<Eigen::Vector2d> sample_spline(const InternalState & state, const std::size_t num_samples);

/**
 * Convert the data points to a series of control points and knots
*/
InternalState convert_to_spline(const Input & input);

/**
 * Repeats the first and last knot point |multiplicity| times to force end conditions
 * on the start and end control point
 *
 * |multiplicity| = 3 with velocity constraints produce splines that start/end at the end control_points with the defined velocity
*/
std::vector<double> apply_multiplicity(const std::vector<double> & knot_sequence, const std::size_t multiplicity);

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

/**
 * Samples the curve at location u in range [0, 1] corresponding the the % along the B spline
*/
Eigen::Vector2d de_boors_algorithm(const double u, const std::size_t degree, const std::vector<Eigen::Vector2d> & control_points, const std::vector<double> & knot_sequence);
}  // namespace BSpline

#endif  // TRAJECTORY_GENERATION__B_SPLINE_HPP_
