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

#include <ateam_common/status.hpp>
#include <deque>

namespace BSpline {

Output build_and_sample_spline(const Input & input, const std::size_t num_samples) {
  std::vector<Eigen::Vector2d> sample_poses = sample_spline(convert_to_spline(input), num_samples);
  double length = 0.0;
  for (int i = 1; i < sample_poses.size(); i++) {
    length += (sample_poses.at(i) - sample_poses.at(i - 1)).norm();
  }

  std::deque<double> sample_curvatures;
  for (int i = 2; i < sample_poses.size(); i++) {
    Eigen::Vector2d diff2 = (sample_poses.at(i) - sample_poses.at(i - 1));
    Eigen::Vector2d diff1 = (sample_poses.at(i - 1) - sample_poses.at(i - 2));
    Eigen::Vector2d avg_diff = (diff1 + diff2) / 2;
    Eigen::Vector2d diffdiff = diff2 - diff1;
    double curvature = std::abs((avg_diff.x() * diffdiff.y() - avg_diff.y() * diffdiff.x()) / std::pow(avg_diff.squaredNorm(), 1.5));
    sample_curvatures.push_back(curvature);
  }
  // Because we need the second derivative, we "lose" 2 samples
  // Add duplicates at the front and back to keep everything the same length
  sample_curvatures.push_front(sample_curvatures.front());
  sample_curvatures.push_back(sample_curvatures.back());

  // Start forward pass
  std::vector<double> speed{input.initial_vel.norm()};
  std::vector<double> accel{input.max_accel};
  for (int i = 1; i < sample_poses.size(); i++) {
    double d = (sample_poses.at(i) - sample_poses.at(i - 1)).norm();
    // given d = v * t + 1/2 * a * t^2, solve for t
    // Gives 2 options, time will always be positive
    double a = accel.at(i - 1);
    double v = speed.at(i - 1);
    double t1 = -1 * (sqrt(2 * a * d + v*v) + v) / a;
    double t2 = (sqrt(2 * a * d + v*v) - v) / a;
    // Quick hack for a == 0
    if (std::abs(a) < 1e-5) {
      t1 = d / v;
      t2 = d / v;
    }
    double t = std::max(t1, t2);

    // Target vel is just the choice from last step
    double target_vel = v + a * t;
    // Only accel up to limit
    double normal_acceleration_from_curvature = sample_curvatures.at(i) * target_vel * target_vel;
    double target_accel = std::max(input.max_accel - normal_acceleration_from_curvature, 0.0);

    // If we are too fast, calculate the max speed at this point and hard set the target vel to that
    // and set target accel to 0
    if (sample_curvatures.at(i) * target_vel * target_vel > input.max_accel) {
      target_vel = sqrt(input.max_accel / sample_curvatures.at(i));
      target_accel = (target_vel - target_vel) / t;
    }

    // Limit vel if too fast
    if (target_vel >= input.max_vel) {
      target_accel = 0;  // Not actually correct but it's easier
      target_vel = input.max_vel;
    }

    speed.push_back(target_vel);
    accel.push_back(target_accel);
  }

  // Hard lock the last frame to the end
  // Max result in discontinuous samples
  speed.back() = input.end_vel.norm();
  accel.back() = -input.max_accel;

  // start backward pass
  for (int i = sample_poses.size() - 1; i >= 1; i--) {
    double d = (sample_poses.at(i) - sample_poses.at(i - 1)).norm();
    // given d = v * t + 1/2 * a * t^2, solve for t
    // Gives 2 options, time will always be positive
    double a = std::abs(accel.at(i));
    double v = std::abs(speed.at(i));

    double t1 = -1 * (sqrt(2 * a * d + v*v) + v) / a;
    double t2 = (sqrt(2 * a * d + v*v) - v) / a;
    if (std::abs(a) < 1e-5) {
      t1 = d / v;
      t2 = d / v;
    }
    double t = std::max(t1, t2);

    double normal_acceleration_from_curvature = sample_curvatures.at(i) * v * v;
    double target_accel = -std::max(input.max_accel - normal_acceleration_from_curvature, 0.0);
    double target_vel = v - target_accel * t;

    // If we are too fast, calculate the max speed at this point and hard set the target vel to that
    // and set target accel to 0
    if (sample_curvatures.at(i) * target_vel * target_vel > input.max_accel) {
      target_vel = sqrt(input.max_accel / sample_curvatures.at(i));
      target_accel = -(target_vel - target_vel) / t;
    }

    // Limit vel
    if (target_vel >= input.max_vel) {
      target_accel = 0;
      target_vel = input.max_vel;
    }

    if (target_vel < speed.at(i - 1)) {
      speed.at(i - 1) = target_vel;
      accel.at(i - 1) = target_accel;
    }
  }

  Output out;
  for (int i = 0; i < sample_poses.size(); i++) {
    out.samples.push_back(Output::Sample2d{.p = sample_poses.at(i), .v = speed.at(i), .a = accel.at(i)});
  }

  return out;
}

std::vector<Eigen::Vector2d> sample_spline(const InternalState & state, const std::size_t num_samples) {
  std::vector<Eigen::Vector2d> samples;
  for (int i = 0; i <= num_samples; i++) {
    samples.push_back(de_boors_algorithm(i * 1.0 / num_samples, 3, state.control_points, state.knot_sequence_with_multiplicity));
  }

  return samples;
}

InternalState convert_to_spline(const Input & input) {
  // See section 9.4 page 154 (pdf page 173)
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
  // Note that row 2 (index 1) is centered on the second basis function index (1, 2, 3)
  // which is i-1, i, i+1, not i, i+1, i+2. All tridiagonal lines should be non-zero
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(k - 1, k - 1);
  A(0, 0) = basis_function(1, 3, tau.at(1), tau_with_multiplicity);
  A(0, 1) = basis_function(2, 3, tau.at(1), tau_with_multiplicity);
  for (std::size_t i = 2; i <= k - 2; i++) {
    for (std::size_t p = 0; p < 3; p++) {
      // Get the p==2 on the diagonal
      int col_idx = p + i - 2;
      int row_idx = i - 1;
      
      // First and law row start and end outside bounds
      if (col_idx < 0 || col_idx >= A.rows()) {
        continue;
      }
      A(row_idx, col_idx) = basis_function(i + p - 1, 3, tau.at(i), tau_with_multiplicity);
    }
  }
  A(k - 2, k - 3) = basis_function(L - 4, 3, tau.at(k - 1), tau_with_multiplicity);
  A(k - 2, k - 2) = basis_function(L - 3, 3, tau.at(k - 1), tau_with_multiplicity);

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
  return centripetal_spacing(control_points);
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
  // Taking the example implementation with the extra programming optimization
  // from https://en.wikipedia.org/wiki/De_Boor%27s_algorithm#Example_implementation

  // This feels wrong deduping the knots, I think it's needed though?
  std::vector<double> un_dup_knots;
  for (const auto & knot : knot_sequence) {
    if (un_dup_knots.empty() || un_dup_knots.back() != knot) {
      un_dup_knots.push_back(knot);
    }
  }

  // Need to pad each side with |degree| knots, so multiplicity |degree| + 1
  std::vector<double> t = apply_multiplicity(un_dup_knots, degree + 1);

  // Find the range [u_k, u_k+1) at which u fits within the squence
  std::size_t k;
  for (k = 0; k < t.size() - 1; k++) {
    // If u == 1, return the range [prev_knot, 1) instead of [1,)
    if (u >= t.at(k) && u < t.at(k + 1) || t.at(k + 1) == 1.0) {
      break;
    }
  }

  // Section below is direct implementation of the algorithms
  std::vector<Eigen::Vector2d> d;
  for (int j = 0; j < degree + 1; j++) {
    // Control point list size, degree, and knot point list size are a function of each other
    // Note: that there is size buffering as well on the knot points which make this all confusing
    ATEAM_CHECK(j + k - degree >= 0, "Degree and control points sizes dont match");
    ATEAM_CHECK(j + k - degree < control_points.size(), "Degree and control points sizes dont match");

    d.push_back(control_points.at(j + k - degree));
  }

  for (int r = 1; r < degree + 1; r++) {
    for (int j = degree; j > r - 1; j--) {
      // See above
      // Mostly done to crash with error message instead of .at access failures
      ATEAM_CHECK(j + k - degree >= 0, "Degree and control points sizes dont match");
      ATEAM_CHECK(j + k - degree < t.size(), "Degree and control points sizes dont match");
      ATEAM_CHECK(j + 1 + k - r >= 0, "Degree and control points sizes dont match");
      ATEAM_CHECK(j + 1 + k - r < t.size(), "Degree and control points sizes dont match");
      ATEAM_CHECK(j >= 0, "Degree and control points sizes dont match");
      ATEAM_CHECK(j < d.size(), "Degree and control points sizes dont match");
      ATEAM_CHECK(j - 1 >= 0, "Degree and control points sizes dont match");
      ATEAM_CHECK(j - 1 < d.size(), "Degree and control points sizes dont match");

      double alpha = (u - t.at(j + k - degree)) / (t.at(j + 1 + k - r) - t.at(j + k - degree));
      d.at(j) = (1.0 - alpha) * d.at(j - 1) + alpha * d.at(j);
    }
  }

  return d.at(degree);
}

}  // BSpline
