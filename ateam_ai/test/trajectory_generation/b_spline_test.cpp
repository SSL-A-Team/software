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

#include <gtest/gtest.h>

#include <cmath>

TEST(b_spline, apply_multiplicity_ShouldRepeat_2point)
{
  std::vector<Eigen::Vector2d> control_points{
    Eigen::Vector2d{0, 0},
    Eigen::Vector2d{1, 1}
  };

  auto ret = BSpline::apply_multiplicity(control_points, 3);

  EXPECT_EQ(ret.size(), 6);
  for (int i = 0; i < 3; i++) {
    EXPECT_EQ(ret.at(i).x(), control_points.front().x());
    EXPECT_EQ(ret.at(i).y(), control_points.front().y());
  }
  for (int i = 3; i < 6; i++) {
    EXPECT_EQ(ret.at(i).x(), control_points.back().x());
    EXPECT_EQ(ret.at(i).y(), control_points.back().y());
  }
}

TEST(b_spline, constant_spacing_ShouldReturn1_2point)
{
  std::vector<Eigen::Vector2d> control_points{
    Eigen::Vector2d{0, 0},
    Eigen::Vector2d{1, 1}
  };

  auto out = BSpline::constant_spacing(control_points);

  EXPECT_EQ(out.size(), 2);
  EXPECT_EQ(out.front(), 0.0);
  EXPECT_EQ(out.back(), 1.0);
}

TEST(b_spline, constant_spacing_ShouldReturnHalf_3point)
{
  std::vector<Eigen::Vector2d> control_points{
    Eigen::Vector2d{0, 0},
    Eigen::Vector2d{1, 1},
    Eigen::Vector2d{2, 2}
  };

  auto out = BSpline::constant_spacing(control_points);

  EXPECT_EQ(out.size(), 3);
  EXPECT_EQ(out.front(), 0.0);
  EXPECT_EQ(out.at(1), 0.5);
  EXPECT_EQ(out.back(), 1.0);
}

TEST(b_spline, chord_length_ShouldReturnExample_4point)
{
  // Example taken from https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/INT-APP/PARA-chord-length.html
  std::vector<Eigen::Vector2d> control_points{
    Eigen::Vector2d{0, 0},
    Eigen::Vector2d{1, 2},
    Eigen::Vector2d{3, 4},
    Eigen::Vector2d{4, 0}
  };

  auto out = BSpline::chord_length_parametrization_spacing(control_points);

  EXPECT_EQ(out.size(), 4);
  EXPECT_EQ(out.front(), 0.0);
  EXPECT_NEAR(out.at(1), 0.2434, 1e-3);
  EXPECT_NEAR(out.at(2), 0.5512, 1e-3);
  EXPECT_NEAR(out.back(), 1.0, 1e-3);
}

TEST(b_spline, centripetal_ShouldReturnExample_4point)
{
  // Example taken from https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/INT-APP/PARA-chord-length.html
  std::vector<Eigen::Vector2d> control_points{
    Eigen::Vector2d{0, 0},
    Eigen::Vector2d{1, 2},
    Eigen::Vector2d{3, 4},
    Eigen::Vector2d{4, 0}
  };

  auto out = BSpline::centripetal_spacing(control_points);

  EXPECT_EQ(out.size(), 4);
  EXPECT_EQ(out.front(), 0.0);
  EXPECT_NEAR(out.at(1), 0.2871, 1e-3);
  EXPECT_NEAR(out.at(2), 0.6101, 1e-3);
  EXPECT_NEAR(out.back(), 1.0, 1e-3);
}

TEST(b_spline, basis_function_ShouldHalfUSq_WhenBetween01)
{
  // Example from https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-basis.html
  double u = 0.5;
  std::vector<double> knot_sequence{0, 1, 2, 3};
  auto out = BSpline::basis_function(0, 2, u, knot_sequence);

  EXPECT_NEAR(out, 0.5 * u * u, 1e-3);
}

TEST(b_spline, basis_function_ShouldWack_WhenBetween12)
{
  // Example from https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-basis.html
  double u = 1.5;
  std::vector<double> knot_sequence{0, 1, 2, 3};
  auto out = BSpline::basis_function(0, 2, u, knot_sequence);

  EXPECT_NEAR(out, 0.5 * (-3 + 6 * u - 2 * u * u), 1e-3);
}

TEST(b_spline, basis_function_ShouldWack_WhenBetween23)
{
  // Example from https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-basis.html
  double u = 2.5;
  std::vector<double> knot_sequence{0, 1, 2, 3};
  auto out = BSpline::basis_function(0, 2, u, knot_sequence);

  EXPECT_NEAR(out, 0.5 * (3 - u) * (3 - u), 1e-3);
}

TEST(b_spline, knot_points_ShouldCircle_WhenOverlaping)
{
  // Example from https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-curve-closed.html
  std::size_t p = 0;
  std::size_t n = 20;
  std::size_t m = n + p + 1;
  std::vector<Eigen::Vector2d> control_points;
  std::vector<double> knot_sequence;
  for (int i = 0; i < n + 1; i++) {
    double percent_circle = static_cast<double>(i) / (n - p + 1);
    double angle = 2 * M_PI * percent_circle;
    control_points.push_back(Eigen::Vector2d{std::cos(angle), std::sin(angle)});
  }

  for (int i = 0; i < m + 1; i++) {
    knot_sequence.push_back(static_cast<double>(i) / m);
  }


  auto out = BSpline::knot_points(p, control_points, knot_sequence);

  ASSERT_EQ(out.size(), m + 1);
  // Note: domain of curve is u_p to u_n-p
  // This is because the spline is "open" (no duplicated end control points)
  // which means the ends of the knot points don't have enough
  // coverage by the basis functions
  // https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-curve-open.html
  for (int i = p; i < n - p + 1; i++) {
    const auto point = out.at(i);
    EXPECT_NEAR(point.norm(), 1, 1e-3);
  }
}
