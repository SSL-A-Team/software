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

#include "trajectory_generation/b_spline_wrapper.hpp"

#include <gtest/gtest.h>

TEST(b_spline_wrapper, basic_test)
{
  std::vector<Eigen::Vector2d> waypoints{
    Eigen::Vector2d{0, 0},
    Eigen::Vector2d{5, 3},
    Eigen::Vector2d{6, 2},
    Eigen::Vector2d{15, 3},
  };
  auto out = BSplineWrapper::Generate(
    waypoints, 0, 1, Eigen::Vector3d{0, 0, 0}, Eigen::Vector3d{0,
      0, 0}, Eigen::Vector3d{2, 2, 2}, Eigen::Vector3d{3, 3, 3},
    0.1, 1);

  std::cout << "output" << std::endl;
  for (int i = 0; i < out.samples.size(); i++) {
    std::cout << out.samples.at(i).time << "\t" << out.samples.at(i).pose.x() << " " <<
      out.samples.at(i).pose.y() << " " << out.samples.at(i).pose.z() << "\t" <<
      out.samples.at(i).vel.x() << " " << out.samples.at(i).vel.y() << " " <<
      out.samples.at(i).vel.z() << std::endl;
  }
}
