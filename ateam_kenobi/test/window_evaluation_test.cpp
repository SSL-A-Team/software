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

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "core/play_helpers/window_evaluation.hpp"
#include <ateam_geometry_testing/testing_utils.hpp>


using namespace ateam_kenobi;  // NOLINT(build/namespaces)
using namespace ateam_kenobi::play_helpers;  // NOLINT(build/namespaces)

using testing::ElementsAre;
using testing::Optional;
using testing::Field;
using testing::Eq;

TEST(WindowEvaluationTest, EmptyRobots)
{
  std::vector<Robot> robots;
  ateam_geometry::Point source{0.0, 0.0};
  ateam_geometry::Segment target{
    ateam_geometry::Point{4.5, -0.5},
    ateam_geometry::Point{4.5, 0.5}
  };
  const auto windows = play_helpers::window_evaluation::getWindows(target, source, robots);

  EXPECT_THAT(windows, ElementsAre(SegmentIsNear(target)));
}

TEST(WindowEvaluationTest, OneRobot)
{
  std::vector<Robot> robots = {
    {1, true, true, ateam_geometry::Point(4.2, 0.0), 0.0, ateam_geometry::Vector{}, 0.0,
      ateam_geometry::Vector{}, 0.0, false,
      true,
      false}
  };
  ateam_geometry::Point source{3.3, 0.0};
  ateam_geometry::Segment target{
    ateam_geometry::Point{4.5, -1},
    ateam_geometry::Point{4.5, 1}
  };
  const auto windows = play_helpers::window_evaluation::getWindows(target, source, robots);

  EXPECT_THAT(
    windows,
    ElementsAre(
      SegmentIsNear(
        ateam_geometry::Segment{
    ateam_geometry::Point{4.5, -1},
    ateam_geometry::Point{4.5, -0.120605}}),
      SegmentIsNear(
        ateam_geometry::Segment{
    ateam_geometry::Point{4.5, 0.120605},
    ateam_geometry::Point{4.5, 1}})));
}
