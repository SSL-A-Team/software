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

#include "trajectory_generation/trajectory_editor.hpp"

#include <gtest/gtest.h>

TEST(trajectory_editor, crop_trajectory_ShouldSame_WhenBoundsInclude)
{
  Trajectory t;
  for (double i = 0; i < 100; i++) {
    t.samples.push_back({.time = i});
  }

  double start = -1;
  double end = 100;
  Trajectory ret = trajectory_editor::crop_trajectory(t, start, end);

  ASSERT_LE(start, ret.samples.front().time);
  ASSERT_GE(end, ret.samples.back().time);
}

TEST(trajectory_editor, crop_trajectory_ShouldLeftHalf_WhenBoundsPartial)
{
  Trajectory t;
  for (double i = 0; i < 100; i++) {
    t.samples.push_back({.time = i});
  }

  double start = -1;
  double end = 50;
  Trajectory ret = trajectory_editor::crop_trajectory(t, start, end);

  ASSERT_LE(start, ret.samples.front().time);
  ASSERT_GE(end, ret.samples.back().time);
}

TEST(trajectory_editor, crop_trajectory_ShouldRightHalf_WhenBoundsPartial)
{
  Trajectory t;
  for (double i = 0; i < 100; i++) {
    t.samples.push_back({.time = i});
  }

  double start = 25;
  double end = 100;
  Trajectory ret = trajectory_editor::crop_trajectory(t, start, end);

  ASSERT_LE(start, ret.samples.front().time);
  ASSERT_GE(end, ret.samples.back().time);
}

TEST(trajectory_editor, crop_trajectory_ShouldBoth_WhenBoundsPartial)
{
  Trajectory t;
  for (double i = 0; i < 100; i++) {
    t.samples.push_back({.time = i});
  }

  double start = 25;
  double end = 50;
  Trajectory ret = trajectory_editor::crop_trajectory(t, start, end);

  ASSERT_LE(start, ret.samples.front().time);
  ASSERT_GE(end, ret.samples.back().time);
}

TEST(trajectory_editor, crop_trajectory_ShouldNeither_WhenBoundsExclude)
{
  Trajectory t;
  for (double i = 0; i < 100; i++) {
    t.samples.push_back({.time = i});
  }

  double start = 125;
  double end = 150;
  Trajectory ret = trajectory_editor::crop_trajectory(t, start, end);

  ASSERT_TRUE(ret.samples.empty());
}

TEST(trajectory_editor, crop_trajectory_ShouldNothing_WhenNothingInput)
{
  Trajectory t;

  double start = 125;
  double end = 150;
  Trajectory ret = trajectory_editor::crop_trajectory(t, start, end);

  ASSERT_TRUE(ret.samples.empty());
}

TEST(trajectory_editor, append_trajectory_ShouldEmpty_WhenBothEmpty)
{
  Trajectory a;
  Trajectory b;

  Trajectory ret = trajectory_editor::append_trajectory(a, b);

  ASSERT_TRUE(ret.samples.empty());
}

TEST(trajectory_editor, append_trajectory_ShouldA_WhenBEmpty)
{
  Trajectory a;
  a.samples.push_back({.time = 0});
  Trajectory b;

  Trajectory ret = trajectory_editor::append_trajectory(a, b);

  ASSERT_EQ(ret.samples.size(), 1);
}

TEST(trajectory_editor, append_trajectory_ShouldB_WhenAEmpty)
{
  Trajectory a;
  Trajectory b;
  b.samples.push_back({.time = 0});

  Trajectory ret = trajectory_editor::append_trajectory(a, b);

  ASSERT_EQ(ret.samples.size(), 1);
}

TEST(trajectory_editor, append_trajectory_ShouldBoth_WhenBothFilled)
{
  Trajectory a;
  Trajectory b;
  a.samples.push_back({.time = 0});
  b.samples.push_back({.time = 0});

  Trajectory ret = trajectory_editor::append_trajectory(a, b);

  ASSERT_EQ(ret.samples.size(), 2);
}

TEST(trajectory_editor, append_trajectory_ShouldIncreaseInTime_WhenBothNoneZero)
{
  Trajectory a;
  Trajectory b;
  a.samples.push_back({.time = -10});
  a.samples.push_back({.time = -9});
  b.samples.push_back({.time = 8});
  b.samples.push_back({.time = 9});

  Trajectory ret = trajectory_editor::append_trajectory(a, b);

  ASSERT_EQ(ret.samples.size(), 4);
  EXPECT_EQ(ret.samples.at(0).time, -10);
  EXPECT_EQ(ret.samples.at(1).time, -9);
  EXPECT_EQ(ret.samples.at(2).time, 8);
  EXPECT_EQ(ret.samples.at(3).time, 9);
}
