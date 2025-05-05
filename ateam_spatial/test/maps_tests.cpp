// Copyright 2025 A Team
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
#include <ateam_spatial/maps/receiver_position_quality.hpp>

TEST(MapsTests, ReceiverPositionQuality)
{
  using ateam_spatial::maps::ReceiverPositionQuality;

  ateam_spatial::SpatialSettings settings;
  settings.resolution = 0.001;

  ateam_spatial::FieldDimensions field;
  field.field_length = 16.0;
  field.field_width = 9.0;
  field.boundary_width = 0.3;
  field.defense_area_width = 2.0;
  field.defense_area_depth = 1.0;
  field.goal_width = 1.0;
  field.goal_depth = 0.1;

  ateam_spatial::Ball ball{
    1.0,
    1.0,
    0.0,
    0.0
  };

  std::array<ateam_spatial::Robot, 16> their_robots;
  their_robots[0] = ateam_spatial::Robot{
    true,
    1.0,
    2.0,
    0.0,
    0.0,
    0.0,
    0.0
  };

  const auto kAcceptableError = 1e-4;
  EXPECT_FLOAT_EQ(ReceiverPositionQuality(0, 4800, ball, their_robots.data(), field, settings), 0.0);
  EXPECT_NEAR(ReceiverPositionQuality(600, 4800, ball, their_robots.data(), field, settings), 0.48, kAcceptableError);
  EXPECT_NEAR(ReceiverPositionQuality(8300, 4800, ball, their_robots.data(), field, settings), 8.3, kAcceptableError);
  EXPECT_NEAR(ReceiverPositionQuality(14500, 4800, ball, their_robots.data(), field, settings), 14.5, kAcceptableError);
  EXPECT_FLOAT_EQ(ReceiverPositionQuality(16600, 4800, ball, their_robots.data(), field, settings), 0.0);
  EXPECT_FLOAT_EQ(ReceiverPositionQuality(9300, 7000, ball, their_robots.data(), field, settings), 0.0);
}
