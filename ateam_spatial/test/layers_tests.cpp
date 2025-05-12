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
#include <ateam_spatial/layers/distance_down_field.hpp>
#include <ateam_spatial/layers/distance_from_field_edge.hpp>
#include <ateam_spatial/layers/distance_to_their_bots.hpp>
#include <ateam_spatial/layers/in_field.hpp>
#include <ateam_spatial/layers/line_of_sight_ball.hpp>
#include <ateam_spatial/layers/outside_their_defense_area.hpp>
#include <ateam_spatial/layers/width_of_shot_on_goal.hpp>

TEST(LayersTests, DistanceDownField)
{
  using ateam_spatial::layers::DistanceDownField;

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

  EXPECT_FLOAT_EQ(DistanceDownField(-8.3, field, settings), 0.0);
  EXPECT_FLOAT_EQ(DistanceDownField(0.0, field, settings), 8.3);
  EXPECT_FLOAT_EQ(DistanceDownField(8.3, field, settings), 16.6);
}

TEST(LayersTests, DistanceFromFieldEdge)
{
  using ateam_spatial::layers::DistanceFromFieldEdge;

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

  const auto kAcceptableError = 1e-4;
  EXPECT_NEAR(DistanceFromFieldEdge(-7.8, 0.0, field, settings), 0.5, kAcceptableError);
  EXPECT_NEAR(DistanceFromFieldEdge(-7.8, -4.77, field, settings), 0.03, kAcceptableError);
  EXPECT_NEAR(DistanceFromFieldEdge(0.0, 0.0, field, settings), 4.8, kAcceptableError);
  EXPECT_NEAR(DistanceFromFieldEdge(7.7, 0.0, field, settings), 0.6, kAcceptableError);
  EXPECT_NEAR(DistanceFromFieldEdge(0.0, -3.8, field, settings), 1.0, kAcceptableError);
}

TEST(LayersTests, DistanceToTheirBots)
{
  using ateam_spatial::layers::DistanceToTheirBots;

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
  their_robots[1] = ateam_spatial::Robot{
    true,
    4.5,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  };

  const auto kAcceptableError = 1e-4;
  EXPECT_NEAR(DistanceToTheirBots(0.0, 0.0, their_robots.data(), field, settings), 2.236, kAcceptableError);
  EXPECT_NEAR(DistanceToTheirBots(4.0, 0.0, their_robots.data(), field, settings), 0.5, kAcceptableError);
}

TEST(LayersTests, InField)
{
  using ateam_spatial::layers::InField;

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

  EXPECT_FLOAT_EQ(InField(-8.3, 0.0, field, settings), 0.0);
  EXPECT_FLOAT_EQ(InField(-7.999, 0.0, field, settings), 1.0);
  EXPECT_FLOAT_EQ(InField(0.0, 0.0, field, settings), 1.0);
  EXPECT_FLOAT_EQ(InField(7.999, 0.0, field, settings), 1.0);
  EXPECT_FLOAT_EQ(InField(8.3, 0.0, field, settings), 0.0);
  EXPECT_FLOAT_EQ(InField(0.0, -4.8, field, settings), 0.0);
  EXPECT_FLOAT_EQ(InField(0.0, -4.499, field, settings), 1.0);
  EXPECT_FLOAT_EQ(InField(0.0, 0.0, field, settings), 1.0);
  EXPECT_FLOAT_EQ(InField(0.0, 4.499, field, settings), 1.0);
  EXPECT_FLOAT_EQ(InField(0.0, 4.8, field, settings), 0.0);
}

TEST(LayersTests, LineOfSightBall)
{
  using ateam_spatial::layers::LineOfSightBall;

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

  EXPECT_FLOAT_EQ(LineOfSightBall(0.0, 0.0, ball, their_robots.data(), field, settings), 1.0);
  EXPECT_FLOAT_EQ(LineOfSightBall(1.0, 2.2, ball, their_robots.data(), field, settings), 0.0);
  EXPECT_FLOAT_EQ(LineOfSightBall(1.0, 0.2, ball, their_robots.data(), field, settings), 1.0);
}

TEST(LayersTests, OutsideTheirDefenseArea)
{
  using ateam_spatial::layers::OutsideTheirDefenseArea;

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

  EXPECT_FLOAT_EQ(OutsideTheirDefenseArea(-8.3, -4.8, field, settings), 1.0);
  EXPECT_FLOAT_EQ(OutsideTheirDefenseArea(7.199, 0.0, field, settings), 0.0);
  EXPECT_FLOAT_EQ(OutsideTheirDefenseArea(7.199, -3.8, field, settings), 1.0);
  EXPECT_FLOAT_EQ(OutsideTheirDefenseArea(7.199, 1.2, field, settings), 1.0);
  EXPECT_FLOAT_EQ(OutsideTheirDefenseArea(-7.8, 0.0, field, settings), 1.0);
}


TEST(LayersTests, WidthOfShotOnGoal)
{
  using ateam_spatial::layers::WidthOfShotOnGoal;

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

  std::array<ateam_spatial::Robot, 16> their_robots;
  their_robots[0] = ateam_spatial::Robot{
    true,
    -1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  };

  const auto kAcceptableError = 1e-2;
  EXPECT_NEAR(WidthOfShotOnGoal(0.0, 0.0, their_robots.data(), field, settings), 1.0, kAcceptableError);
  their_robots[0].x = 0.2;
  their_robots[0].x = 0.0;
  EXPECT_NEAR(WidthOfShotOnGoal(0.0, 0.0, their_robots.data(), field, settings), 0.0, kAcceptableError);
  their_robots[0].x = 0.2;
  their_robots[0].y = 0.09;
  EXPECT_NEAR(WidthOfShotOnGoal(0.0, 0.0, their_robots.data(), field, settings), 0.5, kAcceptableError);
  their_robots[0].x = 0.2;
  their_robots[0].y = -0.09;
  EXPECT_NEAR(WidthOfShotOnGoal(0.0, 0.0, their_robots.data(), field, settings), 0.5, kAcceptableError);
  their_robots[0].x = 4.5;
  their_robots[0].y = 0.18;
  EXPECT_NEAR(WidthOfShotOnGoal(0.0, 0.0, their_robots.data(), field, settings), 0.65, kAcceptableError);
}
