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
#include <ateam_spatial/layers/in_field.hpp>
#include <ateam_spatial/layers/outside_their_defense_area.hpp>

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

  EXPECT_FLOAT_EQ(DistanceDownField(0, field, settings), 0.0);
  EXPECT_FLOAT_EQ(DistanceDownField(8300, field, settings), 8.3);
  EXPECT_FLOAT_EQ(DistanceDownField(16600, field, settings), 16.6);
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
  EXPECT_NEAR(DistanceFromFieldEdge(500, 4800, field, settings), 0.5, kAcceptableError);
  EXPECT_NEAR(DistanceFromFieldEdge(500, 30, field, settings), 0.03, kAcceptableError);
  EXPECT_NEAR(DistanceFromFieldEdge(8300, 4800, field, settings), 4.8, kAcceptableError);
  EXPECT_NEAR(DistanceFromFieldEdge(16000, 4800, field, settings), 0.6, kAcceptableError);
  EXPECT_NEAR(DistanceFromFieldEdge(8300, 1000, field, settings), 1.0, kAcceptableError);
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

  EXPECT_FLOAT_EQ(InField(0, 4800, field, settings), 0.0);
  EXPECT_FLOAT_EQ(InField(301, 4800, field, settings), 1.0);
  EXPECT_FLOAT_EQ(InField(8300, 4800, field, settings), 1.0);
  EXPECT_FLOAT_EQ(InField(16299, 4800, field, settings), 1.0);
  EXPECT_FLOAT_EQ(InField(16600, 4800, field, settings), 0.0);
  EXPECT_FLOAT_EQ(InField(8300, 0, field, settings), 0.0);
  EXPECT_FLOAT_EQ(InField(8300, 301, field, settings), 1.0);
  EXPECT_FLOAT_EQ(InField(8300, 4800, field, settings), 1.0);
  EXPECT_FLOAT_EQ(InField(8300, 9299, field, settings), 1.0);
  EXPECT_FLOAT_EQ(InField(8300, 9600, field, settings), 0.0);
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

  EXPECT_FLOAT_EQ(OutsideTheirDefenseArea(0, 0, field, settings), 1.0);
  EXPECT_FLOAT_EQ(OutsideTheirDefenseArea(15500, 4800, field, settings), 0.0);
  EXPECT_FLOAT_EQ(OutsideTheirDefenseArea(15500, 1000, field, settings), 1.0);
  EXPECT_FLOAT_EQ(OutsideTheirDefenseArea(15500, 6000, field, settings), 1.0);
  EXPECT_FLOAT_EQ(OutsideTheirDefenseArea(500, 4800, field, settings), 1.0);
}
