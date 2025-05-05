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
#include <ateam_spatial/coordinate_conversions.hpp>

TEST(CooridinateConversionsTests, BasicTest)
{
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

  EXPECT_FLOAT_EQ(WorldWidthReal(field), 16.6);
  EXPECT_FLOAT_EQ(WorldHeightReal(field), 9.6);
  EXPECT_EQ(WorldWidthSpatial(field, settings), 16600);
  EXPECT_EQ(WorldHeightSpatial(field, settings), 9600);
  EXPECT_FLOAT_EQ(SpatialToRealX(1000, field, settings), -7.3);
  EXPECT_FLOAT_EQ(SpatialToRealY(2000, field, settings), -2.8);
  EXPECT_EQ(RealToSpatialDist(0.1, settings), 100);
}
