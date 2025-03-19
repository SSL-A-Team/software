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
#include <ateam_spatial/spatial_evaluator.hpp>

TEST(SpatialEvaluatorTests, Basic)
{
  using ateam_spatial::SpatialEvaluator;
  using ateam_spatial::MapId;
  SpatialEvaluator eval;

  ateam_spatial::FieldDimensions field;
  field.field_length = 16.0;
  field.field_width = 9.0;
  field.boundary_width = 0.3;
  field.defense_area_width = 2.0;
  field.defense_area_depth = 1.0;
  field.goal_width = 1.0;
  field.goal_depth = 0.1;

  ateam_spatial::Ball ball;

  std::array<ateam_spatial::Robot, 16> our_bots;

  std::array<ateam_spatial::Robot, 16> their_bots;

  eval.UpdateMaps(field, ball, our_bots, their_bots);

  std::vector<float> buffer_out;

  eval.CopyMapBuffer(MapId::ReceiverPositionQuality, buffer_out);

  EXPECT_EQ(buffer_out.size(), 1'593'600);

  std::vector<uint8_t> rendered_buffer;
  eval.RenderMapBuffer(MapId::ReceiverPositionQuality, rendered_buffer);

  EXPECT_EQ(rendered_buffer.size(), 1'593'600);
}
