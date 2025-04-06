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

#ifndef ATEAM_SPATIAL__TYPES_HPP_
#define ATEAM_SPATIAL__TYPES_HPP_

namespace ateam_spatial
{

struct Point
{
  float x;
  float y;
};

struct Robot
{
  bool visible = false;
  float x = 0.0;
  float y = 0.0;
  float theta = 0.0;
  float x_vel = 0.0;
  float y_vel = 0.0;
  float theta_vel = 0.0;
};

struct Ball
{
  float x = 0.0;
  float y = 0.0;
  float x_vel = 0.0;
  float y_vel = 0.0;
};

struct FieldDimensions
{
  float field_width = 0.0;
  float field_length = 0.0;
  float goal_width = 0.0;
  float goal_depth = 0.0;
  float boundary_width = 0.0;
  float defense_area_width = 0.0;
  float defense_area_depth = 0.0;

  bool operator==(const FieldDimensions & other) const
  {
    const float kEpsilon = 1e-3;
    auto near_eq = [&kEpsilon](const float & a, const float & b) {
        return std::abs(a - b) < kEpsilon;
      };
    return near_eq(field_width, other.field_width) &&
           near_eq(field_length, other.field_length) &&
           near_eq(goal_width, other.goal_width) &&
           near_eq(goal_depth, other.goal_depth) &&
           near_eq(boundary_width, other.boundary_width) &&
           near_eq(defense_area_width, other.defense_area_width) &&
           near_eq(defense_area_depth, other.defense_area_depth);
  }

  bool operator!=(const FieldDimensions & other) const
  {
    return !operator==(other);
  }
};

struct SpatialSettings
{
  float resolution = 0.0;  // m / px
  std::size_t width = 0;
  std::size_t height = 0;
};

}  // namespace ateam_spatial

#endif  // ATEAM_SPATIAL__TYPES_HPP_
