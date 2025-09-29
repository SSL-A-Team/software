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

#ifndef ATEAM_GAME_STATE__FIELD_HPP_
#define ATEAM_GAME_STATE__FIELD_HPP_

#include <array>
#include <ateam_geometry/types.hpp>

namespace ateam_game_state
{
struct FieldSidedInfo
{
  std::array<ateam_geometry::Point, 4> defense_area_corners;
  std::array<ateam_geometry::Point, 4> goal_corners;
  std::array<ateam_geometry::Point, 2> goal_posts;
};
struct Field
{
  float field_length = 0.f;
  float field_width = 0.f;
  float goal_width = 0.f;
  float goal_depth = 0.f;
  float boundary_width = 0.f;
  float defense_area_width = 0.f;
  float defense_area_depth = 0.f;
  std::array<ateam_geometry::Point, 4> field_corners;
  int ignore_side = 0;
  FieldSidedInfo ours;
  FieldSidedInfo theirs;
};
}  // namespace ateam_kenobi

#endif  // ATEAM_GAME_STATE__FIELD_HPP_
