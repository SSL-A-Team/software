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

#ifndef TYPES__FIELD_HPP_
#define TYPES__FIELD_HPP_

#include <Eigen/Dense>
#include <array>

#include <ateam_geometry/ateam_geometry.hpp>

struct FieldSidedInfo
{
  // could be non axis aligned which was the whole point so didnt want to use our types yet
  std::array<Eigen::Vector2d, 4> goalie_box;
  std::array<Eigen::Vector2d, 4> goal;
};
struct Field
{
  // we will definetly change the format of this at some point this is preliminary
  // since we dont really have a geometry library yet
  // all of this should be in our coordinate system
  // There is a mirroring ros message of this type
  float field_length;
  float field_width;
  float goal_width;
  float goal_depth;
  float boundary_width;
  ateam_geometry::Circle center_circle;
  std::array<Eigen::Vector2d, 4> field_corners;
  FieldSidedInfo ours;
  FieldSidedInfo theirs;
};


#endif  // TYPES__FIELD_HPP_
