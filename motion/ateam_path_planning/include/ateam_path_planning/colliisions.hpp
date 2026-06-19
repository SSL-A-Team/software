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

#ifndef ATEAM_PATH_PLANNING__COLLIISIONS_HPP_
#define ATEAM_PATH_PLANNING__COLLIISIONS_HPP_

#include <ateam_controls/ateam_controls.h>
#include <vector>
#include "obstacle.hpp"

namespace ateam_path_planning::collisions
{

std::optional<double> TimeToCollision(
  const BangBangTraj3D & trajectory,
  const Vector6C_t & start_state,
  const double & start_t,
  const std::vector<Obstacle> & obstacles,
  const double collision_check_resolution,
  const double footprint_inflation);

}  // namespace ateam_path_planning::collisions

#endif  // ATEAM_PATH_PLANNING__COLLIISIONS_HPP_
