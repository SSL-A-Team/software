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

#ifndef CORE__SPATIAL__LAYERS__ROBOT_SHADOWS_HPP_
#define CORE__SPATIAL__LAYERS__ROBOT_SHADOWS_HPP_

#include <utility>
#include <vector>
#include <ateam_geometry/types.hpp>
#include "core/types/robot.hpp"
#include "core/types/world.hpp"

namespace ateam_kenobi::spatial::layers
{

ateam_geometry::Ray GetRobotTangentRay(
  const Robot & robot, const ateam_geometry::Point & source,
  const CGAL::Orientation & orientation);

std::pair<ateam_geometry::Ray, ateam_geometry::Ray> GetRobotShadowRays(
  const Robot & robot,
  const ateam_geometry::Point & source);
std::pair<ateam_geometry::Ray, ateam_geometry::Ray> GetRobotShadowRays(
  const Robot & robot,
  const ateam_geometry::Segment & source);

std::vector<ateam_geometry::Point> GetRobotShadowPoly(
  const Robot & robot,
  const ateam_geometry::Point & source, const World & world);
std::vector<ateam_geometry::Point> GetRobotShadowPoly(
  const Robot & robot,
  const ateam_geometry::Segment & source, const World & world);

}  // namespace ateam_kenobi::spatial::layers

#endif  // CORE__SPATIAL__LAYERS__ROBOT_SHADOWS_HPP_
