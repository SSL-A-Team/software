// Copyright 2026 A Team
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

#ifndef CORE__MOTION__PATH_PLANNING__PATH_PLANNING_TARGET_HPP_
#define CORE__MOTION__PATH_PLANNING__PATH_PLANNING_TARGET_HPP_

#include <vector>
#include <ateam_geometry/types.hpp>
#include <ateam_geometry/any_shape.hpp>
#include "planner_options.hpp"
#include "core/motion/motion_intent.hpp"

namespace ateam_kenobi::motion::path_planning
{

struct PathPlanningTarget
{
  int robot_id;
  ateam_geometry::Point position;
  double heading;
  PlannerOptions planner_options;
  std::vector<ateam_geometry::AnyShape> obstacles;
  motion::Limits limits;
};

}  // namespace ateam_kenobi::motion::path_planning


#endif  // CORE__MOTION__PATH_PLANNING__PATH_PLANNING_TARGET_HPP_
