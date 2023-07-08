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


#ifndef MOTION__WORLD_TO_BODY_VEL_HPP_
#define MOTION__WORLD_TO_BODY_VEL_HPP_

#include <CGAL/Aff_transformation_2.h>
#include <ateam_geometry/ateam_geometry.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "types/robot.hpp"

namespace ateam_kenobi::motion
{

inline void ConvertWorldVelsToBodyVels(
  ateam_msgs::msg::RobotMotionCommand & command,
  const Robot & robot)
{
  ateam_geometry::Vector velocity(command.twist.linear.x, command.twist.linear.y);
  CGAL::Aff_transformation_2<ateam_geometry::Kernel> transformation(CGAL::ROTATION,
    std::sin(-robot.theta), std::cos(-robot.theta));
  velocity = velocity.transform(transformation);
  command.twist.linear.x = velocity.x();
  command.twist.linear.y = velocity.y();
}

inline void ConvertWorldVelsToBodyVels(
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & commands,
  const std::array<std::optional<Robot>, 16> & robots)
{
  for (auto i = 0u; i < 16; ++i) {
    auto & maybe_command = commands.at(i);
    const auto & maybe_robot = robots.at(i);
    if (maybe_command && maybe_robot) {
      ConvertWorldVelsToBodyVels(maybe_command.value(), maybe_robot.value());
    }
  }
}

}  // namespace ateam_kenobi::motion

#endif  // MOTION__WORLD_TO_BODY_VEL_HPP_
