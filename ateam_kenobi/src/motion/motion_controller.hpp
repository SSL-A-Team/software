// Copyright 2023 A Team
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

#ifndef MOTION__MOTION_CONTROLLER_HPP_
#define MOTION__MOTION_CONTROLLER_HPP_

#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_geometry/ateam_geometry.hpp>
#include <control_toolbox/pid.hpp>

#include "types/robot.hpp"
#include "types/world.hpp"


/**
 * Generate robot motion commands to follow given trajectory, may handle extra features such as pointing at a location
 */
class MotionController
{
public:
  MotionController();

  // Load a new trajectory into the motion controller resetting its progress along the old one
  void set_trajectory(const std::vector<ateam_geometry::Point>& trajectory);

  // Generate a robot motion command to follow a trajectory
  ateam_msgs::msg::RobotMotionCommand get_command(ateam_kenobi::Robot robot, double current_time);

  // Return a motion command to sit still
  ateam_msgs::msg::RobotMotionCommand empty_command();

  // Reset the PID controllers and remove previous time to recalculate dt
  void reset();

// Target velocity along the trajectory
double vel = 2;

// Velocity limits
double x_max = 2;
double y_max = 2;
double t_max = 2;

private:
  double prev_time;
  std::vector<ateam_geometry::Point> trajectory;

  int prev_point; // last point used in the trajectory
  double progress;
  double total_dist;

  // Might not actually be doing this although it could generate a nicer acceleration profile:
  // This controller acts on our progress along the trajectory
  // This enables it to smoothly ramp up to its velocity limit and then ramp back down at the end of the trajectory
  // while helping us choose what point on the trajectory to actually compare our position against
  //control_toolbox::Pid progress_controller;

  control_toolbox::Pid x_controller;
  control_toolbox::Pid y_controller;
  control_toolbox::Pid t_controller;
};

#endif  // MOTION__MOTION_CONTROLLER_HPP_
