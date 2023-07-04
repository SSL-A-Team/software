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

#include "motion/motion_controller.hpp"

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_common/parameters.hpp>

#include <vector>
#include <algorithm>
#include <angles/angles.h>
#include <cmath>

#include "ateam_geometry/ateam_geometry.hpp"
#include "control_toolbox/pid.hpp"
#include "CGAL/squared_distance_2.h"

/*
// PID gains
CREATE_PARAM(double, "motion/pid/x_kp", x_kp, 2);
CREATE_PARAM(double, "motion/pid/y_kp", y_kp, 2);
CREATE_PARAM(double, "motion/pid/t_kp", t_kp, 3);

// PID velocity limits
CREATE_PARAM(double, "motion/pid/x_max", x_max, 2);
CREATE_PARAM(double, "motion/pid/y_max", y_max, 2);
CREATE_PARAM(double, "motion/pid/t_max", t_max, 4);
*/

void MotionController::set_trajectory(const std::vector<ateam_geometry::Point>&  trajectory) {
  this->trajectory = trajectory;
  this->prev_point = 0;

  this->progress = 0;
  // TODO: get total distance along the trajectory
  this->total_dist = 0;
}

ateam_msgs::msg::RobotMotionCommand MotionController::get_command(ateam_kenobi::Robot robot, double current_time)
{
  ateam_msgs::msg::RobotMotionCommand motion_command;

  // Skip the first frame if we can't calculate dt yet
  if (std::isnan(this->prev_time)) {
    this->prev_time = current_time;
    return motion_command;
  }

  // figure out how to convert this to ROS uint64
  //double dt = current_time - this->prev_time;
  double dt = 0.01; // 10ms in seconds
  uint64_t duration = 10000000; // 10 ms in nanoseconds

  // TODO: figure out what point on the trajectory to use as the target
  uint64_t index;

  // find a point in the trajectory that is far enough away from our current location
  for(index = this->prev_point; index < trajectory.size(); index++) {
    double dist = sqrt(CGAL::squared_distance(robot.pos, trajectory[index]));

    if (dist > this->vel * dt) {
      break;
    }
  }

  // maybe do some sort of interpolation between points

  ateam_geometry::Point target = trajectory[index];
  // double target_angle = 0; // TODO: get a target theta

  double x_error = target.x() - robot.pos.x();
  double x_command = this->x_controller.computeCommand(x_error, duration);
  
  double y_error = target.y() - robot.pos.y();
  double y_command = this->y_controller.computeCommand(y_error, duration);

  //double t_error = angles::shortest_angular_distance(robot.theta, target_angle);
  double t_error = 0; // TODO: Decide how to generate a target theta
  double t_command = this->t_controller.computeCommand(t_error, duration);

  // This clamping might be able to cause some weird issues with changing the angle of our velocity vector
  motion_command.twist.linear.x = std::clamp(x_command, -this->x_max, this->x_max);
  motion_command.twist.linear.y = std::clamp(y_command, -this->y_max, this->y_max);
  motion_command.twist.angular.z = std::clamp(t_command, -this->t_max, this->t_max);

  this->prev_point = index;
  this->prev_time = current_time;

  return motion_command;
}

ateam_msgs::msg::RobotMotionCommand MotionController::empty_command()
{
  ateam_msgs::msg::RobotMotionCommand motion_command;
  return motion_command;
}

void MotionController::reset()
{
  // TODO: handle pid gains better
  this->x_controller.initPid(1.0, 0, 0, 0, 0);
  this->y_controller.initPid(1.0, 0, 0, 0, 0);
  this->t_controller.initPid(1.0, 0, 0, 0, 0);

  this->progress = 0;
  this->total_dist = 0;

  this->prev_time = NAN;
}