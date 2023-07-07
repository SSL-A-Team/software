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


MotionController::MotionController() {
  this->reset();
}

// if the point isn't valid then fail over to face_travel
void MotionController::face_point(std::optional<ateam_geometry::Point> point) {
  if (point.has_value()) {
    this->face_towards = point;
    this->angle_mode = AngleMode::face_point;
  } else {
    this->angle_mode = AngleMode::face_travel;
  }
}

void MotionController::face_travel() {
  this->angle_mode = AngleMode::face_travel;
}

void MotionController::face_absolute(double angle){
  this->face_angle = angle;
  this->angle_mode = AngleMode::face_absolute;
}

void MotionController::no_face() {
  this->angle_mode = AngleMode::no_face;
}


void MotionController::set_trajectory(const std::vector<ateam_geometry::Point>&  trajectory) {
  this->trajectory = trajectory;
  this->prev_point = 0;

  this->progress = 0;
  // TODO: get total distance along the trajectory
  this->total_dist = 0;
}

ateam_msgs::msg::RobotMotionCommand MotionController::get_command(ateam_kenobi::Robot robot, double current_time, const MotionOptions & options)
{
  ateam_msgs::msg::RobotMotionCommand motion_command;

  // Skip the first frame if we can't calculate dt or there isn't a trajectory
  if (std::isnan(this->prev_time) || this->trajectory.size() <= 0) {
    this->prev_time = current_time;
    return motion_command;
  }

  double dt = current_time - this->prev_time;
  uint64_t dt_nano = dt * 1000000; // convert to nanoseconds

  // TODO: figure out what point on the trajectory to use as the target
  uint64_t index;

  // find a point in the trajectory that is far enough away from our current location
  // for loop bound ensures index never exceeds the length of the trajectory even after the loop ends
  for(index = this->prev_point; index < this->trajectory.size() - 1; index++) {
    double dist = sqrt(CGAL::squared_distance(robot.pos, this->trajectory[index]));

    if (dist > this->v_max * dt) {
      break;
    }
  }

  // maybe do some sort of interpolation between points

  ateam_geometry::Point target = this->trajectory[index];

  double dist = sqrt(CGAL::squared_distance(robot.pos, target));
  bool trajectory_complete = (dist <= options.completion_threshold);

  double x_error = target.x() - robot.pos.x();
  double y_error = target.y() - robot.pos.y();

  if (!trajectory_complete) {
    // Calculate translational movement commands
    double x_command = this->x_controller.computeCommand(x_error, dt_nano);
    double y_command = this->y_controller.computeCommand(y_error, dt_nano);

    auto vel_vector = ateam_geometry::Vector(x_command, y_command);

    // clamp to max velocity
    if (ateam_geometry::norm(vel_vector) > this->v_max) {
      vel_vector = this->v_max * ateam_geometry::normalize(vel_vector);
    }

    motion_command.twist.linear.x = vel_vector.x();
    motion_command.twist.linear.y = vel_vector.y();
  }

  // calculate angle movement commands
  double target_angle;

  switch(this->angle_mode) {
    case AngleMode::no_face:
      break;
    case AngleMode::face_absolute:
      target_angle = this->face_angle;
      break;
    case AngleMode::face_point:
      if (this->face_towards.has_value()) {
        target_angle = atan2(this->face_towards.value().y() - robot.pos.y(), this->face_towards.value().x() - robot.pos.x());
        break;
      }
      [[fallthrough]];
      // otherwise default to face travel
    case AngleMode::face_travel:
      target_angle = atan2(y_error, x_error);
      break;
  }

  if (this->angle_mode != AngleMode::no_face) {
    double t_error = angles::shortest_angular_distance(robot.theta, target_angle);
    double t_command = this->t_controller.computeCommand(t_error, dt_nano);
    motion_command.twist.angular.z = std::clamp(t_command, -this->t_max, this->t_max);
  }

  this->prev_point = index;
  this->prev_time = current_time;

  return motion_command;
}

void MotionController::reset()
{
  // TODO: handle pid gains better
  this->x_controller.initPid(3.0, 0, 0, 0, 0);
  this->y_controller.initPid(3.0, 0, 0, 0, 0);
  this->t_controller.initPid(5.0, 0, 0, 0, 0);

  this->progress = 0;
  this->total_dist = 0;

  this->prev_time = NAN;

  this->angle_mode = AngleMode::face_travel;
  this->face_towards.reset();
  this->face_angle = 0;
}