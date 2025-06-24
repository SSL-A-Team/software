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

#include "motion_controller.hpp"

#include <angles/angles.h>
#include <CGAL/squared_distance_2.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_common/parameters.hpp>
#include "ateam_geometry/ateam_geometry.hpp"
#include "control_toolbox/pid.hpp"

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


MotionController::MotionController()
{
  this->reset();
}

// if the point isn't valid then fail over to face_travel
void MotionController::face_point(std::optional<ateam_geometry::Point> point)
{
  if (point.has_value()) {
    this->face_towards = point;
    this->angle_mode = AngleMode::face_point;
  } else {
    this->angle_mode = AngleMode::face_travel;
  }
}

void MotionController::face_travel()
{
  this->angle_mode = AngleMode::face_travel;
}

void MotionController::face_absolute(double angle)
{
  this->face_angle = angle;
  this->angle_mode = AngleMode::face_absolute;
}

void MotionController::no_face()
{
  this->angle_mode = AngleMode::no_face;
}


void MotionController::update_trajectory(const std::vector<ateam_geometry::Point> & trajectory,
  ateam_geometry::Vector target_velocity)
{
  this->trajectory = trajectory;
  this->target_velocity = target_velocity;
}

void MotionController::reset_trajectory(const std::vector<ateam_geometry::Point> & trajectory,
  ateam_geometry::Vector target_velocity)
{
  this->trajectory = trajectory;
  this->target_velocity = target_velocity;

  this->prev_point = 0;

  this->progress = 0;
  // TODO(anon): get total distance along the trajectory
  this->total_dist = 0;
}

double MotionController::calculate_trapezoidal_velocity(const ateam_kenobi::Robot& robot, double remaining_dist, double dt) {

  // TODO: because this uses vector norms it doesn't really handle when the target velocity towards the robot

  double vel = ateam_geometry::norm(robot.vel);
  // // Prefer to use the previously commanded velocity for smoothness unless it is very wrong
  // if (prev_command_vel < 1.2 * vel && prev_command_vel > 0.8 * prev_command_vel) {
  //   vel = this->prev_command_vel;
  // }

  vel = this->prev_command_vel;

  double target_vel = ateam_geometry::norm(target_velocity);
  double deceleration_to_reach_target = ((vel*vel) - (target_vel * target_vel)) / (2 * remaining_dist);

  // Cruise
  double trapezoidal_vel = this->v_max;

  // Decelerate to target velocity
  if (deceleration_to_reach_target > decel_limit * 0.95) {
    trapezoidal_vel = vel - (deceleration_to_reach_target * dt);

  // Accelerate to speed
  } else if (vel < this->v_max) {
    trapezoidal_vel = vel + (accel_limit * dt);
  }

  return std::clamp(trapezoidal_vel, 0.0, this->v_max);
}

ateam_msgs::msg::RobotMotionCommand MotionController::get_command(
  ateam_kenobi::Robot robot,
  double current_time,
  const MotionOptions & options)
{
  ateam_msgs::msg::RobotMotionCommand motion_command;

  // Skip if there isn't a trajectory
  if (this->trajectory.size() <= 0) {
    this->prev_time = current_time;
    return motion_command;
  }

  double dt = current_time - this->prev_time;
  // If we don't have a valid dt just assume we are running at standard loop rate
  if (std::isnan(this->prev_time)) {
    dt = 1/100.0; // TODO: set this dynamically
  }

  // I don't like having to iterate the trajectory twice but I'm not sure if there is a better way

  // Find the closest point on the trajectory
  u_int64_t closest_index = this->trajectory.size() - 1;
  double closest_distance = sqrt(CGAL::squared_distance(robot.pos, this->trajectory[closest_index]));
  for (int i = this->trajectory.size() - 2; i > 0; i--) {
    double dist_to_point = sqrt(CGAL::squared_distance(robot.pos, this->trajectory[i]));

    if (dist_to_point <= closest_distance) {
      closest_index = i;
      closest_distance = dist_to_point;
    }
  }

  // handle if the closest point is behind us
  if (closest_index < this->trajectory.size() - 1) {
    auto trajectory_segment_vector = trajectory[closest_index + 1] - trajectory[closest_index];
    auto robot_to_closest_vector = trajectory[closest_index] - robot.pos;

    // Use the next point in the trajectory instead
    if (trajectory_segment_vector * robot_to_closest_vector < 0) {
      closest_index++;
    }
  }

  // 1. Find target point that is farther ahead of the robot than the lookahead distance
  // 2. Calculate the total distance along the trajectory from the robot to the end point
  double remaining_dist_along_trajectory = 0.0;
  uint64_t target_index = this->trajectory.size() - 1;
  for (uint64_t i = closest_index; i < this->trajectory.size(); i++) {
    double dist_to_point = sqrt(CGAL::squared_distance(robot.pos, this->trajectory[i]));

    // Find the target point farther than the lookahead distance
    if (target_index == this->trajectory.size() - 1) {
      if (dist_to_point > this->v_max * dt) {
        // Should this target point be interpolated?
        target_index = i;
        remaining_dist_along_trajectory = dist_to_point;
      }

    // Add up the total distance remaining along the trajectory
    } else {
      remaining_dist_along_trajectory += ateam_geometry::norm(trajectory[i] - trajectory[i-1]);
    }
  }

  // target_index will either be a validly chosen point or the last point in the trajectory
  ateam_geometry::Point target = this->trajectory[target_index];

  double dist = sqrt(CGAL::squared_distance(robot.pos, target));
  bool trajectory_complete = (dist <= options.completion_threshold) && (target_index == this->trajectory.size() - 1);

  double x_error = target.x() - robot.pos.x();
  double y_error = target.y() - robot.pos.y();

  bool xy_slow = false;
  if (ateam_geometry::norm(target_velocity) > 0.01 || !trajectory_complete) {

    // Calculate translational movement commands
    // double x_feedback = this->x_controller.compute_command(x_error, dt);
    // double y_feedback = this->y_controller.compute_command(y_error, dt);

    // auto feedback_vector = ateam_geometry::Vector(x_feedback, y_feedback);

    auto vel_vector = (calculate_trapezoidal_velocity(robot, remaining_dist_along_trajectory, dt)
      * ateam_geometry::normalize(target - robot.pos));
      // + feedback_vector;

    // clamp to max/min velocity
    if (ateam_geometry::norm(vel_vector) > this->v_max) {
      vel_vector = this->v_max * ateam_geometry::normalize(vel_vector);
    }
    motion_command.twist.linear.x = vel_vector.x();
    motion_command.twist.linear.y = vel_vector.y();
    this->prev_command_vel = ateam_geometry::norm(vel_vector);
  } else {
    this->prev_command_vel = 0.0;
  }

  // calculate angle movement commands
  double target_angle = 0.0;

  switch (this->angle_mode) {
    case AngleMode::no_face:
      break;
    case AngleMode::face_absolute:
      target_angle = this->face_angle;
      break;
    case AngleMode::face_point:
      if (this->face_towards.has_value()) {
        target_angle = atan2(
          this->face_towards.value().y() - robot.pos.y(),
          this->face_towards.value().x() - robot.pos.x());
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
    double t_command = this->t_controller.compute_command(t_error, dt);

    if (trajectory_complete && xy_slow) {
      double theta_min = 0.6;
      if (abs(t_command) < theta_min) {
        if (t_command > 0) {
          t_command = std::clamp(t_command, theta_min, this->t_max);
        } else {
          t_command = std::clamp(t_command, -theta_min, -this->t_max);
        }
      }
    }
    motion_command.twist.angular.z = std::clamp(t_command, -this->t_max, this->t_max);
  }

  //this->prev_point = index;
  this->prev_time = current_time;

  return motion_command;
}

void MotionController::reset()
{
  // TODO(anon): handle pid gains better
  this->x_controller.initialize(0.1, 0.0, 0.002, 0.3, -0.3, true);
  this->y_controller.initialize(0.1, 0.0, 0.002, 0.15, -0.15, true);
  this->t_controller.initialize(2.5, 0.0, 0.0, 0.5, -0.5, true);

  this->progress = 0;
  this->total_dist = 0;

  this->prev_command_vel = 0.0;
  this->prev_time = NAN;

  this->angle_mode = AngleMode::face_travel;
  this->face_towards.reset();
  this->face_angle = 0;
}

void MotionController::set_x_pid_gain(GainType gain, double value)
{
  control_toolbox::Pid::Gains gains = this->x_controller.get_gains();
  switch (gain) {
    case GainType::p:
      gains.p_gain_ = value;
      break;
    case GainType::i:
      gains.i_gain_ = value;
      break;
    case GainType::d:
      gains.d_gain_ = value;
      break;
  }
  this->x_controller.set_gains(gains);
}

void MotionController::set_y_pid_gain(GainType gain, double value)
{
  control_toolbox::Pid::Gains gains = this->y_controller.get_gains();
  switch (gain) {
    case GainType::p:
      gains.p_gain_ = value;
      break;
    case GainType::i:
      gains.i_gain_ = value;
      break;
    case GainType::d:
      gains.d_gain_ = value;
      break;
  }
  this->y_controller.set_gains(gains);
}

void MotionController::set_t_pid_gain(GainType gain, double value)
{
  control_toolbox::Pid::Gains gains = this->t_controller.get_gains();
  switch (gain) {
    case GainType::p:
      gains.p_gain_ = value;
      break;
    case GainType::i:
      gains.i_gain_ = value;
      break;
    case GainType::d:
      gains.d_gain_ = value;
      break;
  }
  this->t_controller.set_gains(gains);
}

void MotionController::set_x_pid_gains(double p, double i, double d)
{
  control_toolbox::Pid::Gains gains;
  gains.p_gain_ = p;
  gains.i_gain_ = i;
  gains.d_gain_ = d;
  this->x_controller.set_gains(gains);
}

void MotionController::set_y_pid_gains(double p, double i, double d)
{
  control_toolbox::Pid::Gains gains;
  gains.p_gain_ = p;
  gains.i_gain_ = i;
  gains.d_gain_ = d;
  this->y_controller.set_gains(gains);
}

void MotionController::set_t_pid_gains(double p, double i, double d)
{
  control_toolbox::Pid::Gains gains;
  gains.p_gain_ = p;
  gains.i_gain_ = i;
  gains.d_gain_ = d;
  this->t_controller.set_gains(gains);
}
