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


void MotionController::set_trajectory(const std::vector<ateam_geometry::Point> & trajectory)
{
  this->trajectory = trajectory;
  this->prev_index = 0;

  this->progress = 0;
  // TODO(anon): get total distance along the trajectory
  this->total_dist = 0;
}

double MotionController::calculate_trapezoidal_velocity(const ateam_kenobi::Robot& robot, double dt) {
  double accel_limit = 5.0;
  double decel_limit = 0.1;

  double vel = ateam_geometry::norm(robot.vel);
  //double vel = ateam_geometry::norm(this->prev_vel);
  double dist_to_stop = (vel*vel) / (2 * decel_limit);

  // Cruise
  double trapezoidal_vel = this->v_max;

  // Decelerate to a stop
  if (dist_to_stop >= ateam_geometry::norm(trajectory.back() - robot.pos)) {
    if (robot.id == 0) {
      // std::cerr << "DECEL" << std::endl;
    }

    trapezoidal_vel = vel - decel_limit * dt;
    
  // Accelerate to speed
  } else if (vel < this->v_max) {
    if (robot.id == 0) {
      // std::cerr << "ACCEL" << std::endl;
    }

    trapezoidal_vel = vel + (accel_limit * dt);
    if (trapezoidal_vel < 0.1) {
      trapezoidal_vel = 0.1;
    }
  } else {
    if (robot.id == 0) {
      // std::cerr << "CRUISE" << std::endl;
    }
  }

  return std::clamp(trapezoidal_vel, 0.0, this->v_max);
}

ateam_msgs::msg::RobotMotionCommand MotionController::get_command(
  const ateam_kenobi::Robot& robot,
  double current_time,
  const MotionOptions & options)
{
  ateam_msgs::msg::RobotMotionCommand motion_command;

  // Skip the first frame if we can't calculate dt or there isn't a trajectory
  if (std::isnan(this->prev_time) || this->trajectory.size() <= 0) {
    this->prev_time = current_time;
    return motion_command;
  }

  double dt = current_time - this->prev_time;
  uint64_t dt_nano = dt * 1e9;  // convert to nanoseconds

  // TODO(anon): figure out what point on the trajectory to use as the target
  uint64_t index;

  // find a point in the trajectory that is far enough away from our current location
  // for loop bound ensures index never exceeds the trajectory length even after the loop ends
  for (index = this->prev_index; index < this->trajectory.size() - 1; index++) {
    double dist = sqrt(CGAL::squared_distance(robot.pos, this->trajectory[index]));

    if (dist > this->v_max / this->pid_gains[0]) {
      break;
    }
  }

  // maybe do some sort of interpolation between points

  ateam_geometry::Point target = this->trajectory[index];

  /*
  if (smooth_position_target) {
    //double target_dist = this->v_max / this->pid_gains[0];

    double acc = 3.0; // m/s
    double target_dist = (ateam_geometry::norm(robot.vel) + (acc * dt)) / this->pid_gains[0];

    if (target_dist < 0.1) {
      target_dist = 0.1;
    }

    //double target_dist = ateam_geometry::norm(robot.vel) / this->pid_gains[0];
    auto target_vec = (target - robot.pos);

    if (target_dist < ateam_geometry::norm(target_vec)) {
      target = robot.pos + (target_dist * (target_vec / ateam_geometry::norm(target_vec)));
    }

    if (robot.id == 0) {
      std::cerr << "orig: " << ateam_geometry::norm(target_vec) << ", new: " << ateam_geometry::norm(target - robot.pos) << ", target dist: " << target_dist << std::endl;
    }
  }
  */

  /*
  if (abs(target.x() - prev_point.x()) < pid_reset_threshold) {
    this->x_controller.reset();
  }
  if (abs(target.y() - prev_point.y()) < pid_reset_threshold) {
    this->y_controller.reset();
  }
  */


  if (enable_trapezoidal) {
    double target_dist = calculate_trapezoidal_velocity(robot, dt);
    if (robot.id == 0) {
    }
    auto target_vec = (target - robot.pos);

    target = robot.pos + (target_dist * (target_vec / ateam_geometry::norm(target_vec)));

    /*
    if (target_dist < ateam_geometry::norm(target_vec)) {
      std::cerr << "APPLYING target velocity: " << target_dist << std::endl;
      target = robot.pos + (target_dist * (target_vec / ateam_geometry::norm(target_vec)));
    } else {
      std::cerr << "not applying" << std::endl;
    }
    */
  }

  prev_point = target;

  double dist = sqrt(CGAL::squared_distance(robot.pos, target));
  bool trajectory_complete = (dist <= options.completion_threshold);

  double x_error = target.x() - robot.pos.x();
  double y_error = target.y() - robot.pos.y();


  if (!trajectory_complete) {
    if (robot.id == 0) {
      std::cerr << "distance to target: " << dist << std::endl;
    }
    if (enable_trapezoidal) {
      double target_dist = calculate_trapezoidal_velocity(robot, dt);
      if (robot.id == 0) {
      }
      auto target_vec = (target - robot.pos);

      target = robot.pos + (target_dist * (target_vec / ateam_geometry::norm(target_vec)));
      x_error = target.x() - robot.pos.x();
      y_error = target.y() - robot.pos.y();

      /*
      if (target_dist < ateam_geometry::norm(target_vec)) {
        std::cerr << "APPLYING target velocity: " << target_dist << std::endl;
        target = robot.pos + (target_dist * (target_vec / ateam_geometry::norm(target_vec)));
      } else {
        std::cerr << "not applying" << std::endl;
      }
      */
    }

    double body_x_error = cos(robot.theta) * x_error - sin(robot.theta) * y_error;
    double body_y_error = sin(robot.theta) * x_error + cos(robot.theta) * y_error;

    // Calculate translational movement commands
    double x_command = this->x_controller.computeCommand(body_x_error, dt_nano);
    double y_command = this->y_controller.computeCommand(body_y_error, dt_nano);
    
    if (enable_min_speed_boost) {
      const float min_x = 0.2;
      const float min_y = 0.35;
      if (abs(x_command) < min_x && abs(y_command) < min_y) {
        if (body_x_error > sqrt(options.completion_threshold)) {
          x_command = (x_command > 0) ? min_x : -min_x;
          y_command = 0;
        } else if (body_y_error > sqrt(options.completion_threshold)){
          y_command = (y_command > 0) ? min_y : -min_y;
          x_command = 0;
        }
      }
    }

    double world_x_cmd = cos(-robot.theta) * x_command - sin(-robot.theta) * y_command;
    double world_y_cmd = sin(-robot.theta) * x_command + cos(-robot.theta) * y_command;
    auto vel_vector = ateam_geometry::Vector(world_x_cmd, world_y_cmd);

    // clamp to max velocity
    if (ateam_geometry::norm(vel_vector) > this->v_max) {
      vel_vector = this->v_max * ateam_geometry::normalize(vel_vector);
    }

    /*
    // clamp acceleration
    if (this->enable_acceleration_limits) {
      auto acc_vector = ateam_geometry::Vector(vel_vector - this->prev_vel);
      double acc_limit = 0.3 * dt;
      if (ateam_geometry::norm(acc_vector) > acc_limit) {
        acc_vector = acc_limit * ateam_geometry::normalize(acc_vector);
        vel_vector = this->prev_vel + acc_vector;
      }
    }
    */

    this->prev_vel = vel_vector;

    motion_command.twist.linear.x = vel_vector.x();
    motion_command.twist.linear.y = vel_vector.y();
  }

  // calculate angle movement commands
  double target_angle;

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
    /*
    if (abs(target_angle - prev_angle) < pid_reset_threshold) {
      this->t_controller.reset();
    }
    */
    prev_angle = target_angle;

    double t_error = angles::shortest_angular_distance(robot.theta, target_angle);
    double t_command = this->t_controller.computeCommand(t_error, dt_nano);
    motion_command.twist.angular.z = std::clamp(t_command, -this->t_max, this->t_max);
  }

  this->prev_index = index;
  this->prev_time = current_time;

  return motion_command;
}

void MotionController::reset()
{
  // TODO(anon): handle pid gains better
  this->x_controller.initPid(this->pid_gains[0], 0.0, 0.0, 0.3, -0.3, true);
  this->y_controller.initPid(this->pid_gains[1], 0.0, 0.0, 0.3, -0.3, true);
  this->t_controller.initPid(this->pid_gains[2], 0.0, 0.0, 0.5, -0.5, true);

  this->progress = 0;
  this->total_dist = 0;

  this->prev_time = NAN;

  this->angle_mode = AngleMode::face_travel;
  this->face_towards.reset();
  this->face_angle = 0;
}

void MotionController::set_x_pid_gain(GainType gain, double value)
{
  control_toolbox::Pid::Gains gains = this->x_controller.getGains();
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
  this->x_controller.setGains(gains);
}

void MotionController::set_y_pid_gain(GainType gain, double value)
{
  control_toolbox::Pid::Gains gains = this->y_controller.getGains();
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
  this->y_controller.setGains(gains);
}

void MotionController::set_t_pid_gain(GainType gain, double value)
{
  control_toolbox::Pid::Gains gains = this->t_controller.getGains();
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
  this->t_controller.setGains(gains);
}

void MotionController::set_x_pid_gains(double p, double i, double d)
{
  control_toolbox::Pid::Gains gains;
  gains.p_gain_ = p;
  gains.i_gain_ = i;
  gains.d_gain_ = d;
  this->x_controller.setGains(gains);
}

void MotionController::set_y_pid_gains(double p, double i, double d)
{
  control_toolbox::Pid::Gains gains;
  gains.p_gain_ = p;
  gains.i_gain_ = i;
  gains.d_gain_ = d;
  this->y_controller.setGains(gains);
}

void MotionController::set_t_pid_gains(double p, double i, double d)
{
  control_toolbox::Pid::Gains gains;
  gains.p_gain_ = p;
  gains.i_gain_ = i;
  gains.d_gain_ = d;
  this->t_controller.setGains(gains);
}
