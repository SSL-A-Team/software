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
#include <limits>
#include <vector>
#include <utility>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_common/parameters.hpp>
#include "ateam_geometry/ateam_geometry.hpp"
#include "ateam_common/robot_constants.hpp"
#include "pid.hpp"

namespace ateam_kenobi::motion {

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


void MotionController::update_trajectory(
  const std::vector<ateam_geometry::Point> & trajectory,
  ateam_geometry::Vector target_velocity)
{
  this->trajectory = trajectory;
  this->target_velocity = target_velocity;
}

void MotionController::reset_trajectory(
  const std::vector<ateam_geometry::Point> & trajectory,
  ateam_geometry::Vector target_velocity)
{
  this->trajectory = trajectory;
  this->target_velocity = target_velocity;

  if (trajectory.size()) {
    this->target_point = trajectory[0];
  }
}

void MotionController::calculate_trajectory_velocity_limits()
{
  trajectory_velocity_limits.reserve(trajectory.size());

  // Generate a maximum allowed speed at each trajectory point
  // for the trajectory segment that it starts.
  // Must meet two requirements at each trajectory point:
  //  1. Be slow enough to decelerate to the next point's limit by the time the robot reaches it
  //  2. Be slow enough to reasonably turn as sharply as required at the current point

  trajectory_velocity_limits[trajectory.size() - 1] = target_velocity;
  for (int i = trajectory.size() - 2; i >= 0; i--) {
    const ateam_geometry::Point point = trajectory[i];

    const ateam_geometry::Point next_point = trajectory[i + 1];
    const ateam_geometry::Vector next_vel = trajectory_velocity_limits[i + 1];

    const ateam_geometry::Vector direction = ateam_geometry::normalize(next_point - point);

    // Max velocity robot could linearly decelerate to the next velocity from
    double distance = ateam_geometry::norm(next_point - point);
    double max_decel_velocity = sqrt(std::pow(ateam_geometry::norm(next_vel),
      2) + 2 * decel_limit * distance);

    // Max velocity robot can make turn (also have to account for next_vel being (0,0))
    double max_turn_velocity = v_max;
    if (i > 0) {
      const ateam_geometry::Point prev_point = trajectory[i - 1];
      const ateam_geometry::Vector prev_direction = point - prev_point;

      double angle = ateam_geometry::ShortestAngleBetween(direction, prev_direction);
      max_turn_velocity = (abs(angle) > M_PI / 4.0) ? 0.5 : v_max;
    }

    double selected_velocity = std::clamp(std::min(max_decel_velocity, max_turn_velocity),
      0.0, v_max);

    trajectory_velocity_limits[i] = selected_velocity * direction;
  }
}

double MotionController::calculate_trapezoidal_velocity(
  const ateam_kenobi::Robot & robot,
  ateam_geometry::Point target, size_t target_index, double dt)
{
  // TODO(chachmu): make this smarter about the angle calculation
  // when we are off the trajectory

  double vel = ateam_geometry::norm(robot.prev_command_vel);

  const auto target_to_next_trajectory_point = trajectory[target_index] - target;
  double distance_to_next_trajectory_point =
    ateam_geometry::norm(target_to_next_trajectory_point);

  if (distance_to_next_trajectory_point == 0.0) {
    distance_to_next_trajectory_point = ateam_geometry::norm(target - robot.pos);
  }

  // Project target velocity onto the trajectory
  double target_vel = 0;
  if (ateam_geometry::norm(target_velocity) > 0.0) {
    target_vel = (target_velocity * target_to_next_trajectory_point) /
      ateam_geometry::norm(target_velocity);
  }

  double deceleration_to_reach_target = ((vel * vel) - (target_vel * target_vel)) /
    (2 * distance_to_next_trajectory_point);

  // Cruise
  double trapezoidal_vel = this->v_max;

  // Decelerate to target velocity
  if (deceleration_to_reach_target > decel_limit * 0.95) {
    trapezoidal_vel = vel - (deceleration_to_reach_target * dt);

  // Accelerate to speed
  } else if (vel < this->v_max) {
    trapezoidal_vel = vel + (accel_limit * dt);
  }

  return std::clamp(trapezoidal_vel, -this->v_max, this->v_max);
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
    dt = 1 / 100.0;  // TODO(chachmu): set this dynamically
  }

  size_t target_index = this->trajectory.size() - 1;
  ateam_geometry::Point target = this->trajectory[target_index];
  ateam_geometry::Vector target_direction = ateam_geometry::normalize(target - robot.pos);

  const double lookahead_distance = this->v_max * dt;
  const auto lookahead = ateam_geometry::makeCircle(robot.pos, lookahead_distance);

  u_int64_t closest_index = this->trajectory.size() - 1;
  ateam_geometry::Point closest_point = this->trajectory[closest_index];
  double closest_distance = ateam_geometry::norm(closest_point - robot.pos);

  // Only search if we aren't near the final target point of the trajectory
  if (ateam_geometry::norm(target - robot.pos) > lookahead_distance) {
    // This loop is calculating 3 things at the same time:
    //  1: Checking for the best target point using a lookahead distance
    //  2: Tracking the closest point on the trajectory in case the lookahead fails to find a target
    //  3: A unit vector pointing along the trajectory of the chosen target point
    for (int i = this->trajectory.size() - 1; i > 0; i--) {
      const auto a = this->trajectory[i - 1];
      const auto b = this->trajectory[i];

      const ateam_geometry::Segment s(a, b);

      // Check if lookahead lands on the trajectory
      const auto maybe_intersection = ateam_geometry::intersection(lookahead, s);
      using ptPair = std::pair<ateam_geometry::Point, ateam_geometry::Point>;
      if (maybe_intersection.has_value()) {
        if (std::holds_alternative<ateam_geometry::Point>(maybe_intersection.value())) {
          const auto intersection_point =
            std::get<ateam_geometry::Point>(maybe_intersection.value());
          target = intersection_point;

        } else if (std::holds_alternative<ptPair>(maybe_intersection.value())) {
          const auto intersection_pair = std::get<std::pair<ateam_geometry::Point,
              ateam_geometry::Point>>(maybe_intersection.value());

          // Pick the point further along the segment
          if ((b - a) * (intersection_pair.second - intersection_pair.first) > 0) {
            target = intersection_pair.second;
          } else {
            target = intersection_pair.first;
          }
        }

        // We've found the best target point
        target_index = i;
        target_direction = ateam_geometry::normalize(b - target);
        break;
      }

      const auto point = ateam_geometry::nearestPointOnSegment(s, robot.pos);
      const double segment_distance = ateam_geometry::norm(point - robot.pos);
      if (segment_distance < closest_distance) {
        closest_index = i;
        closest_distance = segment_distance;

        target = point;
        // This should only happen at the start of the trajectory
        if (target == a) {
          target_index = i - 1;
          target_direction = ateam_geometry::normalize(a - robot.pos);
        } else {
          target_index = i;
          target_direction = ateam_geometry::normalize(b - a);
        }
      }
    }
  }

  double distance_to_end = sqrt(CGAL::squared_distance(robot.pos, trajectory.back()));
  bool target_is_last_point = (target_index == this->trajectory.size() - 1);
  bool zero_target_vel = ateam_geometry::norm(target_velocity) < 0.01;
  bool trajectory_complete = (distance_to_end <= options.completion_threshold) &&
    target_is_last_point &&
    zero_target_vel;

  if (!trajectory_complete) {
    auto trajectory_line = ateam_geometry::Segment(trajectory[target_index],
      robot.pos).supporting_line();
    if (target_index > 0) {
      trajectory_line = ateam_geometry::Segment(trajectory[target_index],
        trajectory[target_index - 1]).supporting_line();
    }

    ateam_geometry::Vector error = trajectory[target_index] - robot.pos;
    CGAL::Aff_transformation_2<ateam_geometry::Kernel> transformation(CGAL::ROTATION,
      std::sin(-robot.theta), std::cos(-robot.theta));
    const auto error_body_frame = error.transform(transformation);

    ateam_geometry::Vector cross_track_error = trajectory_line.projection(robot.pos) - robot.pos;
    // If the line segment is degenerate then the projection can have nan values
    if (std::isnan(cross_track_error.x()) || std::isnan(cross_track_error.y())) {
      cross_track_error = ateam_geometry::Vector(0, 0);
    }
    // ateam_geometry::Vector along_track_error = error - cross_track_error;

    // Calculate pid feedback (in robot body frame)
    double cross_track_feedback = this->cross_track_controller.compute_command(
      ateam_geometry::norm(cross_track_error), dt);
    double x_feedback = this->x_controller.compute_command(error_body_frame.x(), dt);
    double y_feedback = this->y_controller.compute_command(error_body_frame.y(), dt);
    ateam_geometry::Vector body_feedback = ateam_geometry::Vector(x_feedback, y_feedback);
    const auto world_feedback = body_feedback.transform(transformation.inverse());

    // Calculate trapezoidal velocity feedforward
    calculate_trajectory_velocity_limits();
    double calculated_velocity = calculate_trapezoidal_velocity(robot, target, target_index, dt);

    ateam_geometry::Vector vel_vector;

    bool should_use_full_pid_control = target_is_last_point && zero_target_vel &&
      distance_to_end < 3.0 * kRobotRadius;

    if (should_use_full_pid_control) {
      vel_vector = world_feedback;
    } else {
      vel_vector = cross_track_feedback * ateam_geometry::normalize(cross_track_error) +
        (calculated_velocity * target_direction);
    }

    // clamp to max/min velocity
    if (ateam_geometry::norm(vel_vector) > this->v_max) {
      vel_vector = this->v_max * ateam_geometry::normalize(vel_vector);
    }

    motion_command.twist.linear.x = vel_vector.x();
    motion_command.twist.linear.y = vel_vector.y();
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
      if (trajectory_complete) {
        target_angle = robot.theta;
      } else {
        target_angle = atan2(target_direction.y(), target_direction.x());
      }
      break;
  }

  if (this->angle_mode != AngleMode::no_face) {
    double t_error = angles::shortest_angular_distance(robot.theta, target_angle);
    if (std::abs(t_error) > options.angular_completion_threshold) {
      double t_command = this->t_controller.compute_command(t_error, dt);
      if (trajectory_complete) {
        double theta_min = 0.0;
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
  }

  // Rotate the commanded vector to account for delay
  // if (abs(robot.prev_command_omega) > 0.5) {
  //   double angle_offset = -robot.prev_command_omega * 4 * dt;
  //   const auto cmd_vel = ateam_geometry::Vector(motion_command.twist.linear.x,
  //     motion_command.twist.linear.y);
  //   motion_command.twist.linear.x = cmd_vel.x() * cos(angle_offset) - cmd_vel.y() *
  //     sin(angle_offset);
  //   motion_command.twist.linear.y = cmd_vel.x() * sin(angle_offset) + cmd_vel.y() *
  //     cos(angle_offset);
  // }

  this->target_point = target;
  this->prev_time = current_time;

  return motion_command;
}

void MotionController::reset()
{
  cross_track_controller.set_gains(3.0, 0.0, 0.005, 0.31, -0.31);
  x_controller.set_gains(3.0, 0.0, 0.2, 0.3, -0.3);
  y_controller.set_gains(5.5, 0.0, 0.0, 0.3, -0.3);
  t_controller.set_gains(3.5, 0.0, 0.05, 0.5, -0.5);

  this->target_point = ateam_geometry::Point(0, 0);

  this->prev_time = NAN;

  this->angle_mode = AngleMode::face_travel;
  this->face_towards.reset();
  this->face_angle = 0;
}

void MotionController::set_cross_track_pid_gains(
  double p, double i, double d, double i_max,
  double i_min)
{
  cross_track_controller.set_gains(p, i, d, i_max, i_min);
}

void MotionController::set_x_pid_gains(double p, double i, double d, double i_max, double i_min)
{
  x_controller.set_gains(p, i, d, i_max, i_min);
}

void MotionController::set_y_pid_gains(double p, double i, double d, double i_max, double i_min)
{
  y_controller.set_gains(p, i, d, i_max, i_min);
}

void MotionController::set_t_pid_gains(double p, double i, double d, double i_min, double i_max)
{
  t_controller.set_gains(p, i, d, i_max, i_min);
}

}  // namespace ateam_kenobi::motion
