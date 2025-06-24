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

#ifndef CORE__MOTION__MOTION_CONTROLLER_HPP_
#define CORE__MOTION__MOTION_CONTROLLER_HPP_

#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_geometry/ateam_geometry.hpp>

// TODO(barulicm) Remove when control_toolbox no longer uses deprecated realtime_tools headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#include <control_toolbox/pid.hpp>
#pragma GCC diagnostic pop

#include "core/types/robot.hpp"
#include "core/types/world.hpp"

// cause the robot to: always face a point, face in the direction of travel, or stay facing the
// same direction
enum class AngleMode
{
  face_point,
  face_absolute,
  face_travel,
  no_face
};

enum class GainType
{
  p,
  i,
  d
};

struct MotionOptions
{
  /**
   * @brief radius around the end point that will be considered completed
   */
  double completion_threshold = .02;  // meters
};

/**
 * Generate robot motion commands to follow given trajectory, may handle extra features such as pointing at a location
 */
class MotionController
{
public:
  MotionController();

  // Update the current trajectory (usually just moves the last point a bit)
  void update_trajectory(const std::vector<ateam_geometry::Point> & trajectory,
    ateam_geometry::Vector target_velocity = ateam_geometry::Vector(0, 0));
  // Load a new trajectory into the motion controller resetting its progress along the old one
  void reset_trajectory(const std::vector<ateam_geometry::Point> & trajectory,
    ateam_geometry::Vector target_velocity = ateam_geometry::Vector(0, 0));

  void face_point(std::optional<ateam_geometry::Point> point);
  void face_absolute(double angle);
  void face_travel();
  void no_face();

  double calculate_trapezoidal_velocity(
    const ateam_kenobi::Robot& robot,
    double remaining_dist,
    double dt);

  // Generate a robot motion command to follow a trajectory
  ateam_msgs::msg::RobotMotionCommand get_command(
    ateam_kenobi::Robot robot, double current_time,
    const MotionOptions & options = MotionOptions());

  // Reset the PID controllers and remove previous time to recalculate dt
  void reset();
  // const auto distance_to_ball = CGAL::approximate_sqrt(CGAL::squared_distance(robot.pos,
  //     world.ball.pos));

  // const auto decel_distance = distance_to_ball - approach_radius_;

  // const auto max_decel_vel = std::sqrt((2.0 * decel_limit_ * decel_distance) +
  //     (capture_speed_ * capture_speed_));

  // easy_move_to_.setMaxVelocity(std::min(max_decel_vel, max_speed_));
  // Set gains for individual PID controllers
  void set_x_pid_gain(GainType gain, double value);
  void set_y_pid_gain(GainType gain, double value);
  void set_t_pid_gain(GainType gain, double value);
  void set_x_pid_gains(double p, double i, double d);
  void set_y_pid_gains(double p, double i, double d);
  void set_t_pid_gains(double p, double i, double d);

  // Velocity limits
  double v_max = 2.0;
  double t_max = 2;

  // Acceleration limits
  double accel_limit = 3.0;
  double decel_limit = 2.0;

  double face_angle = 0;
  std::optional<ateam_geometry::Point> face_towards;

private:
  double prev_time;
  double prev_command_vel;
  double prev_command;
  std::vector<ateam_geometry::Point> trajectory;
  ateam_geometry::Vector target_velocity;

  AngleMode angle_mode = AngleMode::face_travel;  // This mode should have the best performance

  int prev_point;  // last point used in the trajectory
  double progress;
  double total_dist;

  // Might not actually be doing this although it could generate a nicer acceleration profile:
  // This controller acts on our progress along the trajectory
  // This enables it to smoothly ramp up to its velocity limit and then ramp back down at the end
  // of the trajectory while helping us choose what point on the trajectory to actually compare
  // our position against control_toolbox::Pid progress_controller;

  control_toolbox::Pid x_controller;
  control_toolbox::Pid y_controller;
  control_toolbox::Pid t_controller;
};

#endif  // CORE__MOTION__MOTION_CONTROLLER_HPP_
