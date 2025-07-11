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
#include "core/types/robot.hpp"
#include "core/types/world.hpp"
#include "pid.hpp"

#include "nlohmann/json.hpp" // REMOVE THIS

// cause the robot to: always face a point, face in the direction of travel, or stay facing the
// same direction
enum class AngleMode
{
  face_point,
  face_absolute,
  face_travel,
  no_face
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
  nlohmann::json debug_json; // REMOVE THIS
  MotionController();

  // Update the current trajectory (usually just moves the last point a bit)
  void update_trajectory(
    const std::vector<ateam_geometry::Point> & trajectory,
    ateam_geometry::Vector target_velocity = ateam_geometry::Vector(0, 0));
  // Load a new trajectory into the motion controller resetting its progress along the old one
  void reset_trajectory(
    const std::vector<ateam_geometry::Point> & trajectory,
    ateam_geometry::Vector target_velocity = ateam_geometry::Vector(0, 0));

  void face_point(std::optional<ateam_geometry::Point> point);
  void face_absolute(double angle);
  void face_travel();
  void no_face();

  void calculate_trajectory_velocity_limits();

  double calculate_trapezoidal_velocity(
    const ateam_kenobi::Robot & robot,
    ateam_geometry::Point target,
    size_t target_index,
    double dt);

  // Generate a robot motion command to follow a trajectory
  ateam_msgs::msg::RobotMotionCommand get_command(
    ateam_kenobi::Robot robot, double current_time,
    const MotionOptions & options = MotionOptions());

  // Reset the PID controllers and remove previous time to recalculate dt
  void reset();

  // Set gains for individual PID controllers
  void set_cross_track_pid_gains(double p, double i, double d, double i_max, double i_min);
  void set_x_pid_gains(double p, double i, double d, double i_max, double i_min);
  void set_y_pid_gains(double p, double i, double d, double i_max, double i_min);
  void set_t_pid_gains(double p, double i, double d, double i_max, double i_min);

  // Velocity limits
  double v_max = 2.0;
  double t_max = 2;

  // Acceleration limits
  double accel_limit = 3.0;
  double decel_limit = 2.0;

  double face_angle = 0;
  std::optional<ateam_geometry::Point> face_towards;

  ateam_geometry::Point target_point;

private:
  double prev_time;
  double prev_command_vel;
  double prev_command;
  std::vector<ateam_geometry::Point> trajectory;
  std::vector<ateam_geometry::Vector> trajectory_velocity_limits;
  ateam_geometry::Vector target_velocity;

  AngleMode angle_mode = AngleMode::face_travel;  // This mode should have the best performance

  PID cross_track_controller;

  PID x_controller;
  PID y_controller;
  PID t_controller;
};

#endif  // CORE__MOTION__MOTION_CONTROLLER_HPP_
