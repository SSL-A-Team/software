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

#ifndef ROBOT_MANEUVERS_HPP_
#define ROBOT_MANEUVERS_HPP_

#include <angles/angles.h>
#include <ssl_league_protobufs/ssl_simulation_robot_control.pb.h>

#include <chrono>
#include <optional>
#include <vector>

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/game_state_robot.hpp>

#include <ateam_geometry/ateam_geometry.hpp>
#include "ateam_controls/ateam_controls.h"

namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers
{
class ManeuverExecutor {
public:
  ManeuverExecutor()
  {
    traj_params_ = ateam_controls_default_traj_params();
    prev_update_time_ = std::chrono::steady_clock::now();
  }

  void set_command(ateam_msgs::msg::RobotMotionCommand command)
  {
    command_ = command;
  }

  void execute_maneuver(
    RobotMoveCommand * robot_move_command,
    const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot);

  // Whether the trajectory reference has been seeded yet.
  bool is_initialized() const {return initialized_;}

  // Current trajectory position reference in the global field frame.
  double reference_x() const {return ref_state_.data[0];}
  double reference_y() const {return ref_state_.data[1];}
  double reference_theta() const {return ref_state_.data[2];}

private:
  // Hold the current pose with zero velocity / acceleration.
  void hold_maneuver(ateam_msgs::msg::GameStateRobot robot);

  // Generate the controls-repo trajectory for the current command, seeded from the
  // previous reference state (open-loop; no vision feedback, no replan-on-error).
  void plan_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg);
  // Advance the active trajectory one frame and sample it into ref_state_/ref_accel_.
  void tick_and_sample(float dt);
  // Open-loop forward integration of the reference for acceleration modes.
  void integrate_accel(const ateam_msgs::msg::Twist2D & global_acceleration, float dt);
  // Serialize the global reference into the simulator move command.
  void send_reference(RobotMoveCommand * robot_move_command);

  double get_dt();
  double get_yaw(geometry_msgs::msg::Pose pose);
  ateam_msgs::msg::Twist2D rotate_frame(ateam_msgs::msg::Twist2D input_frame, double angle);
  Vector6C_t seed_from_vision(ateam_msgs::msg::GameStateRobot robot);

  TrajectoryParams_t generate_trajectory_params(
    const ateam_msgs::msg::RobotMotionCommand & ros_msg);
  PivotParams_t generate_pivot_params(
    const ateam_msgs::msg::RobotMotionCommand & ros_msg);
  bool command_uses_pivot();

  ateam_msgs::msg::RobotMotionCommand command_;   // Current ros command, updates every frame
  bool initialized_ = false;

  // Global field-frame reference handed to the simulator each frame.
  Vector6C_t ref_state_{};   // [x, y, theta, vx, vy, vtheta]
  Vector3C_t ref_accel_{};   // [ax, ay, atheta]

  std::chrono::steady_clock::time_point prev_update_time_;

  BangBangTraj3D_t position_trajectory_;
  TrajectoryParams_t traj_params_;

  PivotTrajectory_t pivot_trajectory_;
  PivotParams_t pivot_params_;
};

}  // namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers

#endif  // ROBOT_MANEUVERS_HPP_
