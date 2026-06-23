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

#include <chrono>
#include <optional>
#include <vector>

#include <ssl_league_protobufs/ssl_simulation_robot_control.pb.h>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/game_state_robot.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>

#include <angles/angles.h>
#include <ateam_geometry/ateam_geometry.hpp>
#include "ateam_controls/ateam_controls.h"

namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers
{
  class ManeuverExecutor {

    public:
    ManeuverExecutor() {
      traj_params_ = ateam_controls_default_traj_params();
      prev_update_time_ = std::chrono::steady_clock::now();
    }

    void set_command(ateam_msgs::msg::RobotMotionCommand command) {
      command_ = command;
    }

    void execute_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot);

    private:
    void bcm_off_maneuver();

    // Trajectory Maneuvers
    void global_position_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot);

    // Global Maneuvers
    void global_velocity_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg);
    void global_acceleration_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg);

    // Local Maneuvers
    void local_velocity_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot);
    void local_acceleration_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot);

    void finalize_command(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot);

    ateam_msgs::msg::Twist2D apply_xy_motion_limits(ateam_msgs::msg::Twist2D prev, ateam_msgs::msg::Twist2D command, double vel_limit, double acc_limit, double dt);
    double apply_1d_motion_limits(double prev, double commanded, double vel_limit, double acc_limit, double dt);

    double get_dt();
    double get_yaw(geometry_msgs::msg::Pose pose);
    ateam_msgs::msg::Twist2D rotate_frame(ateam_msgs::msg::Twist2D input_frame, double angle);
    TrajectoryParams_t generate_trajectory_params(const ateam_msgs::msg::RobotMotionCommand & ros_msg);
    bool use_trajectory_angle();

    BangBangTraj3D_t trajectory_;
    TrajectoryParams_t traj_params_;
    ateam_msgs::msg::RobotMotionCommand command_; // Current ros command, updates with every new message

    ateam_msgs::msg::Twist2D current_global_command_; // Velocity to be commanded this frame
    ateam_msgs::msg::Twist2D prev_global_command_; // Velocity commanded previous frame
    std::chrono::steady_clock::time_point prev_update_time_;
  };

}  // namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers

#endif  // ROBOT_MANEUVERS_HPP_
