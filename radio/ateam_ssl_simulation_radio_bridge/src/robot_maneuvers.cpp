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

#include <cmath>
#include "robot_maneuvers.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers
{

void ManeuverExecutor::execute_maneuver(
  RobotMoveCommand * robot_move_command,
  const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot)
{
  switch(ros_msg.body_control_mode) {
    case ateam_msgs::msg::RobotMotionCommand::BCM_OFF:
      bcm_off_maneuver();
      return;  // Ignores acceleration limits set in message and immediately stops the robot

    // Trajectory Maneuvers
    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION:
    case ateam_msgs::msg::RobotMotionCommand::BCM_HEADING_PIVOT:
    case ateam_msgs::msg::RobotMotionCommand::BCM_POINT_PIVOT:
      trajectory_maneuver(ros_msg, robot);
      break;

    // Global Maneuvers
    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_VELOCITY:
      global_velocity_maneuver(ros_msg);
      break;
    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_ACCEL:
      global_acceleration_maneuver(ros_msg);
      break;

    // Local Maneuvers
    case ateam_msgs::msg::RobotMotionCommand::BCM_LOCAL_VELOCITY:
      local_velocity_maneuver(ros_msg, robot);
      break;
    case ateam_msgs::msg::RobotMotionCommand::BCM_LOCAL_ACCEL:
      local_acceleration_maneuver(ros_msg, robot);
      break;
  }

  finalize_command(robot_move_command, ros_msg, robot);
}

void ManeuverExecutor::bcm_off_maneuver()
{
  current_global_command_ = ateam_msgs::msg::Twist2D();
  prev_global_command_ = ateam_msgs::msg::Twist2D();
  prev_update_time_ = std::chrono::steady_clock::now();
}

// Trajectory Maneuvers
void ManeuverExecutor::trajectory_maneuver(
  const ateam_msgs::msg::RobotMotionCommand & ros_msg,
  ateam_msgs::msg::GameStateRobot robot)
{
  // Update trajectory state to current time
  float dt = static_cast<float>(get_dt());
  tick_trajectory(dt);
  Vector6C_t state = get_trajectory_state();

  double vision_yaw = get_yaw(robot.pose);

    // Check current position against predicted position
  double current_dist = std::hypot(robot.pose.position.x - state.data[0],
      robot.pose.position.y - state.data[1]);
  bool current_pos_needs_replan = current_dist > 0.5 ||
    angles::shortest_angular_distance(vision_yaw, state.data[2]) > 0.3;

  bool maneuver_command_changed = (command_ != ros_msg);
  bool replanning = maneuver_command_changed || current_pos_needs_replan;
  if (replanning) {
    Vector6C_t starting_state = state; // start from trajectory predicted state

    if (current_pos_needs_replan) {
      // Predicted robot state is inaccurate
      starting_state = Vector6C_t{
        static_cast<float>(robot.pose.position.x),
        static_cast<float>(robot.pose.position.y),
        static_cast<float>(vision_yaw),
        static_cast<float>(prev_global_command_.x),
        static_cast<float>(prev_global_command_.y),
        static_cast<float>(prev_global_command_.theta),
      };
    } else if (!command_uses_trajectory()) {
      // If previous command did not use trajectory controller use prev commanded velocities instead
      starting_state.data[3] = static_cast<float>(prev_global_command_.x);
      starting_state.data[4] = static_cast<float>(prev_global_command_.y);
      starting_state.data[5] = static_cast<float>(prev_global_command_.theta);
    }

    command_ = ros_msg;
    plan_trajectory(starting_state);
    state = get_trajectory_state();

    pid_x_traj_.reset();
    pid_y_traj_.reset();
    pid_theta_traj_.reset();

    pid_x_target_.reset();
    pid_y_target_.reset();
    pid_theta_target_.reset();
  }

  ateam_msgs::msg::Twist2D global_target_err = ateam_msgs::msg::Twist2D();
  global_target_err.x = command_.pose.x - robot.pose.position.x;
  global_target_err.y = command_.pose.y - robot.pose.position.y;
  global_target_err.theta = angles::shortest_angular_distance(get_yaw(robot.pose),
    command_.pose.theta);

  ateam_msgs::msg::Twist2D global_traj_err = ateam_msgs::msg::Twist2D();
  global_traj_err.x = state.data[0] - robot.pose.position.x;
  global_traj_err.y = state.data[1] - robot.pose.position.y;
  global_traj_err.theta = angles::shortest_angular_distance(get_yaw(robot.pose), state.data[2]);

  ateam_msgs::msg::Twist2D global_feedforward = ateam_msgs::msg::Twist2D();
  ateam_msgs::msg::Twist2D global_feedback = ateam_msgs::msg::Twist2D();

  // Default Control Behavior
  global_feedforward.x = state.data[3];
  global_feedforward.y = state.data[4];
  global_feedforward.theta = state.data[5];

  global_feedback.x = pid_x_traj_.compute_command(global_traj_err.x, dt);
  global_feedback.y = pid_y_traj_.compute_command(global_traj_err.y, dt);
  global_feedback.theta = pid_theta_traj_.compute_command(global_traj_err.theta, dt);

  // Custom trajectory control behavior
  switch(command_.body_control_mode) {
    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION:
      {
        global_feedback.theta = 0.0;

        // Might be better to check the error between the trajectory setpoint and the final target instead

        // Handle slight xy overshoot and final position offset due to vision filter delay
        const double linear_threshold = 0.01;
        const double angular_threshold = 0.1;

        if (abs(global_target_err.x) < linear_threshold) {
          global_feedforward.x = 0.0;
          global_feedback.x = pid_x_target_.compute_command(global_target_err.x, dt);
        }

        if (abs(global_target_err.y) < linear_threshold) {
          global_feedforward.y = 0.0;
          global_feedback.y = pid_y_target_.compute_command(global_target_err.y, dt);
        }

        if (abs(global_target_err.theta) < angular_threshold) {
          global_feedforward.theta = 0.0;
          global_feedback.theta = pid_y_target_.compute_command(global_target_err.theta, dt);
        }

        break;
      }

    case ateam_msgs::msg::RobotMotionCommand::BCM_HEADING_PIVOT:
    case ateam_msgs::msg::RobotMotionCommand::BCM_POINT_PIVOT:
      global_feedback.x = 0.0;
      global_feedback.y = 0.0;
      break;
  }

  current_global_command_.x = global_feedforward.x + global_feedback.x;
  current_global_command_.y = global_feedforward.y + global_feedback.y;
  current_global_command_.theta = global_feedforward.theta + global_feedback.theta;

  std::cerr << "theta: " << get_yaw(robot.pose) << ", final err: " << global_target_err.theta << ", traj err: " << global_traj_err.theta << std::endl;
}

// Global Maneuvers
void ManeuverExecutor::global_velocity_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg)
{
  current_global_command_ = ros_msg.velocity;
}

void ManeuverExecutor::global_acceleration_maneuver(
  const ateam_msgs::msg::RobotMotionCommand & ros_msg)
{
  const double dt = get_dt();

  current_global_command_.x = prev_global_command_.x + dt * ros_msg.acceleration.x;
  current_global_command_.y = prev_global_command_.y + dt * ros_msg.acceleration.y;
  current_global_command_.theta = prev_global_command_.theta + dt * ros_msg.acceleration.theta;
}

// Local Maneuvers
void ManeuverExecutor::local_velocity_maneuver(
  const ateam_msgs::msg::RobotMotionCommand & ros_msg,
  ateam_msgs::msg::GameStateRobot robot)
{
  current_global_command_ = rotate_frame(ros_msg.velocity, -get_yaw(robot.pose));
}

void ManeuverExecutor::local_acceleration_maneuver(
  const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot)
{
  ateam_msgs::msg::Twist2D global_accel = rotate_frame(ros_msg.acceleration, -get_yaw(robot.pose));
  const double dt = get_dt();

  current_global_command_.x = prev_global_command_.x + dt * global_accel.x;
  current_global_command_.y = prev_global_command_.y + dt * global_accel.y;
  current_global_command_.theta = prev_global_command_.theta + dt * global_accel.theta;
}

void ManeuverExecutor::finalize_command(
  RobotMoveCommand * robot_move_command,
  const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot)
{
  command_ = ros_msg;
  double dt = get_dt();

  ateam_msgs::msg::Twist2D local_command;
  if (command_uses_trajectory()) {
    // Motion limits for trajectory type maneuvers are mostly handled by the planner
    Vector6C_t state = get_trajectory_state();
    local_command = rotate_frame(current_global_command_, state.data[2]);
  } else {
    // Handle non-trajectory type maneuver motion limits
    traj_params_ = generate_trajectory_params(command_);
    current_global_command_ = apply_xy_motion_limits(prev_global_command_, current_global_command_,
        traj_params_.max_vel_linear, traj_params_.max_accel_linear, dt);
    current_global_command_.theta = apply_1d_motion_limits(prev_global_command_.theta,
        current_global_command_.theta, traj_params_.max_vel_angular, traj_params_.max_accel_angular,
        dt);

    local_command = rotate_frame(current_global_command_, get_yaw(robot.pose));
  }

  MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();
  local_velocity_command->set_forward(local_command.x);
  local_velocity_command->set_left(local_command.y);
  local_velocity_command->set_angular(local_command.theta);

  prev_global_command_ = current_global_command_;
  prev_update_time_ = std::chrono::steady_clock::now();
}

ateam_msgs::msg::Twist2D ManeuverExecutor::apply_xy_motion_limits(
  ateam_msgs::msg::Twist2D prev,
  ateam_msgs::msg::Twist2D command, double vel_limit, double acc_limit, double dt)
{
  ateam_geometry::Vector prev_xy{prev.x, prev.y};
  ateam_geometry::Vector command_xy{command.x, command.y};
  ateam_geometry::Vector requested_acceleration = command_xy - prev_xy;

  double acceleration_magnitude = std::clamp(ateam_geometry::norm(requested_acceleration), 0.0,
      dt * acc_limit);
  ateam_geometry::Vector acc_limited_command = prev_xy +
    (acceleration_magnitude * ateam_geometry::normalize(requested_acceleration));

  double final_magnitude = std::clamp(ateam_geometry::norm(acc_limited_command), 0.0, vel_limit);
  ateam_geometry::Vector final_xy = final_magnitude *
    ateam_geometry::normalize(acc_limited_command);

  command.x = final_xy.x();
  command.y = final_xy.y();

  return command;
}

double ManeuverExecutor::apply_1d_motion_limits(
  double prev, double command, double vel_limit,
  double acc_limit, double dt)
{
  double requested_acceleration = command - prev;
  double acc_limited_command = prev + std::clamp(requested_acceleration, -dt * acc_limit,
      dt * acc_limit);
  return std::clamp(acc_limited_command, -vel_limit, vel_limit);
}

double ManeuverExecutor::get_dt()
{
  // const std::chrono::duration<double> elapsed =
  //  std::chrono::steady_clock::now() - prev_update_time_;
  // return elapsed.count();
  return 0.01;
}

double ManeuverExecutor::get_yaw(geometry_msgs::msg::Pose pose)
{
  tf2::Quaternion quat;
  tf2::fromMsg(pose.orientation, quat);
  double yaw, pitch, roll;
  tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);

  return yaw;
}

ateam_msgs::msg::Twist2D ManeuverExecutor::rotate_frame(
  ateam_msgs::msg::Twist2D input_frame,
  double angle)
{
  ateam_msgs::msg::Twist2D output_frame = ateam_msgs::msg::Twist2D();
  output_frame.x = cos(-angle) * input_frame.x - sin(-angle) * input_frame.y;
  output_frame.y = sin(-angle) * input_frame.x + cos(-angle) * input_frame.y;
  output_frame.theta = input_frame.theta;
  return output_frame;
}

TrajectoryParams_t ManeuverExecutor::generate_trajectory_params(
  const ateam_msgs::msg::RobotMotionCommand & ros_msg)
{
  TrajectoryParams_t traj_params = ateam_controls_default_traj_params();

  if (ros_msg.limit_vel_linear != 0.0) {
    traj_params.max_vel_linear = ros_msg.limit_vel_linear;
  }
  if (ros_msg.limit_vel_angular != 0.0) {
    traj_params.max_vel_angular = ros_msg.limit_vel_angular;
  }
  if (ros_msg.limit_acc_linear != 0.0) {
    traj_params.max_accel_linear = ros_msg.limit_acc_linear;
  }
  if (ros_msg.limit_acc_angular != 0.0) {
    traj_params.max_accel_angular = ros_msg.limit_acc_angular;
  }

  return traj_params;
}

PivotParams_t ManeuverExecutor::generate_pivot_params(
  const ateam_msgs::msg::RobotMotionCommand & ros_msg)
{
  PivotParams_t pivot_params = ateam_controls_default_pivot_params();
  if (ros_msg.limit_vel_angular != 0.0) {
    pivot_params.max_vel_angular = ros_msg.limit_vel_angular;
  }
  if (ros_msg.limit_acc_angular != 0.0) {
    pivot_params.max_accel_angular = ros_msg.limit_acc_angular;
  }

  pivot_params.orbit_radius = ros_msg.pivot_orbit_radius;
  pivot_params.inset_angle = ros_msg.pivot_inset_angle;
  pivot_params.compute_inset_angle = ros_msg.pivot_compute_inset_angle;
  pivot_params.direction = static_cast<PivotDirection_t>(ros_msg.pivot_direction);

  return pivot_params;
}

bool ManeuverExecutor::command_uses_trajectory()
{
  switch(command_.body_control_mode) {
    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION:
    case ateam_msgs::msg::RobotMotionCommand::BCM_HEADING_PIVOT:
    case ateam_msgs::msg::RobotMotionCommand::BCM_POINT_PIVOT:
      return true;

    default:
      return false;
  }
}

Vector6C_t ManeuverExecutor::get_trajectory_state()
{
  switch(command_.body_control_mode) {
    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION:
      return position_trajectory_.state;
    case ateam_msgs::msg::RobotMotionCommand::BCM_HEADING_PIVOT:
    case ateam_msgs::msg::RobotMotionCommand::BCM_POINT_PIVOT:
      return pivot_trajectory_.state;
    default:
      return Vector6C_t();
  }
}

void ManeuverExecutor::plan_trajectory(Vector6C_t starting_state)
{
  switch (command_.body_control_mode) {
    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION:
      {
        const Vector3C_t target_pose{
          static_cast<float>(command_.pose.x),
          static_cast<float>(command_.pose.y),
          static_cast<float>(command_.pose.theta)
        };
        traj_params_ = generate_trajectory_params(command_);
        ateam_controls_traj_from_target_pose(
          starting_state,
          target_pose,
          traj_params_,
          &position_trajectory_
        );
        break;
      }
    case ateam_msgs::msg::RobotMotionCommand::BCM_HEADING_PIVOT:
      {
        pivot_params_ = generate_pivot_params(command_);
        ateam_controls_pivot_traj_from_target_heading(
          starting_state,
          command_.pivot_global_theta,
          pivot_params_,
          &pivot_trajectory_
        );
        break;
      }
    case ateam_msgs::msg::RobotMotionCommand::BCM_POINT_PIVOT:
      {
        pivot_params_ = generate_pivot_params(command_);
        ateam_controls_pivot_traj_from_target_point(
          starting_state,
          command_.pivot_target_x,
          command_.pivot_target_y,
          pivot_params_,
          &pivot_trajectory_
        );
        break;
      }
  }
}

void ManeuverExecutor::tick_trajectory(float dt)
{
  switch (command_.body_control_mode) {
    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION:
      ateam_controls_traj_tick(&position_trajectory_, dt);
      break;
    case ateam_msgs::msg::RobotMotionCommand::BCM_HEADING_PIVOT:
    case ateam_msgs::msg::RobotMotionCommand::BCM_POINT_PIVOT:
      ateam_controls_pivot_traj_tick(&pivot_trajectory_, dt);
      break;
  }
}
}  // namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers
