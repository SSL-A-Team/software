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
  command_ = ros_msg;
  const float dt = static_cast<float>(get_dt());

  // Initialize the reference from vision once. Thereafter the reference evolves
  // purely from the previous trajectory state (open-loop): the simulator runs the
  // tracking control law against ground truth, so the bridge needs no feedback.
  if (!initialized_) {
    ref_state_ = seed_from_vision(robot);
    ref_accel_ = Vector3C_t{};
    initialized_ = true;
  }

  switch (ros_msg.body_control_mode) {
    case ateam_msgs::msg::RobotMotionCommand::BCM_OFF:
    case ateam_msgs::msg::RobotMotionCommand::BCM_ESTOP_BRAKE:
      hold_maneuver(robot);
      break;

    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_ACCEL:
      integrate_accel(ros_msg.acceleration, dt);
      break;
    case ateam_msgs::msg::RobotMotionCommand::BCM_LOCAL_ACCEL:
      integrate_accel(rotate_frame(ros_msg.acceleration, -ref_state_.data[2]), dt);
      break;

    // Trajectory maneuvers: generate the controls-repo trajectory, tick, sample.
    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION:
    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_VELOCITY:
    case ateam_msgs::msg::RobotMotionCommand::BCM_LOCAL_VELOCITY:
    case ateam_msgs::msg::RobotMotionCommand::BCM_HEADING_PIVOT:
    case ateam_msgs::msg::RobotMotionCommand::BCM_POINT_PIVOT:
      plan_maneuver(ros_msg);
      tick_and_sample(dt);
      break;
  }

  send_reference(robot_move_command);
  prev_update_time_ = std::chrono::steady_clock::now();
}

void ManeuverExecutor::hold_maneuver(ateam_msgs::msg::GameStateRobot robot)
{
  // Hold the current pose with zero velocity / acceleration.
  ref_state_ = seed_from_vision(robot);
  ref_state_.data[3] = 0.0f;
  ref_state_.data[4] = 0.0f;
  ref_state_.data[5] = 0.0f;
  ref_accel_ = Vector3C_t{};
}

void ManeuverExecutor::plan_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg)
{
  // Always seed from the previous trajectory state for a continuous, open-loop
  // handoff between consecutive plans.
  const Vector6C_t seed = ref_state_;

  switch (ros_msg.body_control_mode) {
    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION:
      {
        traj_params_ = generate_trajectory_params(ros_msg);
        const Vector3C_t target_pose{
          static_cast<float>(ros_msg.pose.x),
          static_cast<float>(ros_msg.pose.y),
          static_cast<float>(ros_msg.pose.theta)
        };
        ateam_controls_traj_from_target_pose(
          seed, target_pose, traj_params_, &position_trajectory_);
        break;
      }
    case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_VELOCITY:
    case ateam_msgs::msg::RobotMotionCommand::BCM_LOCAL_VELOCITY:
      {
        traj_params_ = generate_trajectory_params(ros_msg);
        ateam_msgs::msg::Twist2D global_velocity = ros_msg.velocity;
        if (ros_msg.body_control_mode ==
          ateam_msgs::msg::RobotMotionCommand::BCM_LOCAL_VELOCITY)
        {
          // Rotate local -> global using the trajectory's current heading.
          global_velocity = rotate_frame(ros_msg.velocity, -ref_state_.data[2]);
        }
        const Vector3C_t target_twist{
          static_cast<float>(global_velocity.x),
          static_cast<float>(global_velocity.y),
          static_cast<float>(global_velocity.theta)
        };
        ateam_controls_traj_from_target_twist(
          seed, target_twist, traj_params_, &position_trajectory_);
        break;
      }
    case ateam_msgs::msg::RobotMotionCommand::BCM_HEADING_PIVOT:
      {
        pivot_params_ = generate_pivot_params(ros_msg);
        ateam_controls_pivot_traj_from_target_heading(
          seed, ros_msg.pivot_global_theta, pivot_params_, &pivot_trajectory_);
        break;
      }
    case ateam_msgs::msg::RobotMotionCommand::BCM_POINT_PIVOT:
      {
        pivot_params_ = generate_pivot_params(ros_msg);
        ateam_controls_pivot_traj_from_target_point(
          seed, ros_msg.pivot_target_x, ros_msg.pivot_target_y, pivot_params_,
          &pivot_trajectory_);
        break;
      }
  }
}

void ManeuverExecutor::tick_and_sample(float dt)
{
  if (command_uses_pivot()) {
    ateam_controls_pivot_traj_tick(&pivot_trajectory_, dt);
    ateam_controls_pivot_traj_sample(pivot_trajectory_, &ref_state_, &ref_accel_);
  } else {
    ateam_controls_traj_tick(&position_trajectory_, dt);
    ateam_controls_traj_sample(position_trajectory_, &ref_state_, &ref_accel_);
  }
}

void ManeuverExecutor::integrate_accel(
  const ateam_msgs::msg::Twist2D & global_acceleration, float dt)
{
  ref_accel_.x = static_cast<float>(global_acceleration.x);
  ref_accel_.y = static_cast<float>(global_acceleration.y);
  ref_accel_.z = static_cast<float>(global_acceleration.theta);

  const float vx = ref_state_.data[3];
  const float vy = ref_state_.data[4];
  const float vtheta = ref_state_.data[5];

  ref_state_.data[0] += vx * dt;
  ref_state_.data[1] += vy * dt;
  ref_state_.data[2] =
    static_cast<float>(angles::normalize_angle(ref_state_.data[2] + vtheta * dt));
  ref_state_.data[3] = vx + ref_accel_.x * dt;
  ref_state_.data[4] = vy + ref_accel_.y * dt;
  ref_state_.data[5] = vtheta + ref_accel_.z * dt;
}

void ManeuverExecutor::send_reference(RobotMoveCommand * robot_move_command)
{
  MoveGlobalTrajectoryReference * reference =
    robot_move_command->mutable_global_trajectory_reference();
  reference->set_x(ref_state_.data[0]);
  reference->set_y(ref_state_.data[1]);
  reference->set_theta(ref_state_.data[2]);
  reference->set_vx(ref_state_.data[3]);
  reference->set_vy(ref_state_.data[4]);
  reference->set_vangular(ref_state_.data[5]);
  reference->set_ax(ref_accel_.x);
  reference->set_ay(ref_accel_.y);
  reference->set_aangular(ref_accel_.z);
}

Vector6C_t ManeuverExecutor::seed_from_vision(ateam_msgs::msg::GameStateRobot robot)
{
  return Vector6C_t{
    static_cast<float>(robot.pose.position.x),
    static_cast<float>(robot.pose.position.y),
    static_cast<float>(get_yaw(robot.pose)),
    static_cast<float>(robot.velocity.linear.x),
    static_cast<float>(robot.velocity.linear.y),
    static_cast<float>(robot.velocity.angular.z),
  };
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

bool ManeuverExecutor::command_uses_pivot()
{
  switch (command_.body_control_mode) {
    case ateam_msgs::msg::RobotMotionCommand::BCM_HEADING_PIVOT:
    case ateam_msgs::msg::RobotMotionCommand::BCM_POINT_PIVOT:
      return true;

    default:
      return false;
  }
}
}  // namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers
