#include "robot_maneuvers.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers
{

  void global_position_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    const Vector3C_t target_pose{
      float(ros_msg.pose.x),
      float(ros_msg.pose.y),
      float(ros_msg.pose.theta)
    };
  
    tf2::Quaternion quat;
    tf2::fromMsg(robot.pose.orientation, quat);
    double yaw, pitch, roll;
    tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);


    const double local_x_pos = cos(-yaw) * robot.pose.position.x - sin(-yaw) * robot.pose.position.y;
    const double local_y_pos = sin(-yaw) * robot.pose.position.x + cos(-yaw) * robot.pose.position.y;
    const double local_x_vel = cos(-yaw) * robot.velocity.linear.x - sin(-yaw) * robot.velocity.linear.y;
    const double local_y_vel = sin(-yaw) * robot.velocity.linear.x + cos(-yaw) * robot.velocity.linear.y;

    const Vector6C_t curr_state{
      float(local_x_pos),
      float(local_y_pos),
      float(yaw),
      float(local_x_vel),
      float(local_y_vel),
      float(robot.velocity.angular.z)
    };
    
    TrajectoryParams_t traj_params = generate_trajectory_params(ros_msg);

    BangBangTraj3D_t traj;
    ateam_controls_traj_from_target_pose(
      curr_state,
      target_pose,
      traj_params,
      &traj
    );

    Vector6C_t traj_command;
    ateam_controls_traj_state_at(
      traj,
      curr_state,
      0.0f,
      0.05f,
      &traj_command
    );

    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

    local_velocity_command->set_forward(traj_command.data[3]);
    local_velocity_command->set_left(traj_command.data[4]);
    local_velocity_command->set_angular(traj_command.data[5]);
  }

  void global_velocity_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg) {
    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

    local_velocity_command->set_forward(ros_msg.velocity.x);
    local_velocity_command->set_left(ros_msg.velocity.y);
    local_velocity_command->set_angular(ros_msg.velocity.theta);
  }

  void local_velocity_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg) {
    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

    local_velocity_command->set_forward(ros_msg.velocity.x);
    local_velocity_command->set_left(ros_msg.velocity.y);
    local_velocity_command->set_angular(ros_msg.velocity.theta);
  }

  void global_acceleration_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();
    const double dt = 1.0 / 100.0;

    local_velocity_command->set_forward(robot.velocity.linear.x + dt * ros_msg.acceleration.x);
    local_velocity_command->set_left(robot.velocity.linear.y + dt * ros_msg.acceleration.y);
    local_velocity_command->set_angular(robot.velocity.angular.z + dt * ros_msg.acceleration.theta);
  }

  void local_acceleration_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {

    tf2::Quaternion quat;
    tf2::fromMsg(robot.pose.orientation, quat);
    double yaw, pitch, roll;
    tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);

    // Test this, I might have done it backwards
    const double local_x_vel = cos(-yaw) * robot.velocity.linear.x - sin(-yaw) * robot.velocity.linear.y;
    const double local_y_vel = sin(-yaw) * robot.velocity.linear.x + cos(-yaw) * robot.velocity.linear.y;
    const double dt = 1.0 / 100.0;

    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

    local_velocity_command->set_forward(local_x_vel + dt * ros_msg.acceleration.x);
    local_velocity_command->set_left(local_y_vel +  dt * ros_msg.acceleration.y);
    local_velocity_command->set_angular(robot.velocity.angular.z + dt * ros_msg.acceleration.theta);
  }

  TrajectoryParams_t generate_trajectory_params(const ateam_msgs::msg::RobotMotionCommand & ros_msg) {

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
}  // namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers