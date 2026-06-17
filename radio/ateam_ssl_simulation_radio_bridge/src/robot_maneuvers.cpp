#include "robot_maneuvers.hpp"

namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers
{

    void global_position_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    // TODO (chachmu): finish implementing this once we have rust functions/types
    //   const Vector3f target_pose = Vector3f{
    //     ros_msg.pose.x,
    //     ros_msg.pose.y,
    //     ros_msg.pose.theta
    //   }
      
    //   const double robot_angle = from_quaternion(
    //     robot.pose.orientation.x,
    //     robot.pose.orientation.y,
    //     robot.pose.orientation.z,
    //     robot.pose.orientation.w,
    //   )

    //   const Vector6f cur_state = Vector6f{
    //     robot.pose.position.x, 
    //     robot.pose.position.y, 
    //     robot_angle,
    //     robot.twist.linear.x,
    //     robot.twist.linear.y,
    //     robot.twist.angular.z
    //   }
      
    //   TrajectoryParams traj_params = generate_trajectory_params(ros_msg);
    //   BangBangTraj3D traj = BangBangTraj3D::from_target_pose(cur_state, target_pose, traj_params);

    //   Vector6f traj_command = traj.state_at(cur_state, 0.0, 0.0);

    //   MoveGlobalVelocity * global_velocity_command = robot_move_command->mutable_global_velocity();

    //   global_velocity_command->set_x(traj_command[3]);
    //   global_velocity_command->set_y(traj_command[4]);
    //   global_velocity_command->set_angular(traj_command[5]);
    }

    void global_velocity_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg) {
      MoveGlobalVelocity * global_velocity_command = robot_move_command->mutable_global_velocity();

      global_velocity_command->set_x(ros_msg.velocity.x);
      global_velocity_command->set_y(ros_msg.velocity.y);
      global_velocity_command->set_angular(ros_msg.velocity.theta);
    }

    void local_velocity_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg) {
      MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

      local_velocity_command->set_forward(ros_msg.velocity.x);
      local_velocity_command->set_left(ros_msg.velocity.y);
      local_velocity_command->set_angular(ros_msg.velocity.theta);
    }

    void global_acceleration_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
      MoveGlobalVelocity * global_velocity_command = robot_move_command->mutable_global_velocity();

      global_velocity_command->set_x(robot.velocity.linear.x + 0.01 * ros_msg.acceleration.x);
      global_velocity_command->set_y(robot.velocity.linear.y + 0.01 * ros_msg.acceleration.y);
      global_velocity_command->set_angular(robot.velocity.angular.z + 0.01 * ros_msg.acceleration.theta);
    }

    void local_acceleration_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
      // TODO (chachmu): handle converting to local robot velocity
      MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

      local_velocity_command->set_forward(robot.velocity.linear.x + 0.01 * ros_msg.acceleration.x);
      local_velocity_command->set_left(robot.velocity.linear.y + 0.01 * ros_msg.acceleration.y);
      local_velocity_command->set_angular(robot.velocity.angular.z + 0.01 * ros_msg.acceleration.theta);
    }

    // TrajectoryParams generate_trajectory_params(const ateam_msgs::msg::RobotMotionCommand & ros_msg) {
    // TODO (chachmu): finish implementing this once we have rust functions/types
    //      let traj_params = TrajectoryParams {
    //         max_vel_linear: if cmd.max_linear_vel != 0.0 {
    //             cmd.max_linear_vel
    //         } else {
    //             default_params.max_vel_linear
    //         },
    //         max_vel_angular: if cmd.max_angular_vel != 0.0 {
    //             cmd.max_angular_vel
    //         } else {
    //             default_params.max_vel_angular
    //         },
    //         max_accel_linear: if cmd.max_linear_acc != 0.0 {
    //             cmd.max_linear_acc
    //         } else {
    //             default_params.max_accel_linear
    //         },
    //         max_accel_angular: if cmd.max_angular_acc != 0.0 {
    //             cmd.max_angular_acc
    //         } else {
    //             default_params.max_accel_angular
    //         },
    //     };
    // }
}  // namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers