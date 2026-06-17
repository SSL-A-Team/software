#include <cmath>
#include "robot_maneuvers.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers
{

  void global_position_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot, ManeuverInfo & maneuver_info) {
    const Vector3C_t target_pose{
      float(ros_msg.pose.x),
      float(ros_msg.pose.y),
      float(ros_msg.pose.theta)
    };
  
    tf2::Quaternion quat;
    tf2::fromMsg(robot.pose.orientation, quat);
    double yaw, pitch, roll;
    tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);

    Vector6C_t seed_state = maneuver_info.trajectory_state;

    // Check target point
    double target_dist = std::pow(maneuver_info.target_pose.x - ros_msg.pose.x, 2) + std::pow(maneuver_info.target_pose.y - ros_msg.pose.y, 2);
    // TODO (chachmu): fix angle wraparound issues and improve this check
    bool target_pos_needs_replan = target_dist > 0.01 || abs(maneuver_info.target_pose.theta - ros_msg.pose.theta) > 0.01;

    // Check current position
    double current_dist = std::pow(robot.pose.position.x - maneuver_info.trajectory_state.data[0], 2) + std::pow(robot.pose.position.y - maneuver_info.trajectory_state.data[1], 2);
    // TODO (chachmu): fix angle wraparound issues and improve this check
    bool current_pos_needs_replan = current_dist > 0.5 || abs(yaw - maneuver_info.trajectory_state.data[2]) > 0.2;
    // current_pos_needs_replan = false; // REMOVE THIS

    bool maneuver_type_mismatch = maneuver_info.maneuver_type != ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION;

    bool replanning = maneuver_type_mismatch || target_pos_needs_replan || current_pos_needs_replan;
    if (replanning) {
      // std::cerr << "REPLANNING: " << maneuver_type_mismatch << ", " << target_pos_needs_replan << ", " <<  current_pos_needs_replan << std::endl;
      seed_state = Vector6C_t{
        float(robot.pose.position.x),
        float(robot.pose.position.y),
        float(yaw),
        float(robot.velocity.linear.x),
        float(robot.velocity.linear.y),
        float(robot.velocity.angular.z)
      };
    }

    TrajectoryParams_t traj_params = generate_trajectory_params(ros_msg);

    BangBangTraj3D_t traj;
    ateam_controls_traj_from_target_pose(
      seed_state,
      target_pose,
      traj_params,
      &traj
    );

    // TODO (chachmu): figure out how to safely calculate dt
    std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - maneuver_info.prev_update_time;
    // float dt = elapsed.count();
    float dt = 0.01;

    // const double global_x_err = ros_msg.pose.x - robot.pose.position.x;
    // const double global_y_err = ros_msg.pose.y - robot.pose.position.y;
    const double global_x_err = seed_state.data[0] - robot.pose.position.x;
    const double global_y_err = seed_state.data[1] - robot.pose.position.y;
    const double global_theta_err = seed_state.data[2] - yaw;

    const double total_err = std::pow(global_x_err, 2) + std::pow(global_y_err, 2);
    // std::cerr << "total_err: " << total_err << ", x: " << global_x_err << ", y: " << global_y_err << ", theta: " << global_theta_err << std::endl;

    const double kPos = 1.2;
    const double kAngle = 1.0;

    const double global_x_vel = seed_state.data[3] + (kPos * global_x_err);
    const double global_y_vel = seed_state.data[4] + (kPos * global_y_err);
    const double theta_vel = seed_state.data[5] + (kAngle * global_theta_err);

    const double local_x_vel = cos(-yaw) * global_x_vel - sin(-yaw) * global_y_vel;
    const double local_y_vel = sin(-yaw) * global_x_vel + cos(-yaw) * global_y_vel;

    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

    local_velocity_command->set_forward(local_x_vel);
    local_velocity_command->set_left(local_y_vel);
    local_velocity_command->set_angular(theta_vel);

    // Update the trajectory state for the next iteration
    ateam_controls_traj_state_at(
      traj,
      seed_state,
      0.0f,
      dt,
      &maneuver_info.trajectory_state
    );

    // Only update the maneuver_info once we are sure we haven't errored out of using the maneuver
    if (replanning) {
      maneuver_info.target_pose = ros_msg.pose;
    }
    maneuver_info.maneuver_type = ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION;
    maneuver_info.trajectory = traj;
    maneuver_info.prev_update_time = std::chrono::steady_clock::now();
  }

  void global_velocity_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

    tf2::Quaternion quat;
    tf2::fromMsg(robot.pose.orientation, quat);
    double yaw, pitch, roll;
    tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);

    const double local_x_vel = cos(-yaw) * ros_msg.velocity.x - sin(-yaw) * ros_msg.velocity.y;
    const double local_y_vel = sin(-yaw) * ros_msg.velocity.x + cos(-yaw) * ros_msg.velocity.y;

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
    // TODO (chachmu): convert the coordinate frames
    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();
    const double dt = 1.0 / 100.0;

    local_velocity_command->set_forward(robot.velocity.linear.x + dt * ros_msg.acceleration.x);
    local_velocity_command->set_left(robot.velocity.linear.y + dt * ros_msg.acceleration.y);
    local_velocity_command->set_angular(robot.velocity.angular.z + dt * ros_msg.acceleration.theta);
  }

  void local_acceleration_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();
    const double dt = 1.0 / 100.0;

    local_velocity_command->set_forward(robot.velocity.linear.x + dt * ros_msg.acceleration.x);
    local_velocity_command->set_left(robot.velocity.linear.y +  dt * ros_msg.acceleration.y);
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