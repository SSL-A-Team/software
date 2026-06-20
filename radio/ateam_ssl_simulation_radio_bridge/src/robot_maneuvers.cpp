#include <cmath>
#include "robot_maneuvers.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers
{

  void ManeuverExecutor::execute_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    command_ = ros_msg;

    switch(ros_msg.body_control_mode) {
      case ateam_msgs::msg::RobotMotionCommand::BCM_OFF:
        bcm_off_maneuver(robot_move_command);
        prev_body_command_ = ateam_msgs::msg::Twist2D();
        prev_update_time_ = std::chrono::steady_clock::now();
        return; // Ignores acceleration limits set in message and immediately stops the robot
      case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION:
        global_position_maneuver(robot_move_command, ros_msg, robot);
        break;
      case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_VELOCITY:
        global_velocity_maneuver(robot_move_command, ros_msg, robot);
        break;
      case ateam_msgs::msg::RobotMotionCommand::BCM_LOCAL_VELOCITY:
        local_velocity_maneuver(robot_move_command, ros_msg);
        break;
      case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_ACCEL:
        global_acceleration_maneuver(robot_move_command, ros_msg, robot);
        break;
      case ateam_msgs::msg::RobotMotionCommand::BCM_LOCAL_ACCEL:
        local_acceleration_maneuver(robot_move_command, ros_msg, robot);
        break;
    }

    finalize_command(robot_move_command, robot);
  }

  void ManeuverExecutor::global_position_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    const Vector3C_t new_target_pose{
      float(ros_msg.pose.x),
      float(ros_msg.pose.y),
      float(ros_msg.pose.theta)
    };

    tf2::Quaternion quat;
    tf2::fromMsg(robot.pose.orientation, quat);
    double yaw, pitch, roll;
    tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);

    // Check if new target is too far from previous
    double target_change_dist = std::hypot(target_pose_.x - ros_msg.pose.x, target_pose_.y - ros_msg.pose.y);
    bool target_pos_needs_replan = target_change_dist > 0.01 || angles::shortest_angular_distance(target_pose_.theta, ros_msg.pose.theta) > 0.01;

    // Check current position against predicted position
    double current_dist = std::hypot(robot.pose.position.x - trajectory_state_.data[0], robot.pose.position.y - trajectory_state_.data[1]);
    bool current_pos_needs_replan = current_dist > 0.5 || angles::shortest_angular_distance(yaw, trajectory_state_.data[2]) > 0.2;

    bool maneuver_type_mismatch = command_.body_control_mode != ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION;

    bool replanning = maneuver_type_mismatch || target_pos_needs_replan || current_pos_needs_replan;

    // Use previously predicted state unless we need to replan
    Vector6C_t seed_state = trajectory_state_;
    if (replanning) {
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
      new_target_pose,
      traj_params,
      &traj
    );

    // TODO (chachmu): The controls process may need to be shuffled around to calculate the trajectory state at the beginning of the function
    // so that the actual dt for that timestep can be properly calculated
    // float dt = float(get_dt());
    float dt = 0.01;

    const double global_x_err = seed_state.data[0] - robot.pose.position.x;
    const double global_y_err = seed_state.data[1] - robot.pose.position.y;
    const double global_theta_err = seed_state.data[2] - yaw;

    // const double kPos = 1.2;
    // const double kAngle = 1.0;
    const double kPos = 0;
    const double kAngle = 0;

    ateam_msgs::msg::Twist2D global_vel = ateam_msgs::msg::Twist2D();
    global_vel.x = seed_state.data[3] + (kPos * global_x_err);
    global_vel.y = seed_state.data[4] + (kPos * global_y_err);
    global_vel.theta = seed_state.data[5] + (kAngle * global_theta_err);

    ateam_msgs::msg::Twist2D local_vel = rotate_frame(global_vel, robot.pose);

    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();
    local_velocity_command->set_forward(local_vel.x);
    local_velocity_command->set_left(local_vel.y);
    local_velocity_command->set_angular(local_vel.theta);

    // Update the trajectory state for the next iteration
    ateam_controls_traj_state_at(
      traj,
      seed_state,
      0.0f,
      dt,
      &trajectory_state_
    );
    trajectory_ = traj;

    // Only update the target once we are sure we haven't errored out of using the maneuver
    if (replanning) {
      target_pose_ = ros_msg.pose;
    }
  }

  void ManeuverExecutor::global_velocity_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

    const ateam_msgs::msg::Twist2D local_vel = rotate_frame(ros_msg.velocity, robot.pose);
    local_velocity_command->set_forward(local_vel.x);
    local_velocity_command->set_left(local_vel.y);
    local_velocity_command->set_angular(local_vel.theta);
  }

  void ManeuverExecutor::local_velocity_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg) {
    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

    local_velocity_command->set_forward(ros_msg.velocity.x);
    local_velocity_command->set_left(ros_msg.velocity.y);
    local_velocity_command->set_angular(ros_msg.velocity.theta);
  }

  void ManeuverExecutor::bcm_off_maneuver(RobotMoveCommand * robot_move_command) {
    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

    local_velocity_command->set_forward(0.0);
    local_velocity_command->set_left(0.0);
    local_velocity_command->set_angular(0.0);
  }

  void ManeuverExecutor::global_acceleration_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();
    const double dt = get_dt();

    ateam_msgs::msg::Twist2D global_accel = ateam_msgs::msg::Twist2D();
    global_accel.x = robot.velocity.linear.x + dt * ros_msg.acceleration.x;
    global_accel.y = robot.velocity.linear.y + dt * ros_msg.acceleration.y;
    global_accel.theta = robot.velocity.angular.z + dt * ros_msg.acceleration.theta;

    const ateam_msgs::msg::Twist2D local_accel = rotate_frame(global_accel, robot.pose);

    local_velocity_command->set_forward(local_accel.x);
    local_velocity_command->set_left(local_accel.y);
    local_velocity_command->set_angular(local_accel.theta);
  }

  void ManeuverExecutor::local_acceleration_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();
    const double dt = get_dt();

    local_velocity_command->set_forward(robot.velocity.linear.x + dt * ros_msg.acceleration.x);
    local_velocity_command->set_left(robot.velocity.linear.y +  dt * ros_msg.acceleration.y);
    local_velocity_command->set_angular(robot.velocity.angular.z + dt * ros_msg.acceleration.theta);
  }

  void ManeuverExecutor::finalize_command(RobotMoveCommand * robot_move_command, ateam_msgs::msg::GameStateRobot robot) {

    // TODO (chachmu): Expose default limits from the rust controls package
    double limit_vel_linear = 3.0;
    double limit_vel_angular = 5.0 * M_PI;
    double limit_acc_linear = 3.0;
    double limit_acc_angular = 10.0 * M_PI;

    if (command_.limit_vel_linear != 0.0) {
      limit_vel_linear = command_.limit_vel_linear;
    }
    if (command_.limit_vel_angular != 0.0) {
      limit_vel_angular = command_.limit_vel_angular;
    }
    if (command_.limit_acc_linear != 0.0) {
      limit_acc_linear = command_.limit_acc_linear;
    }
    if (command_.limit_acc_angular != 0.0) {
      limit_acc_angular = command_.limit_acc_angular;
    }

    double dt = get_dt();
    if (dt == 0.0 || std::isnan(dt) || dt > 0.1) {
      // This is a bit hacky but should keep dt from going too crazy
      dt = 0.01;
    }

    if (robot_move_command->has_local_velocity()) {
      MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

      ateam_msgs::msg::Twist2D local_command = ateam_msgs::msg::Twist2D();
      local_command.x = local_velocity_command->forward();
      local_command.y = local_velocity_command->left();
      local_command.theta = local_velocity_command->angular();

      local_command = apply_xy_motion_limits(prev_body_command_, local_command, limit_vel_linear, limit_acc_linear, dt);
      local_command.theta = apply_1d_motion_limits(prev_body_command_.theta, local_command.theta, limit_vel_angular, limit_acc_angular, dt);

      local_velocity_command->set_forward(local_command.x);
      local_velocity_command->set_left(local_command.y);
      local_velocity_command->set_angular(local_command.theta);

      prev_body_command_ = local_command;
    } else if (robot_move_command->has_global_velocity()) {
      MoveGlobalVelocity * global_velocity_command = robot_move_command->mutable_global_velocity();

      ateam_msgs::msg::Twist2D global_command = ateam_msgs::msg::Twist2D();
      global_command.x = global_velocity_command->x();
      global_command.y = global_velocity_command->y();
      global_command.theta = global_velocity_command->angular();

      global_command = apply_xy_motion_limits(prev_body_command_, global_command, limit_vel_linear, limit_acc_linear, dt);
      global_command.theta = apply_1d_motion_limits(prev_body_command_.theta, global_command.theta, limit_vel_angular, limit_acc_angular, dt);

      global_velocity_command->set_x(global_command.x);
      global_velocity_command->set_y(global_command.y);
      global_velocity_command->set_angular(global_command.theta);

      prev_body_command_ = rotate_frame(global_command, robot.pose);
    }

    prev_update_time_ = std::chrono::steady_clock::now();
  }


  ateam_msgs::msg::Twist2D ManeuverExecutor::apply_xy_motion_limits(ateam_msgs::msg::Twist2D prev, ateam_msgs::msg::Twist2D command, double vel_limit, double acc_limit, double dt) {
    ateam_geometry::Vector prev_xy{prev.x, prev.y};
    ateam_geometry::Vector command_xy{command.x, command.y};
    ateam_geometry::Vector requested_acceleration = command_xy - prev_xy;

    double acceleration_magnitude = std::clamp(ateam_geometry::norm(requested_acceleration), 0.0, dt * acc_limit);
    ateam_geometry::Vector acc_limited_command = prev_xy + (acceleration_magnitude * ateam_geometry::normalize(requested_acceleration));

    double final_magnitude = std::clamp(ateam_geometry::norm(acc_limited_command), 0.0, vel_limit);
    ateam_geometry::Vector final_xy = final_magnitude * ateam_geometry::normalize(acc_limited_command);

    command.x = final_xy.x();
    command.y = final_xy.y();

    return command;
  }

  double ManeuverExecutor::apply_1d_motion_limits(double prev, double command, double vel_limit, double acc_limit, double dt) {
    double requested_acceleration = command - prev;
    double acc_limited_command = prev + std::clamp(requested_acceleration, -dt * acc_limit, dt * acc_limit);
    return std::clamp(acc_limited_command, -vel_limit, vel_limit);
  }

  double ManeuverExecutor::get_dt() {
    const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - prev_update_time_;
    return elapsed.count();
  }

  ateam_msgs::msg::Twist2D ManeuverExecutor::rotate_frame(ateam_msgs::msg::Twist2D input_frame, geometry_msgs::msg::Pose pose) {
    tf2::Quaternion quat;
    tf2::fromMsg(pose.orientation, quat);
    double yaw, pitch, roll;
    tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);

    return rotate_frame(input_frame, yaw);
  }

  ateam_msgs::msg::Twist2D ManeuverExecutor::rotate_frame(ateam_msgs::msg::Twist2D input_frame, double angle) {
    ateam_msgs::msg::Twist2D output_frame = ateam_msgs::msg::Twist2D();
    output_frame.x = cos(-angle) * input_frame.x - sin(-angle) * input_frame.y;
    output_frame.y = sin(-angle) * input_frame.x + cos(-angle) * input_frame.y;
    output_frame.theta = input_frame.theta;
    return output_frame;
  }

  TrajectoryParams_t ManeuverExecutor::generate_trajectory_params(const ateam_msgs::msg::RobotMotionCommand & ros_msg) {

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