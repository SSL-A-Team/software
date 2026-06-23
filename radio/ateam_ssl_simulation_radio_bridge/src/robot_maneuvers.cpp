#include <cmath>
#include "robot_maneuvers.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers
{

  void ManeuverExecutor::execute_maneuver(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    switch(ros_msg.body_control_mode) {
      case ateam_msgs::msg::RobotMotionCommand::BCM_OFF:
        bcm_off_maneuver();
        return; // Ignores acceleration limits set in message and immediately stops the robot

      // Trajectory Maneuvers
      case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION:
        global_position_maneuver(ros_msg, robot);
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

  void ManeuverExecutor::bcm_off_maneuver() {
    current_global_command_ = ateam_msgs::msg::Twist2D();
    prev_global_command_ = ateam_msgs::msg::Twist2D();
    prev_update_time_ = std::chrono::steady_clock::now();
  }

  // Trajectory Maneuvers
  void ManeuverExecutor::global_position_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {

    // Update trajectory state to current time
    float dt = float(get_dt());
    ateam_controls_tick_traj(&trajectory_, dt);

    double yaw = get_yaw(robot.pose);

    // Check current position against predicted position
    double current_dist = std::hypot(robot.pose.position.x - trajectory_.state.data[0], robot.pose.position.y - trajectory_.state.data[1]);
    bool current_pos_needs_replan = current_dist > 0.5 || angles::shortest_angular_distance(yaw, trajectory_.state.data[2]) > 0.3;

    bool target_changed = std::hypot(command_.pose.x - ros_msg.pose.x, command_.pose.y - ros_msg.pose.y) > 0.02 || angles::shortest_angular_distance(command_.pose.theta, ros_msg.pose.theta) > 0.1;

    // bool maneuver_command_changed = (command_ != ros_msg);
    bool maneuver_command_changed = (command_.body_control_mode != ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION || target_changed);

    bool replanning = maneuver_command_changed || current_pos_needs_replan;

    if (replanning) {
      std::cerr << "replanning id " << robot.id << ": (" << maneuver_command_changed << ", " << current_pos_needs_replan << ")" << std::endl;
      Vector6C_t starting_state = Vector6C_t{
        float(robot.pose.position.x),
        float(robot.pose.position.y),
        float(yaw),
        // float(robot.velocity.linear.x),
        // float(robot.velocity.linear.y),
        // float(robot.velocity.angular.z)
        float(prev_global_command_.x),
        float(prev_global_command_.y),
        float(prev_global_command_.theta),
      };

      const Vector3C_t target_pose{
        float(ros_msg.pose.x),
        float(ros_msg.pose.y),
        float(ros_msg.pose.theta)
      };

      command_ = ros_msg;
      traj_params_ = generate_trajectory_params(ros_msg);
      ateam_controls_traj_from_target_pose(
        starting_state,
        target_pose,
        traj_params_,
        &trajectory_
      );
    }

    ateam_msgs::msg::Twist2D global_target_err = ateam_msgs::msg::Twist2D();
    global_target_err.x = command_.pose.x - robot.pose.position.x;
    global_target_err.y = command_.pose.y - robot.pose.position.y;

    const double kpPos = 1.0;
    const double kdPos = 0.0;
    const double kpAngle = 0.0;

    ateam_msgs::msg::Twist2D global_feedback = ateam_msgs::msg::Twist2D();

    // Provide some minor xy feedback to account for slight final position offsets
    if (abs(global_target_err.x) < 0.05) {
      global_feedback.x += std::clamp(kpPos * global_target_err.x, -0.05, 0.05);
    }
    if (abs(global_target_err.y) < 0.05) {
      global_feedback.y += std::clamp(kpPos * global_target_err.y, -0.05, 0.05);
    }

    current_global_command_.x = trajectory_.state.data[3] + global_feedback.x;
    current_global_command_.y = trajectory_.state.data[4] + global_feedback.y;
    current_global_command_.theta = trajectory_.state.data[5] + global_feedback.theta;
  }

  // Global Maneuvers
  void ManeuverExecutor::global_velocity_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg) {
    current_global_command_ = ros_msg.velocity;
    std::cerr << "global command, global vel: " << current_global_command_.x << ", " << current_global_command_.y << ", " << current_global_command_.theta << std::endl;
  }

  void ManeuverExecutor::global_acceleration_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg) {
    const double dt = get_dt();

    current_global_command_.x = prev_global_command_.x + dt * ros_msg.acceleration.x;
    current_global_command_.y = prev_global_command_.y + dt * ros_msg.acceleration.y;
    current_global_command_.theta = prev_global_command_.theta + dt * ros_msg.acceleration.theta;
  }

  // Local Maneuvers
  void ManeuverExecutor::local_velocity_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    current_global_command_ = rotate_frame(ros_msg.velocity, -get_yaw(robot.pose));
    std::cerr << "local command, global vel: " << current_global_command_.x << ", " << current_global_command_.y << ", " << current_global_command_.theta << std::endl;
  }

  void ManeuverExecutor::local_acceleration_maneuver(const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    ateam_msgs::msg::Twist2D global_accel = rotate_frame(ros_msg.acceleration, -get_yaw(robot.pose));
    const double dt = get_dt();

    current_global_command_.x = prev_global_command_.x + dt * global_accel.x;
    current_global_command_.y = prev_global_command_.y + dt * global_accel.y;
    current_global_command_.theta = prev_global_command_.theta + dt * global_accel.theta;
  }

  void ManeuverExecutor::finalize_command(RobotMoveCommand * robot_move_command, const ateam_msgs::msg::RobotMotionCommand & ros_msg, ateam_msgs::msg::GameStateRobot robot) {
    command_ = ros_msg;
    double dt = get_dt();

    traj_params_ = generate_trajectory_params(command_);
    current_global_command_ = apply_xy_motion_limits(prev_global_command_, current_global_command_, traj_params_.max_vel_linear, traj_params_.max_accel_linear, dt);
    current_global_command_.theta = apply_1d_motion_limits(prev_global_command_.theta, current_global_command_.theta, traj_params_.max_vel_angular, traj_params_.max_accel_angular, dt);

    ateam_msgs::msg::Twist2D local_command;
    if (use_trajectory_angle()) {
      local_command = rotate_frame(current_global_command_, trajectory_.state.data[2]);
    } else {
      local_command = rotate_frame(current_global_command_, get_yaw(robot.pose));
    }

    MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();
    local_velocity_command->set_forward(local_command.x);
    local_velocity_command->set_left(local_command.y);
    local_velocity_command->set_angular(local_command.theta);

    prev_global_command_ = current_global_command_;
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
    // const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - prev_update_time_;
    // return elapsed.count();
    return 0.01;
  }

  double ManeuverExecutor::get_yaw(geometry_msgs::msg::Pose pose) {
    tf2::Quaternion quat;
    tf2::fromMsg(pose.orientation, quat);
    double yaw, pitch, roll;
    tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);

    return yaw;
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

    // std::cerr << "traj limits: " << traj_params.max_vel_linear << ", " << traj_params.max_vel_angular << ", " << traj_params.max_accel_linear << ", " << traj_params.max_accel_angular << std::endl;
    return traj_params;
  }

  bool ManeuverExecutor::use_trajectory_angle() {
    switch(command_.body_control_mode) {
      case ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION:
      case ateam_msgs::msg::RobotMotionCommand::BCM_PIVOT:
        return true;

      default:
        return false;
    }
  }
}  // namespace ateam_ssl_simulation_radio_bridge::robot_maneuvers