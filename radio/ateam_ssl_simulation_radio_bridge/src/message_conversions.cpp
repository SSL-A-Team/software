// Copyright 2021 A Team
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

#include "message_conversions.hpp"
#include <cmath>
#include <stdexcept>

#include <rclcpp/logging.hpp>

namespace ateam_ssl_simulation_radio_bridge::message_conversions
{

ateam_radio_msgs::msg::BasicTelemetry fromProto(const RobotFeedback & proto_msg)
{
  ateam_radio_msgs::msg::BasicTelemetry robot_feedback;

  if(proto_msg.has_dribbler_ball_contact()) {
    robot_feedback.breakbeam_ball_detected = proto_msg.dribbler_ball_contact();
  }
  robot_feedback.kicker_available = true;
  robot_feedback.chipper_available = true;
  robot_feedback.battery_percent = 100;

  return robot_feedback;
}

double ReplaceNanWithZero(const double val, rclcpp::Logger logger)
{
  if (std::isnan(val)) {
    RCLCPP_WARN(logger, "Radio bridge is replacing NaNs!");
    return 0.0;
  }
  return val;
}

RobotControl fromMsg(
  const ateam_msgs::msg::RobotMotionCommand & ros_msg, int robot_id,
  rclcpp::Logger logger)
{
  RobotControl robots_control;
  RobotCommand * proto_robot_command = robots_control.add_robot_commands();

  proto_robot_command->set_id(robot_id);
  proto_robot_command->set_dribbler_speed(ReplaceNanWithZero(9.5492968 * ros_msg.dribbler_setpoint,
      logger));

  switch (ros_msg.kick_request) {
    case ateam_msgs::msg::RobotMotionCommand::KR_ARM:
    case ateam_msgs::msg::RobotMotionCommand::KR_DISABLE:
      // Do nothing because the simulator doesn't "charge" kickers
      break;
    case ateam_msgs::msg::RobotMotionCommand::KR_KICK_NOW:
    case ateam_msgs::msg::RobotMotionCommand::KR_KICK_TOUCH:
    case ateam_msgs::msg::RobotMotionCommand::KR_KICK_CAPTURED:
    case ateam_msgs::msg::RobotMotionCommand::KR_CHIP_NOW:
    case ateam_msgs::msg::RobotMotionCommand::KR_CHIP_TOUCH:
    case ateam_msgs::msg::RobotMotionCommand::KR_CHIP_CAPTURED:
      proto_robot_command->set_kick_speed(ReplaceNanWithZero(ros_msg.kick_speed * 3.0, logger));
      break;
  }

  RobotMoveCommand * robot_move_command = proto_robot_command->mutable_move_command();
  MoveLocalVelocity * local_velocity_command = robot_move_command->mutable_local_velocity();

  local_velocity_command->set_forward(ReplaceNanWithZero(ros_msg.velocity.x, logger));
  local_velocity_command->set_left(ReplaceNanWithZero(ros_msg.velocity.y, logger));
  local_velocity_command->set_angular(ReplaceNanWithZero(ros_msg.velocity.theta, logger));

  return robots_control;
}


SimulatorControl fromMsg(const ssl_league_msgs::msg::SimulatorControl & ros_msg)
{
  SimulatorControl sim_control;

  if (ros_msg.teleport_ball.size() != 0) {
    auto ball_msg = ros_msg.teleport_ball[0];
    TeleportBall * proto_teleport_ball = sim_control.mutable_teleport_ball();

    proto_teleport_ball->set_x(ball_msg.pos.x);
    proto_teleport_ball->set_y(ball_msg.pos.y);
    proto_teleport_ball->set_z(ball_msg.pos.z);

    proto_teleport_ball->set_vx(ball_msg.velocity.x);
    proto_teleport_ball->set_vy(ball_msg.velocity.y);
    proto_teleport_ball->set_vz(ball_msg.velocity.z);

    if (!ball_msg.teleport_safely.empty()) {
      proto_teleport_ball->set_teleport_safely(ball_msg.teleport_safely.front());
    }
    if (!ball_msg.roll.empty()) {
      proto_teleport_ball->set_roll(ball_msg.roll.front());
    }
    if (!ball_msg.by_force.empty()) {
      proto_teleport_ball->set_by_force(ball_msg.by_force.front());
    }
  }

  if (ros_msg.teleport_robot.size() != 0) {
    for (auto robot_msg : ros_msg.teleport_robot) {
      TeleportRobot * proto_teleport_robot = sim_control.add_teleport_robot();
      RobotId * proto_robot_id = proto_teleport_robot->mutable_id();

      if (robot_msg.id.id.size() != 0) {
        proto_robot_id->set_id(robot_msg.id.id[0]);
      } else {
        throw std::invalid_argument("No robot number specified");
      }

      if (robot_msg.id.team.size() != 0) {
        proto_robot_id->set_team(static_cast<Team>(robot_msg.id.team[0]));
      } else {
        throw std::invalid_argument("No robot team specified");
      }

      proto_teleport_robot->set_x(robot_msg.pos.x);
      proto_teleport_robot->set_y(robot_msg.pos.y);

      // orientation is yaw in radians directly (no quaternion round trip needed)
      if (!robot_msg.orientation.empty()) {
        proto_teleport_robot->set_orientation(robot_msg.orientation.front());
      }

      proto_teleport_robot->set_v_x(robot_msg.velocity.x);
      proto_teleport_robot->set_v_y(robot_msg.velocity.y);

      if (!robot_msg.v_angular.empty()) {
        proto_teleport_robot->set_v_angular(robot_msg.v_angular.front());
      }

      if (!robot_msg.present.empty()) {
        proto_teleport_robot->set_present(robot_msg.present.front());
      }

      if (!robot_msg.by_force.empty()) {
        proto_teleport_robot->set_by_force(robot_msg.by_force.front());
      }
    }
  }

  if (!ros_msg.simulation_speed.empty()) {
    sim_control.set_simulation_speed(ros_msg.simulation_speed.front());
  }

  return sim_control;
}

}  // namespace ateam_ssl_simulation_radio_bridge::message_conversions
