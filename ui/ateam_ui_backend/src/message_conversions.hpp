// Copyright 2025 A Team
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

#ifndef MESSAGE_CONVERSIONS_HPP_
#define MESSAGE_CONVERSIONS_HPP_

#include <optional>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

#include <ateam_msgs/msg/world.hpp>
#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_msgs/msg/robot_feedback.hpp>
#include <ateam_msgs/msg/behavior_executor_state.hpp>
#include <ateam_msgs/msg/trajectory.hpp>
#include <ateam_msgs/msg/sample3d.hpp>
#include <ateam_msgs/msg/overlay_array.hpp>
#include <ateam_msgs/msg/overlay.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <ateam_msgs/msg/field_sided_info.hpp>
#include <ssl_league_msgs/msg/referee.hpp>
#include <ssl_league_msgs/msg/team_info.hpp>
#include <ateam_msgs/msg/play_info.hpp>
#include <ateam_msgs/msg/playbook_state.hpp>
#include <ateam_msgs/msg/joystick_control_status.hpp>

#include <ateam_msgs/srv/set_desired_keeper.hpp>
#include <ateam_msgs/srv/set_play_enabled.hpp>
#include <ateam_msgs/srv/set_override_play.hpp>
#include <ateam_msgs/srv/set_ignore_field_side.hpp>

#include <ateam_msgs/srv/send_simulator_control_packet.hpp>
#include <ssl_league_msgs/msg/simulator_control.hpp>
#include <ssl_league_msgs/msg/teleport_ball_command.hpp>
#include <ssl_league_msgs/msg/teleport_robot_command.hpp>


#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/polygon.hpp>


/*
  Params:
    name: "/team_client_node:team_name"

    name: "/joystick_control_node:robot_id"
*/

namespace ateam_ui_backend_node::message_conversions
{

nlohmann::json fromMsg(
  const ateam_msgs::msg::World & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::RefereeInfo & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::BallState & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::RobotState & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::RobotFeedback & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::BehaviorExecutorState & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::Trajectory & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::Sample3d & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::OverlayArray & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::Overlay & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::FieldInfo & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::FieldSidedInfo & ros_msg);

nlohmann::json fromMsg(
  const ssl_league_msgs::msg::Referee & ros_msg);

nlohmann::json fromMsg(
  const ssl_league_msgs::msg::TeamInfo & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::PlayInfo & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::PlaybookState & ros_msg);

nlohmann::json fromMsg(
  const ateam_msgs::msg::JoystickControlStatus & ros_msg);


// Services:

ateam_msgs::srv::SetDesiredKeeper::Request toSetDesiredKeeperRequest(
  const nlohmann::json json);

nlohmann::json fromSrvResponse(
  const ateam_msgs::srv::SetDesiredKeeper::Response & ros_msg);

ateam_msgs::srv::SetPlayEnabled::Request toSetPlayEnabledRequest(
  const nlohmann::json json);

nlohmann::json fromSrvResponse(
  const ateam_msgs::srv::SetPlayEnabled::Response & ros_msg);

ateam_msgs::srv::SetOverridePlay::Request toSetOverridePlayRequest(
  const nlohmann::json json);

nlohmann::json fromSrvResponse(
  const ateam_msgs::srv::SetOverridePlay::Response & ros_msg);

ateam_msgs::srv::SetIgnoreFieldSide::Request toIgnoreFieldSideRequest(
  const nlohmann::json json);

nlohmann::json fromSrvResponse(
  const ateam_msgs::srv::SetIgnoreFieldSide::Response & ros_msg);

ateam_msgs::srv::SendSimulatorControlPacket::Request toSendSimulatorControlPacketRequest(
  const nlohmann::json json);

ssl_league_msgs::msg::SimulatorControl toSimulatorControlMsg(
  const nlohmann::json json);

ssl_league_msgs::msg::TeleportBallCommand toTeleportBallCommandMsg(
  const nlohmann::json json);

ssl_league_msgs::msg::TeleportRobotCommand toTeleportRobotCommandMsg(
  const nlohmann::json json);

ssl_league_msgs::msg::RobotId toRobotIdMsg(
  const nlohmann::json json);

ssl_league_msgs::msg::Team toTeamMsg(
  const nlohmann::json json);

nlohmann::json fromSrvResponse(
  const ateam_msgs::srv::SendSimulatorControlPacket::Response & ros_msg);


// Standard ROS Messages
nlohmann::json fromMsg(
  const builtin_interfaces::msg::Time & ros_msg);

nlohmann::json fromMsg(
  const geometry_msgs::msg::Point32 & ros_msg);

nlohmann::json fromMsg(
  const geometry_msgs::msg::Point & ros_msg);

geometry_msgs::msg::Point toPointMsg(
  const nlohmann::json json);

nlohmann::json fromMsg(
  const geometry_msgs::msg::Vector3 & ros_msg);

geometry_msgs::msg::Vector3 toVector3Msg(
  const nlohmann::json json);

nlohmann::json fromMsg(
  const geometry_msgs::msg::Quaternion & ros_msg);

geometry_msgs::msg::Quaternion toQuaternionMsg(
  const nlohmann::json json);

nlohmann::json fromMsg(
  const geometry_msgs::msg::Pose & ros_msg);

geometry_msgs::msg::Pose toPoseMsg(
  const nlohmann::json json);

nlohmann::json fromMsg(
  const geometry_msgs::msg::Twist & ros_msg);

geometry_msgs::msg::Twist toTwistMsg(
  const nlohmann::json json);

nlohmann::json fromMsg(
  const geometry_msgs::msg::Accel & ros_msg);

nlohmann::json fromMsg(
  const geometry_msgs::msg::Polygon & ros_msg);

}  // namespace ateam_ui_backend_node::message_conversions

#endif  // MESSAGE_CONVERSIONS_HPP_
