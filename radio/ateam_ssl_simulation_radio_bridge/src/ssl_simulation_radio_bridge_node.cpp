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

#include <ssl_league_protobufs/ssl_simulation_robot_control.pb.h>
#include <ssl_league_protobufs/ssl_simulation_robot_feedback.pb.h>
#include <ssl_league_protobufs/ssl_simulation_control.pb.h>

#include <array>
#include <string>
#include <functional>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <ateam_common/bi_directional_udp.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_common/protobuf_logging.hpp>
#include <ateam_radio_msgs/msg/basic_telemetry.hpp>
#include <ateam_radio_msgs/msg/connection_status.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/srv/send_simulator_control_packet.hpp>

#include "message_conversions.hpp"

namespace ateam_ssl_simulation_radio_bridge
{

class SSLSimulationRadioBridgeNode : public rclcpp::Node
{
public:
  explicit SSLSimulationRadioBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ateam_ssl_simulation_radio_bridge", options),
    gc_listener_(*this,
      std::bind_front(&SSLSimulationRadioBridgeNode::team_color_change_callback, this))
  {
    using ateam_common::indexed_topic_helpers::create_indexed_publishers;
    using ateam_common::indexed_topic_helpers::create_indexed_subscribers;

    SET_ROS_PROTOBUF_LOG_HANDLER("ateam_ssl_simulation_radio_bridge.protobuf");

    declare_parameter("ssl_sim_radio_ip", "127.0.0.1");
    declare_parameter("ssl_sim_control_port", 10300);
    declare_parameter("ssl_sim_blue_port", 10301);
    declare_parameter("ssl_sim_yellow_port", 10302);

    team_color_change_callback(ateam_common::TeamColor::Blue);

    create_indexed_subscribers
    <ateam_msgs::msg::RobotMotionCommand>(
      command_subscriptions_,
      Topics::kRobotMotionCommandPrefix,
      rclcpp::SystemDefaultsQoS(),
      &SSLSimulationRadioBridgeNode::message_callback,
      this);

    create_indexed_publishers<ateam_radio_msgs::msg::BasicTelemetry>(
      feedback_publishers_,
      Topics::kRobotFeedbackPrefix,
      rclcpp::SystemDefaultsQoS(),
      this);

    create_indexed_publishers<ateam_radio_msgs::msg::ConnectionStatus>(
      connection_publishers_,
      Topics::kRobotConnectionStatusPrefix,
      rclcpp::SystemDefaultsQoS(),
      this);

    send_simulator_control_service_ =
      create_service<ateam_msgs::srv::SendSimulatorControlPacket>("~/send_simulator_control_packet",
        std::bind(&SSLSimulationRadioBridgeNode::handle_send_simulator_control, this,
        std::placeholders::_1, std::placeholders::_2));

    std::ranges::fill(command_timestamps_, std::chrono::steady_clock::now());
    zero_command_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(
        declare_parameter(
          "command_timeout_ms",
          100)),
      std::bind(&SSLSimulationRadioBridgeNode::zero_command_timer_callback, this));


    udp_sim_control_ = std::make_unique<ateam_common::BiDirectionalUDP>(
      get_parameter(
        "ssl_sim_radio_ip").as_string(), get_parameter("ssl_sim_control_port").as_int(),
      [](const uint8_t *, size_t) {}
    );
  }

  void handle_send_simulator_control(
    ateam_msgs::srv::SendSimulatorControlPacket::Request::SharedPtr request,
    ateam_msgs::srv::SendSimulatorControlPacket::Response::SharedPtr response)
  {
    if (!udp_sim_control_) {
      response->reason = "Simulation Radio Bridge UDP port not connect";
      return;
    }

    SimulatorCommand simulator_command;
    SimulatorControl * proto_simulator_control = simulator_command.mutable_control();

    try {
      *proto_simulator_control = message_conversions::fromMsg(request->simulator_control);
    } catch (const std::invalid_argument & e) {
      response->reason = e.what();
    }

    std::vector<uint8_t> buffer;
    buffer.resize(simulator_command.ByteSizeLong());
    if (simulator_command.SerializeToArray(buffer.data(), buffer.size())) {
      udp_sim_control_->send(static_cast<uint8_t *>(buffer.data()), buffer.size());

      response->success = true;
    } else {
      response->reason = "Failed to serialize protobuf packet";
    }

    return;
  }

  void team_color_change_callback(const ateam_common::TeamColor color)
  {
    int port = 0;
    switch (color) {
      case ateam_common::TeamColor::Blue:
        port = get_parameter("ssl_sim_blue_port").as_int();
        break;
      case ateam_common::TeamColor::Yellow:
        port = get_parameter("ssl_sim_yellow_port").as_int();
        break;
      case ateam_common::TeamColor::Unknown:
        RCLCPP_WARN(get_logger(), "Unknown team color. Robot radio connection disabled.");
        udp_robot_control_.reset();
        return;
    }
    RCLCPP_INFO(get_logger(), "Changing radio port to %d", port);
    udp_robot_control_ = std::make_unique<ateam_common::BiDirectionalUDP>(
      get_parameter(
        "ssl_sim_radio_ip").as_string(), port,
      std::bind_front(&SSLSimulationRadioBridgeNode::feedback_callback, this));
  }

  void send_command(const ateam_msgs::msg::RobotMotionCommand & msg, const int robot_id)
  {
    if (!udp_robot_control_) {
      return;
    }
    RobotControl robots_control = message_conversions::fromMsg(msg, robot_id);
    std::vector<uint8_t> buffer;
    buffer.resize(robots_control.ByteSizeLong());
    if (robots_control.SerializeToArray(buffer.data(), buffer.size())) {
      udp_robot_control_->send(static_cast<uint8_t *>(buffer.data()), buffer.size());
    }
  }

  void message_callback(
    const ateam_msgs::msg::RobotMotionCommand::SharedPtr robot_commands_msg,
    int robot_id)
  {
    command_timestamps_[robot_id] = std::chrono::steady_clock::now();
    send_command(*robot_commands_msg, robot_id);
  }

  void feedback_callback(const uint8_t * buffer, size_t bytes_received)
  {
    if (bytes_received <= 0) {
      // ignore empty feedback packets
      return;
    }

    RobotControlResponse feedback_proto;
    if (!feedback_proto.ParseFromArray(buffer, bytes_received)) {
      RCLCPP_WARN(get_logger(), "Failed to parse robot feedback protobuf packet");
      return;
    }

    for (const auto & single_feedback : feedback_proto.feedback()) {
      int robot_id = single_feedback.id();
      feedback_publishers_.at(robot_id)->publish(message_conversions::fromProto(single_feedback));
      ateam_radio_msgs::msg::ConnectionStatus connection_msg;
      connection_msg.radio_connected = true;
      connection_publishers_.at(robot_id)->publish(connection_msg);
    }
  }

  void send_zero_command(const int id)
  {
    ateam_msgs::msg::RobotMotionCommand msg;
    msg.dribbler_speed = 0.0;
    msg.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_DISABLE;
    msg.twist.linear.x = 0.0;
    msg.twist.angular.z = 0.0;
    send_command(msg, id);
  }

  void zero_command_timer_callback()
  {
    const auto timeout_duration = std::chrono::milliseconds(
      get_parameter(
        "command_timeout_ms").as_int());
    const auto timeout_time = std::chrono::steady_clock::now() - timeout_duration;
    for (auto id = 0; id < 16; ++id) {
      if (command_timestamps_[id] < timeout_time) {
        send_zero_command(id);
      }
    }
  }

private:
  ateam_common::GameControllerListener gc_listener_;
  std::unique_ptr<ateam_common::BiDirectionalUDP> udp_robot_control_;
  std::unique_ptr<ateam_common::BiDirectionalUDP> udp_sim_control_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotMotionCommand>::SharedPtr,
    16> command_subscriptions_;
  std::array<rclcpp::Publisher<ateam_radio_msgs::msg::BasicTelemetry>::SharedPtr,
    16> feedback_publishers_;
  std::array<rclcpp::Publisher<ateam_radio_msgs::msg::ConnectionStatus>::SharedPtr,
    16> connection_publishers_;
  rclcpp::Service<ateam_msgs::srv::SendSimulatorControlPacket>::SharedPtr
    send_simulator_control_service_;
  rclcpp::TimerBase::SharedPtr zero_command_timer_;
  std::array<std::chrono::steady_clock::time_point, 16> command_timestamps_;
};

}  // namespace ateam_ssl_simulation_radio_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_ssl_simulation_radio_bridge::SSLSimulationRadioBridgeNode)
