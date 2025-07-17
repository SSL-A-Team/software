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


#include <array>
#include <chrono>
#include <numeric>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_radio_msgs/msg/connection_status.hpp>
#include <ateam_radio_msgs/msg/basic_telemetry.hpp>
#include <ateam_radio_msgs/msg/extended_telemetry.hpp>
#include <ateam_radio_msgs/srv/get_firmware_parameter.hpp>
#include <ateam_radio_msgs/srv/set_firmware_parameter.hpp>
#include <ateam_radio_msgs/srv/send_robot_power_request.hpp>
#include <ateam_radio_msgs/conversion.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_common/multicast_receiver.hpp>
#include <ateam_common/bi_directional_udp.hpp>
#include <ateam_common/game_controller_listener.hpp>

#include "rnp_packet_helpers.hpp"
#include "ip_address_helpers.hpp"
#include "firmware_parameter_server.hpp"

// TODO(barulicm) add warning if we see another instance of this running via multicast

using namespace std::string_literals;

namespace ateam_radio_bridge
{

class RadioBridgeNode : public rclcpp::Node
{
public:
  RadioBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("radio_bridge", options),
    timeout_threshold_(declare_parameter("timeout_ms", 250)),
    command_timeout_threshold_(declare_parameter("command_timeout_ms", 100)),
    game_controller_listener_(*this,
      std::bind_front(&RadioBridgeNode::TeamColorChangeCallback, this)),
    discovery_receiver_(declare_parameter<std::string>("discovery_address", "224.4.20.69"),
      declare_parameter<uint16_t>("discovery_port", 42069),
      std::bind(&RadioBridgeNode::DiscoveryMessageCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      declare_parameter<std::string>("net_interface_address", "")),
    firmware_parameter_server_(*this, connections_)
  {
    declare_parameters<bool>("controls_enabled", {
        {"body_vel", true},
        {"wheel_vel", true},
        {"wheel_torque", false}
    });

    declare_parameter<bool>("shut_down_robots", false);
    declare_parameter<bool>("reboot_robots", false);

    ateam_common::indexed_topic_helpers::create_indexed_subscribers<ateam_msgs::msg::RobotMotionCommand>(
      motion_command_subscriptions_,
      "~/robot_motion_commands/robot",
      rclcpp::SystemDefaultsQoS(),
      &RadioBridgeNode::MotionCommandCallback,
      this);

    ateam_common::indexed_topic_helpers::create_indexed_publishers<ateam_radio_msgs::msg::ConnectionStatus>(
      connection_publishers_,
      "~/robot_feedback/connection/robot",
      rclcpp::SystemDefaultsQoS(),
      this);

    ateam_common::indexed_topic_helpers::create_indexed_publishers<ateam_radio_msgs::msg::BasicTelemetry>(
      feedback_publishers_,
      "~/robot_feedback/basic/robot",
      rclcpp::SystemDefaultsQoS(),
      this);

    ateam_common::indexed_topic_helpers::create_indexed_publishers<ateam_radio_msgs::msg::ExtendedTelemetry>(
      motion_feedback_publishers_,
      "~/robot_feedback/extended/robot",
      rclcpp::SystemDefaultsQoS(),
      this);

    power_request_service_ = create_service<ateam_radio_msgs::srv::SendRobotPowerRequest>(
      "~/send_power_request",
      std::bind(&RadioBridgeNode::SendPowerRequestCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    connection_check_timer_ =
      create_wall_timer(
      std::chrono::duration<double>(
        1.0 /
        declare_parameter<double>("connection_check_frequency", 10.0)),
      std::bind(&RadioBridgeNode::ConnectionCheckCallback, this));

    command_send_timer_ =
      create_wall_timer(
      std::chrono::duration<double>(1.0 / declare_parameter<double>("command_frequency", 60.0)),
      std::bind(&RadioBridgeNode::SendCommandsCallback, this));

    RCLCPP_INFO(get_logger(), "Radio bridge node ready.");
  }

private:
  const std::chrono::milliseconds timeout_threshold_;
  const std::chrono::milliseconds command_timeout_threshold_;
  std::mutex mutex_;
  std::array<ateam_msgs::msg::RobotMotionCommand, 16> motion_commands_;
  std::array<std::chrono::steady_clock::time_point, 16> motion_command_timestamps_;
  std::array<bool, 16> shutdown_requested_;
  std::array<bool, 16> reboot_requested_;
  ateam_common::GameControllerListener game_controller_listener_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotMotionCommand>::SharedPtr,
    16> motion_command_subscriptions_;
  std::array<rclcpp::Publisher<ateam_radio_msgs::msg::ConnectionStatus>::SharedPtr,
    16> connection_publishers_;
  std::array<rclcpp::Publisher<ateam_radio_msgs::msg::BasicTelemetry>::SharedPtr,
    16> feedback_publishers_;
  std::array<rclcpp::Publisher<ateam_radio_msgs::msg::ExtendedTelemetry>::SharedPtr,
    16> motion_feedback_publishers_;
  ateam_common::MulticastReceiver discovery_receiver_;
  FirmwareParameterServer firmware_parameter_server_;
  rclcpp::Service<ateam_radio_msgs::srv::SendRobotPowerRequest>::SharedPtr power_request_service_;
  std::array<std::unique_ptr<ateam_common::BiDirectionalUDP>, 16> connections_;
  std::array<std::chrono::steady_clock::time_point, 16> last_heartbeat_timestamp_;
  rclcpp::TimerBase::SharedPtr connection_check_timer_;
  rclcpp::TimerBase::SharedPtr command_send_timer_;

  void ReplaceNanWithZero(double & val) {
    if (std::isnan(val)) {
      RCLCPP_WARN(get_logger(), "Radio bridge is replacing NaNs!");
      val = 0.0;
    }
  }

  void MotionCommandCallback(
    const ateam_msgs::msg::RobotMotionCommand::SharedPtr command_msg,
    int robot_id)
  {
    const std::lock_guard lock(mutex_);
    motion_commands_[robot_id] = *command_msg;
    auto & command = motion_commands_[robot_id];
    ReplaceNanWithZero(command.twist.linear.x);
    ReplaceNanWithZero(command.twist.linear.y);
    ReplaceNanWithZero(command.twist.angular.z);
    motion_command_timestamps_[robot_id] = std::chrono::steady_clock::now();
  }

  void CloseConnection(const std::size_t & connection_index)
  {
    std::unique_ptr<ateam_common::BiDirectionalUDP> connection;
    {
      std::lock_guard lock(mutex_);
      connections_.at(connection_index).swap(connection);
    }
    if(!connection) {
      // Connection already closed
      // lock released by destructor
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Closing connection to robot %ld (%s:%d)", connection_index,
      connection->GetRemoteIPAddress().c_str(), connection->GetRemotePort());
    const auto packet = CreateEmptyPacket(CC_GOODBYE);
    connection->send(
      reinterpret_cast<const uint8_t *>(&packet),
      GetPacketSize(packet.command_code));
    // Give some time for the message to actually send before closing the connection
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  /**
   * Close any connections whose last heartbeat time is too old.
   * Send 'not connected' messages for appropriate robots.
   */
  void ConnectionCheckCallback()
  {
    for (auto i = 0ul; i < connections_.size(); ++i) {
      std::unique_lock lock(mutex_);
      if (!connections_[i]) {
        ateam_radio_msgs::msg::ConnectionStatus connection_message;
        connection_message.radio_connected = false;
        connection_publishers_[i]->publish(connection_message);
        shutdown_requested_[i] = false;
        reboot_requested_[i] = false;
        continue;
      }
      const auto & last_heartbeat_time = last_heartbeat_timestamp_[i];
      const auto time_since_heartbeat = std::chrono::steady_clock::now() - last_heartbeat_time;
      if (time_since_heartbeat > timeout_threshold_) {
        RCLCPP_WARN(get_logger(), "Connection to robot %ld timed out.", i);
        // release lock early so CloseConnection can grab it
        lock.unlock();
        CloseConnection(i);
      }
      // lock released by destructor
    }
  }

  void SendCommandsCallback()
  {
    const std::lock_guard lock(mutex_);
    for (auto id = 0; id < 16; ++id) {
      if (connections_[id] == nullptr) {
        continue;
      }
      if ((std::chrono::steady_clock::now() - motion_command_timestamps_[id]) >
        command_timeout_threshold_)
      {
        RCLCPP_WARN(get_logger(), "Robot %d command topic inactive. Sending zeros.", id);
        motion_commands_[id] = ateam_msgs::msg::RobotMotionCommand();
        motion_commands_[id].kick_request = ateam_msgs::msg::RobotMotionCommand::KR_DISABLE;
      }
      BasicControl control_msg;
      control_msg.request_shutdown = shutdown_requested_[id];
      control_msg.reboot_robot = reboot_requested_[id];
      control_msg.game_state_in_stop = game_controller_listener_.GetGameCommand() ==
        ateam_common::GameCommand::Stop;
      control_msg.emergency_stop = false;
      control_msg.body_vel_controls_enabled = get_parameter("controls_enabled.body_vel").as_bool();
      control_msg.wheel_vel_control_enabled = get_parameter("controls_enabled.wheel_vel").as_bool();
      control_msg.wheel_torque_control_enabled =
        get_parameter("controls_enabled.wheel_torque").as_bool();
      control_msg.play_song = 0;
      control_msg.vel_x_linear = motion_commands_[id].twist.linear.x;
      control_msg.vel_y_linear = motion_commands_[id].twist.linear.y;
      control_msg.vel_z_angular = motion_commands_[id].twist.angular.z;
      control_msg.dribbler_speed = motion_commands_[id].dribbler_speed;
      control_msg.dribbler_multiplier = 75;
      control_msg.kick_request = static_cast<KickRequest>(motion_commands_[id].kick_request);
      control_msg.kick_vel = motion_commands_[id].kick_speed;
      const auto control_packet = CreatePacket(CC_CONTROL, control_msg);
      connections_[id]->send(
        reinterpret_cast<const uint8_t *>(&control_packet),
        GetPacketSize(control_packet.command_code));
    }
  }

  void DiscoveryMessageCallback(
    const std::string & sender_address, const uint16_t sender_port,
    uint8_t * udp_packet_data, size_t udp_packet_size)
  {
    std::string parsing_error;
    RadioPacket packet = ParsePacket(udp_packet_data, udp_packet_size, parsing_error);
    if (!parsing_error.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring discovery packet. %s", parsing_error.c_str());
      return;
    }

    if (packet.command_code != CC_HELLO_REQ) {
      RCLCPP_WARN(
        get_logger(), "Ignoring discovery packet. Unexpected command code: %d",
        packet.command_code);
      return;
    }

    auto data_variant = ExtractData(packet, parsing_error);
    if (!parsing_error.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring discovery packet. %s", parsing_error.c_str());
      return;
    }

    if (!std::holds_alternative<HelloRequest>(data_variant)) {
      RCLCPP_WARN(get_logger(), "Ignoring discovery packet. Unexpected data type.");
      return;
    }

    HelloRequest hello_data = std::get<HelloRequest>(data_variant);

    if (!(game_controller_listener_.GetTeamColor() == ateam_common::TeamColor::Blue &&
      hello_data.color == TC_BLUE) &&
      !(game_controller_listener_.GetTeamColor() == ateam_common::TeamColor::Yellow &&
      hello_data.color == TC_YELLOW))
    {
      RCLCPP_WARN(get_logger(), "Ignoring discovery packet. Wrong team.");
      // Quietly ignore discovery packets for the other team
      return;
    }

    const auto robot_id = hello_data.robot_id;

    if (robot_id > connections_.size()) {
      // invalid robot ID requested
      const auto reply_packet = CreateEmptyPacket(CC_NACK);
      discovery_receiver_.SendTo(
        sender_address, sender_port,
        reinterpret_cast<const char *>(&reply_packet), GetPacketSize(reply_packet.command_code));
      RCLCPP_WARN(get_logger(), "Rejecting discovery packet. Invalid robot ID: %d", robot_id);
      return;
    }

    const std::lock_guard lock(mutex_);

    if (connections_[robot_id] != nullptr &&
      sender_address != connections_[robot_id]->GetRemoteIPAddress())
    {
      // a connection for this robot ID already exists with a different robot
      const auto reply_packet = CreateEmptyPacket(CC_NACK);
      discovery_receiver_.SendTo(
        sender_address, sender_port,
        reinterpret_cast<const char *>(&reply_packet), GetPacketSize(reply_packet.command_code));
      RCLCPP_WARN(get_logger(), "Rejecting discovery packet. Robot ID already connected: %d",
          robot_id);
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Creating connection for robot %d (%s:%d)", robot_id,
      sender_address.c_str(), sender_port);

    motion_command_timestamps_[robot_id] = {};
    last_heartbeat_timestamp_[robot_id] = std::chrono::steady_clock::now();
    connections_[hello_data.robot_id] = std::make_unique<ateam_common::BiDirectionalUDP>(
      sender_address, sender_port,
      std::bind(
        &RadioBridgeNode::RobotIncomingPacketCallback, this, hello_data.robot_id,
        std::placeholders::_1, std::placeholders::_2));

    HelloResponse response;
    response.port = connections_[hello_data.robot_id]->GetLocalPort();
    const auto local_ip_address = GetClosestIpAddress(GetIpAddresses(), sender_address);
    const auto local_address_bytes =
      boost::asio::ip::make_address(local_ip_address).to_v4().to_bytes();
    std::copy_n(local_address_bytes.begin(), 4, response.ipv4);

    const auto reply_packet = CreatePacket(CC_HELLO_RESP, response);
    discovery_receiver_.SendTo(
      sender_address, sender_port,
      reinterpret_cast<const char *>(&reply_packet), GetPacketSize(reply_packet.command_code));
  }

  void RobotIncomingPacketCallback(
    const uint8_t robot_id, const uint8_t * udp_packet_data,
    const std::size_t udp_packet_size)
  {
    std::string error;
    const auto packet = ParsePacket(udp_packet_data, udp_packet_size, error);
    if (!error.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring incoming message from robot %d. %s", robot_id, error.c_str());
      return;
    }

    const std::lock_guard lock(mutex_);

    switch (packet.command_code) {
      case CC_GOODBYE:
        // close connection. No need to send our own goodbye
        connections_[robot_id].reset();
        break;
      case CC_TELEMETRY:
        {
          last_heartbeat_timestamp_[robot_id] = std::chrono::steady_clock::now();
          const auto data_var = ExtractData(packet, error);
          if (!error.empty()) {
            RCLCPP_WARN(get_logger(), "Ignoring basic telemetry message from robot %d. %s", robot_id, error.c_str());
            return;
          }
          if (std::holds_alternative<BasicTelemetry>(data_var)) {
            feedback_publishers_[robot_id]->publish(ateam_radio_msgs::Convert(
              std::get<BasicTelemetry>(data_var)));
            ateam_radio_msgs::msg::ConnectionStatus connection_message;
            connection_message.radio_connected = true;
            connection_publishers_[robot_id]->publish(connection_message);
          }
          break;
        }
      case CC_CONTROL_DEBUG_TELEMETRY:
        {
          const auto data_var = ExtractData(packet, error);
          if (!error.empty()) {
            RCLCPP_WARN(get_logger(), "Ignoring extended telemetry message from robot %d. %s", robot_id, error.c_str());
            return;
          }

          if (std::holds_alternative<ExtendedTelemetry>(data_var)) {
            motion_feedback_publishers_[robot_id]->publish(ateam_radio_msgs::Convert(
              std::get<ExtendedTelemetry>(data_var)));
          }
          break;
        }
      case CC_ROBOT_PARAMETER_COMMAND:
        {
          const auto data_var = ExtractData(packet, error);
          if (!error.empty()) {
            RCLCPP_WARN(get_logger(), "Ignoring parameter command response message from robot %d. %s",
              robot_id, error.c_str());
            return;
          }
          if(std::holds_alternative<ParameterCommand>(data_var)) {
            firmware_parameter_server_.HandleIncomingParameterPacket(robot_id,
              std::get<ParameterCommand>(data_var));
          }
          break;
        }
      case CC_KEEPALIVE:
        last_heartbeat_timestamp_[robot_id] = std::chrono::steady_clock::now();
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "Ignoring telemetry message from robot %d. Unsupported command code: %d",
          robot_id, packet.command_code);
        return;
    }
  }

  void TeamColorChangeCallback(const ateam_common::TeamColor)
  {
    for (auto i = 0ul; i < connections_.size(); ++i) {
      CloseConnection(i);
    }
  }

  void SendPowerRequestCallback(
    const std::shared_ptr<ateam_radio_msgs::srv::SendRobotPowerRequest::Request> request,
    std::shared_ptr<ateam_radio_msgs::srv::SendRobotPowerRequest::Response> response)
  {
    const auto robot_id = request->robot_id;

    if(robot_id == ateam_radio_msgs::srv::SendRobotPowerRequest::Request::ROBOT_ID_ALL) {
      switch (request->request_type) {
        case ateam_radio_msgs::srv::SendRobotPowerRequest::Request::REQUEST_TYPE_SHUTDOWN:
          std::fill(shutdown_requested_.begin(), shutdown_requested_.end(), true);
          break;
        case ateam_radio_msgs::srv::SendRobotPowerRequest::Request::REQUEST_TYPE_REBOOT:
          std::fill(reboot_requested_.begin(), reboot_requested_.end(), true);
          break;
        default:
          response->success = false;
          response->reason = "Invalid request type";
          return;
      }
    } else {
      if ((robot_id >= (int8_t)connections_.size() || robot_id < 0)) {
        response->success = false;
        response->reason = "Invalid robot ID";
        return;
      }

      switch (request->request_type) {
        case ateam_radio_msgs::srv::SendRobotPowerRequest::Request::REQUEST_TYPE_SHUTDOWN:
          shutdown_requested_[robot_id] = true;
          break;
        case ateam_radio_msgs::srv::SendRobotPowerRequest::Request::REQUEST_TYPE_REBOOT:
          reboot_requested_[robot_id] = true;
          break;
        default:
          response->success = false;
          response->reason = "Invalid request type";
          return;
      }
    }

    response->success = true;
  }

};

}  // namespace ateam_radio_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_radio_bridge::RadioBridgeNode)
