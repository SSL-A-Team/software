#include <array>
#include <chrono>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/robot_feedback.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_common/multicast_receiver.hpp>
#include <ateam_common/bi_directional_udp.hpp>
#include <ateam_common/game_controller_listener.hpp>

#include "rnp_packet_helpers.hpp"
#include "conversion.hpp"
#include "ip_address_helpers.hpp"

// TODO(barulicm) add warning if we see another instance of this running via multicast

namespace ateam_radio_bridge
{

class RadioBridgeNode : public rclcpp::Node
{
public:
  RadioBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("radio_bridge", options),
    timeout_threshold_(declare_parameter("timeout_ms", 250)),
    command_timeout_threshold_(declare_parameter("command_timeout_ms", 100)),
    game_controller_listener_(*this),
    discovery_receiver_(declare_parameter<std::string>("discovery_address", "224.4.20.69"),
      declare_parameter<uint16_t>("discovery_port", 42069),
      std::bind(&RadioBridgeNode::DiscoveryMessageCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      declare_parameter<std::string>("net_interface_address","172.16.1.10"))
  {
    ateam_common::indexed_topic_helpers::create_indexed_subscribers<ateam_msgs::msg::RobotMotionCommand>(
      motion_command_subscriptions_,
      "~/robot_motion_commands/robot",
      rclcpp::SystemDefaultsQoS(),
      &RadioBridgeNode::motion_command_callback,
      this);

    ateam_common::indexed_topic_helpers::create_indexed_publishers<ateam_msgs::msg::RobotFeedback>(
      feedback_publishers_,
      "~/robot_feedback/robot",
      rclcpp::SystemDefaultsQoS(),
      this);

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
  std::array<ateam_msgs::msg::RobotMotionCommand, 16> motion_commands_;
  std::array<std::chrono::steady_clock::time_point, 16> motion_command_timestamps_;
  ateam_common::GameControllerListener game_controller_listener_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotMotionCommand>::SharedPtr,
    16> motion_command_subscriptions_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::RobotFeedback>::SharedPtr, 16> feedback_publishers_;
  ateam_common::MulticastReceiver discovery_receiver_;
  std::array<std::unique_ptr<ateam_common::BiDirectionalUDP>, 16> connections_;
  std::array<std::chrono::steady_clock::time_point, 16> last_heartbeat_timestamp_;
  rclcpp::TimerBase::SharedPtr connection_check_timer_;
  rclcpp::TimerBase::SharedPtr command_send_timer_;

  void motion_command_callback(
    const ateam_msgs::msg::RobotMotionCommand::SharedPtr command_msg,
    int robot_id)
  {
    motion_commands_[robot_id] = *command_msg;
    motion_command_timestamps_[robot_id] = std::chrono::steady_clock::now();
  }

  void CloseConnection(const std::size_t & connection_index)
  {
    auto & connection = connections_.at(connection_index);
    RCLCPP_INFO(
      get_logger(), "Closing connection to robot %ld (%s:%d)", connection_index,
      connection->GetRemoteIPAddress().c_str(), connection->GetRemotePort());
    const auto packet = CreateEmptyPacket(CC_GOODBYE);
    connection->send(
      reinterpret_cast<const uint8_t *>(&packet),
      GetPacketSize(packet.command_code));
    // Give some time for the message to actually send before closing the connection
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    connection.reset();
  }

  /**
   * Close any connections whose last heartbeat time is too old.
   * Send 'not connected' messages for appropriate robots.
   */
  void ConnectionCheckCallback()
  {
    for (auto i = 0ul; i < connections_.size(); ++i) {
      if (!connections_[i]) {
        ateam_msgs::msg::RobotFeedback feedback_message;
        feedback_message.radio_connected = false;
        feedback_publishers_[i]->publish(feedback_message);
        continue;
      }
      const auto & last_heartbeat_time = last_heartbeat_timestamp_[i];
      const auto time_since_heartbeat = std::chrono::steady_clock::now() - last_heartbeat_time;
      if (time_since_heartbeat > timeout_threshold_) {
        RCLCPP_WARN(get_logger(), "Connection to robot %ld timed out.", i);
        CloseConnection(i);
      }
    }
  }

  void SendCommandsCallback()
  {
    for (auto id = 0; id < 16; ++id) {
      if (connections_[id] == nullptr) {
        continue;
      }
      if((std::chrono::steady_clock::now() - motion_command_timestamps_[id]) > command_timeout_threshold_) {
        continue;
      }
      BasicControl control_msg;
      control_msg.vel_x_linear = motion_commands_[id].twist.linear.x;
      control_msg.vel_y_linear = motion_commands_[id].twist.linear.y;
      control_msg.vel_z_angular = motion_commands_[id].twist.angular.z;
      control_msg.kick_vel = 0.0f;
      control_msg.dribbler_speed = motion_commands_[id].dribbler_speed;
      if (motion_commands_[id].kick == 0) {
        control_msg.kick_request = KR_ARM;
      } else if (motion_commands_[id].kick == 1){
        control_msg.kick_request = KR_KICK_TOUCH;
      } else {
        control_msg.kick_request = KR_KICK_NOW;
      }
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
    RCLCPP_INFO(get_logger(), "Multicast packet received!");
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
      return;
    }

    if (connections_[robot_id] != nullptr &&
      sender_address != connections_[robot_id]->GetRemoteIPAddress())
    {
      // a connection for this robot ID already exists with a different robot
      const auto reply_packet = CreateEmptyPacket(CC_NACK);
      discovery_receiver_.SendTo(
        sender_address, sender_port,
        reinterpret_cast<const char *>(&reply_packet), GetPacketSize(reply_packet.command_code));
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
      RCLCPP_WARN(get_logger(), "Ignoring telemetry message. %s", error.c_str());
      return;
    }

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
            RCLCPP_WARN(get_logger(), "Ignoring telemetry message. %s", error.c_str());
            return;
          }
          if (std::holds_alternative<BasicTelemetry>(data_var)) {
            auto msg = Convert(std::get<BasicTelemetry>(data_var));
            msg.radio_connected = true;
            feedback_publishers_[robot_id]->publish(msg);
          }
          break;
        }
      case CC_KEEPALIVE:
        last_heartbeat_timestamp_[robot_id] = std::chrono::steady_clock::now();
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "Ignoring telemetry message. Unsupported command code: %d",
          packet.command_code);
        return;
    }

    motion_command_timestamps_[robot_id] = std::chrono::steady_clock::now();
  }

  void TeamColorChangeCallback(const ateam_common::TeamColor)
  {
    for (auto i = 0ul; i < connections_.size(); ++i) {
      CloseConnection(i);
    }
  }

};

}  // namespace ateam_radio_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_radio_bridge::RadioBridgeNode)
