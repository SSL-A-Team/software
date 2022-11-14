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
#include <ateam_common/team_color_listener.hpp>

#include "rnp_packet_helpers.hpp"
#include "conversion.hpp"

namespace ateam_radio_bridge
{

class RadioBridgeNode : public rclcpp::Node
{
public:
  RadioBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("radio_bridge", options),
    timeout_threshold_(declare_parameter("timeout_ms", 100)),
    color_listener_(*this),
    discovery_receiver_(declare_parameter<std::string>("discovery_address", "224.4.20.69"),
      declare_parameter<uint16_t>("discovery_port", 42069),
      std::bind(&RadioBridgeNode::DiscoveryMessageCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4))
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
      std::chrono::milliseconds(100),
      std::bind(&RadioBridgeNode::ConnectionCheckCallback, this));

    command_send_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&RadioBridgeNode::SendCommandsCallback, this));

    RCLCPP_INFO(get_logger(), "Radio bridge node ready.");
  }

private:
  const std::chrono::milliseconds timeout_threshold_;
  std::array<ateam_msgs::msg::RobotMotionCommand, 16> motion_commands_;
  ateam_common::TeamColorListener color_listener_;
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
  }

  void CloseConnection(std::unique_ptr<ateam_common::BiDirectionalUDP> & connection)
  {
    const auto packet = CreateEmptyPacket(CC_GOODBYE);
    connection->send(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    // Give some time for the message to actually send before closing the connection
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    connection.reset();
  }

  void ConnectionCheckCallback()
  {
    // Close any connections whose last heartbeat time is too old
    for (auto i = 0ul; i < connections_.size(); ++i) {
      if (!connections_[i]) {
        continue;
      }
      const auto & last_heartbeat_time = last_heartbeat_timestamp_[i];
      const auto time_since_heartbeat = std::chrono::steady_clock::now() - last_heartbeat_time;
      if (time_since_heartbeat > timeout_threshold_) {
        CloseConnection(connections_[i]);
      }
    }
  }

  void SendCommandsCallback()
  {
    for (auto id = 0; id < 16; ++id) {
      if (connections_[id] == nullptr) {
        continue;
      }
      BasicControl_t control_msg;
      control_msg.vel_x_linear = motion_commands_[id].twist.linear.x;
      control_msg.vel_y_linear = motion_commands_[id].twist.linear.y;
      control_msg.vel_z_angular = motion_commands_[id].twist.angular.z;
      const auto control_packet = CreatePacket(CC_CONTROL, control_msg);
      connections_[id]->send(
        reinterpret_cast<const uint8_t *>(&control_packet),
        sizeof(control_packet));
    }
  }

  void DiscoveryMessageCallback(
    const std::string & sender_address, const uint16_t sender_port,
    uint8_t * udp_packet_data, size_t udp_packet_size)
  {
    std::string parsing_error;
    RadioPacket_t packet = ParsePacket(udp_packet_data, udp_packet_size, parsing_error);
    if (!parsing_error.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring discovery packet. %s", parsing_error.c_str());
      return;
    }

    if (packet.command_code != CC_HELLO) {
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

    if (!std::holds_alternative<HelloData_t>(data_variant)) {
      RCLCPP_WARN(get_logger(), "Ignoring discovery packet. Unexpected data type.");
      return;
    }

    HelloData_t hello_data = std::get<HelloData_t>(data_variant);

    if (!(color_listener_.GetTeamColor() == ateam_common::TeamColorListener::TeamColor::Blue &&
      hello_data.color == TC_BLUE) &&
      !(color_listener_.GetTeamColor() == ateam_common::TeamColorListener::TeamColor::Yellow &&
      hello_data.color == TC_YELLOW))
    {
      // Quietly ignore discovery packets for the other team
      return;
    }

    if (hello_data.robot_id > connections_.size() ||
      connections_[hello_data.robot_id] != nullptr)
    {
      // A connection already exists for this jersey number
      const auto reply_packet = CreateEmptyPacket(CC_NACK);
      discovery_receiver_.SendTo(
        sender_address, sender_port,
        reinterpret_cast<const char *>(&reply_packet), sizeof(reply_packet));
      return;
    }

    connections_[hello_data.robot_id] = std::make_unique<ateam_common::BiDirectionalUDP>(
      sender_address, sender_port,
      std::bind(
        &RadioBridgeNode::RobotIncomingPacketCallback, this, hello_data.robot_id,
        std::placeholders::_1, std::placeholders::_2));

    const auto reply_packet = CreateEmptyPacket(CC_ACK);
    connections_[hello_data.robot_id]->send(
      reinterpret_cast<const uint8_t *>(&reply_packet),
      sizeof(reply_packet));
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
          if (std::holds_alternative<BasicTelemetry_t>(data_var)) {
            feedback_publishers_[robot_id]->publish(Convert(std::get<BasicTelemetry_t>(data_var)));
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
  }

  void TeamColorChangeCallback(const ateam_common::TeamColorListener::TeamColor)
  {
    std::for_each(
      connections_.begin(), connections_.end(),
      std::bind(&RadioBridgeNode::CloseConnection, this, std::placeholders::_1));
  }

};

}  // namespace ateam_radio_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_radio_bridge::RadioBridgeNode)
