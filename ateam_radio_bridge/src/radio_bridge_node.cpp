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

#include "rnp_packet_helpers.hpp"


namespace ateam_radio_bridge
{

class RadioBridgeNode : public rclcpp::Node
{
public:
  RadioBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("radio_bridge", options),
    discovery_receiver_(declare_parameter<std::string>("discovery_address", "172.16.1.255"),
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

    connection_check_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&RadioBridgeNode::ConnectionCheckCallback, this));
  }

private:
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotMotionCommand>::SharedPtr,
    16> motion_command_subscriptions_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::RobotFeedback>::SharedPtr, 16> feedback_publishers_;
  ateam_common::MulticastReceiver discovery_receiver_;
  std::array<std::unique_ptr<ateam_common::BiDirectionalUDP>, 16> connections_;
  std::array<std::chrono::steady_clock::time_point, 16> last_heartbeat_timestamp_;
  rclcpp::TimerBase::SharedPtr connection_check_timer_;

  void motion_command_callback(
    const ateam_msgs::msg::RobotMotionCommand::SharedPtr /*command_msg*/,
    int /*robot_id*/)
  {
    // Translate motion command and send it over the radio
  }

  void ConnectionCheckCallback()
  {
    // Close any connections whose last heartbeat time is too old
  }

  void DiscoveryMessageCallback(
    const std::string & sender_address, const uint16_t sender_port,
    uint8_t * udp_packet_data, size_t udp_packet_size)
  {
    RadioPacket_t packet;
    std::copy_n(udp_packet_data, kPacketHeaderSize, reinterpret_cast<uint8_t *>(&packet));
    if ((packet.data_length + kPacketHeaderSize) > udp_packet_size) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring discovery packet. Claimed packet size is larger than UDP packet size.");
      return;
    }
    std::copy_n(udp_packet_data + kPacketHeaderSize, packet.data_length, packet.data);
    // TODO(mbarulic) check protocol version

    if (packet.command_code != CC_HELLO) {
      RCLCPP_WARN(
        get_logger(), "Ignoring discovery packet. Unexpected command code: %d",
        packet.command_code);
      return;
    }

    if (packet.data_type != DT_HELLO_DATA) {
      RCLCPP_WARN(
        get_logger(), "Ignoring discovery packet. Unexpected data type: %d", packet.data_type);
      return;
    }

    if (packet.data_length != sizeof(HelloData_t)) {
      RCLCPP_WARN(
        get_logger(), "Ignoring discovery packet. Incorrect data payload length: %d",
        packet.data_length);
      return;
    }

    HelloData_t hello_data;
    std::copy_n(packet.data, packet.data_length, reinterpret_cast<uint8_t *>(&hello_data));

    if (hello_data.color != TC_BLUE) { // TODO(mbarulic) get team color from GC
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
    connections_[hello_data.robot_id]->send(reinterpret_cast<const char *>(&reply_packet), sizeof(reply_packet));
  }

  void RobotIncomingPacketCallback(
    const uint8_t robot_id, const char * udp_packet_data,
    const std::size_t udp_packet_size)
  {
    // Parse incoming packet and react accordingly
    // Goodbye packets should close connection
    // Telemetry packets should be published on corresponding topics
  }

};

}  // namespace ateam_radio_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_radio_bridge::RadioBridgeNode)
