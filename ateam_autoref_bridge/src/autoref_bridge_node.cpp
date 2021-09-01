#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <ateam_common/multicast_receiver.hpp>
#include <ssl_league_msgs/msg/referee.hpp>

#include "message_conversions.hpp"

namespace ateam_autoref_bridge
{

class AutorefBridgeNode : public rclcpp::Node
{
public:
  explicit AutorefBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("autoref_bridge", options),
    multicast_receiver_("224.5.23.1",
      10003,
      [this](auto * buffer, size_t bytes_received) {
        Referee referee_proto;
        if (!referee_proto.ParseFromArray(buffer, bytes_received)) {
          return false;
        }

        referee_publisher_->publish(message_conversions::fromProto(referee_proto));
        return true;
      })
  {
    referee_publisher_ = create_publisher<ssl_league_msgs::msg::Referee>(
      "~/referee_messages",
      rclcpp::SystemDefaultsQoS());
  }

private:
  rclcpp::Publisher<ssl_league_msgs::msg::Referee>::SharedPtr referee_publisher_;
  ateam_common::MulticastReceiver multicast_receiver_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_autoref_bridge::AutorefBridgeNode)
