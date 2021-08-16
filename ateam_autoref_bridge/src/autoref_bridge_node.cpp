#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ssl_league_msgs/msg/referee.hpp>

#include "referee_multicast_bridge.hpp"

namespace ateam_autoref_bridge
{

class AutorefBridgeNode : public rclcpp::Node
{
public:
  explicit AutorefBridgeNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("autoref_bridge", options),
    multicast_bridge_([this](const auto& msg){ referee_publisher_->publish(msg);})
  {
    referee_publisher_ = create_publisher<ssl_league_msgs::msg::Referee>("~/referee_messages", rclcpp::SystemDefaultsQoS());
  }

private:
  rclcpp::Publisher<ssl_league_msgs::msg::Referee>::SharedPtr referee_publisher_;
  RefereeMulticastBridge multicast_bridge_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_autoref_bridge::AutorefBridgeNode)
