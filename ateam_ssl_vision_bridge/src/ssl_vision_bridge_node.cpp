#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <ateam_common/multicast_receiver.hpp>
#include <ssl_league_msgs/msg/vision.hpp>

#include "message_conversions.hpp"

namespace ateam_autoref_bridge
{

class AutorefBridgeNode : public rclcpp::Node
{
public:
  explicit AutorefBridgeNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("autoref_bridge", options),
    multicast_rcevier_("224.5.23.2",
                       10006,
                       [this](auto* buffer, size_t bytes_received){
                         Vision vision_proto;
                         if (!vision_proto.ParseFromArray(buffer, bytes_received))
                           return false;

                         vision_publisher_->publish(message_conversions::fromProto(vision_proto));
                         return true;
                       })
  {
    vision_publisher_ = create_publisher<ssl_league_msgs::msg::Vision>("~/vision_messages", rclcpp::SystemDefaultsQoS());
  }

private:
  rclcpp::Publisher<ssl_league_msgs::msg::Vision>::SharedPtr vision_publisher_;
  MulticastReceiver multicast_rcevier_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_autoref_bridge::AutorefBridgeNode)
