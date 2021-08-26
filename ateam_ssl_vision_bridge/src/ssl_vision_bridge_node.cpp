#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <ateam_common/multicast_receiver.hpp>
#include <ssl_league_msgs/msg/vision.hpp>
#include <ssl_league_protobufs/ssl_vision_detection.pb.h>

#include "message_conversions.hpp"

namespace ateam_ssl_vision_bridge
{

class VisionBridgeNode : public rclcpp::Node
{
public:
  explicit VisionBridgeNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("vision_bridge", options),
    multicast_receiver_("224.5.23.2",
                       10006,
                       [this](auto* buffer, size_t bytes_received){
                         SSL_DetectionFrame vision_proto;
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
  ateam_common::MulticastReceiver multicast_receiver_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_ssl_vision_bridge::VisionBridgeNode)
