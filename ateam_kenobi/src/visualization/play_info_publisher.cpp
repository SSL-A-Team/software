#include "play_info_publisher.hpp"

namespace ateam_kenobi::visualization
{
PlayInfoPublisher::PlayInfoPublisher(rclcpp::Node & node) : publisher_(node.create_publisher<ateam_msgs::msg::PlayInfo>(
      "/play_info",
      rclcpp::SystemDefaultsQoS()))
{  
}

void PlayInfoPublisher::send_play_message(
    const std::string & play_name,
    const std::string & info){
        ateam_msgs::msg::PlayInfo msg;
        msg.name = play_name;
        msg.description = info;
        publisher_->publish(msg);
    }
} // namespace ateam_kenobi::visualization