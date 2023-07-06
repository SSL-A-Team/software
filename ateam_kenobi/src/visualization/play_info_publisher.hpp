#ifndef VISUALIZATION__PLAY_INFO_PUBLISHER_HPP_
#define VISUALIZATION__PLAY_INFO_PUBLISHER_HPP_

#include <rclcpp/node.hpp>
#include <ateam_msgs/msg/play_info.hpp>

namespace ateam_kenobi::visualization
{

class PlayInfoPublisher
{
public:
    explicit PlayInfoPublisher(rclcpp::Node & node);
    void send_play_message(const std::string & play_name, const std::string & message);
private:
    rclcpp::Publisher<ateam_msgs::msg::PlayInfo>::SharedPtr publisher_;
};
} // namespace ateam_kenobi::visualization

#endif // VISUALIZATION__PLAY_INFO_PUBLISHER_HPP_