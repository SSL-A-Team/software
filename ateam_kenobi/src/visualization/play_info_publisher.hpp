#ifndef VISUALIZATION__PLAY_INFO_PUBLISHER_HPP_
#define VISUALIZATION__PLAY_INFO_PUBLISHER_HPP_

#include <rclcpp/node.hpp>
#include <ateam_msgs/msg/play_info.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace ateam_kenobi::visualization
{

class PlayInfoPublisher
{
public:
    explicit PlayInfoPublisher(rclcpp::Node & node);
    void send_play_message(const std::string & play_name);

    json message; // json data for a play to fill out. Is sent and then cleared when publishing

private:
    rclcpp::Publisher<ateam_msgs::msg::PlayInfo>::SharedPtr publisher_;
};
} // namespace ateam_kenobi::visualization

#endif // VISUALIZATION__PLAY_INFO_PUBLISHER_HPP_