#include <array>
#include <functional>
#include <rclcpp/rclcpp.hpp>

namespace ateam_common::indexed_topic_helpers
{

const int kRobotCount = 16;

template<typename MessageType, typename NodeType>
void create_indexed_subscribers(
    std::array<typename rclcpp::Subscription<MessageType>::SharedPtr, kRobotCount>& destination,
    const std::string& topic_base,
    const rclcpp::QoS& qos,
    void(NodeType::* callback_pointer)(const typename MessageType::SharedPtr, const int),
    NodeType* node
)
{
    for(int robot_id = 0; robot_id < kRobotCount; ++robot_id) {
        std::function<void(const typename MessageType::SharedPtr)> callback = 
            std::bind(callback_pointer, node, std::placeholders::_1, robot_id);
        destination.at(robot_id) = node->template create_subscription<MessageType>(topic_base + std::to_string(robot_id), qos, callback);
    }
}

template<typename MessageType, typename NodeType>
void create_indexed_publishers(
    std::array<typename rclcpp::Publisher<MessageType>::SharedPtr, kRobotCount>& destination,
    const std::string& topic_base,
    const rclcpp::QoS& qos,
    NodeType* node
)
{
    for(int robot_id = 0; robot_id < kRobotCount; ++robot_id) {
        destination.at(robot_id) = node->template create_publisher<MessageType>(topic_base + std::to_string(robot_id), qos);
    }
}

}  // ateam_common
