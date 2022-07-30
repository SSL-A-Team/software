#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/robot_feedback.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>

#include "stspin.h"

namespace ateam_radio_bridge
{

class RadioBridgeNode : public rclcpp::Node
{
public:
    RadioBridgeNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("radio_bridge", options)
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
    }


private:
    std::array<rclcpp::Subscription<ateam_msgs::msg::RobotMotionCommand>::SharedPtr, 16> motion_command_subscriptions_;
    std::array<rclcpp::Publisher<ateam_msgs::msg::RobotFeedback>::SharedPtr, 16> feedback_publishers_;

    void motion_command_callback(const ateam_msgs::msg::RobotMotionCommand::SharedPtr command_msg, int robot_id)
    {
        MotorCommandPacket radio_packet;
    }


};

}  // namespace ateam_radio_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_radio_bridge::RadioBridgeNode)
