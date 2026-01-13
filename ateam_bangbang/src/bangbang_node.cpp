#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/geometry_msgs/msg/quaternion.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/vision_state_robot.hpp>

class BangBangNode : public rclcpp::Node
{
public:
    BangBangNode()
    : Node("bangbang_node")
    {
        // // Subscriber
        // sub_ = this->create_subscription<ateam_msgs::msg::VisionStateRobot>(
        //     "/yellow_team/robot2",
        //     10,
        //     std::bind(&BangBangNode::callback, this, std::placeholders::_1)
        // );

        // Publisher
        pub_ = this->create_publisher<ateam_msgs::msg::RobotMotionCommand>("/robot_motion_commands/robot2", 10);

        // 100 Hz = 10 ms period
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&BangBangNode::publish_latest, this)
        );

        RCLCPP_INFO(this->get_logger(), "Node started: publishing at 100 Hz");
    }

private:
    // void callback(const ateam_msgs::msg::VisionStateRobot::SharedPtr msg)
    // {
    //     latest_msg_ = *msg;  // store local copy
    //     have_msg_ = true;
    // }

    void publish_latest()
    {

        ateam_msgs::msg::RobotMotionCommand msg;
        msg.pose.position.x = 0.0;
        msg.pose.position.y = 0.0;
        msg.pose.orientation = geometry_msgs::msg::Quaternion();

        pub_->publish(msg);
    }

    // rclcpp::Subscription<ateam_msgs::msg::VisionStateRobot>::SharedPtr sub_;
    rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ateam_msgs::msg::VisionStateRobot latest_msg_;  
    // bool have_msg_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BangBangNode>());
    rclcpp::shutdown();
    return 0;
}
