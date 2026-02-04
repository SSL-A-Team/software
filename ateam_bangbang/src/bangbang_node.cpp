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

        declare_parameter<float>("a_linear", 0.2f);
        declare_parameter<float>("a_angular", 0.0f);

        // Publisher
        pub_ = this->create_publisher<ateam_msgs::msg::RobotMotionCommand>("/robot_motion_commands/robot2", 10);

        this->get_parameter("a_linear", a_linear_); // m/s^2
        this->get_parameter("a_angular", a_angular_); // m/s^2
        RCLCPP_INFO(this->get_logger(), "BangBangNode: a_linear = %f", a_linear_);
        RCLCPP_INFO(this->get_logger(), "BangBangNode: a_angular = %f deg", a_angular_);
        a_angular_ = a_angular_ * M_PI / 180.0f; // convert to rad
        RCLCPP_INFO(this->get_logger(), "BangBangNode: a_angular = %f rad", a_angular_);
        w_ = 2.0f * M_PI / 3.0f; // rad/s

        // 100 Hz = 10 ms period
        period_ms_ = 10;
        t_ = 0.0f;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms_),
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
        msg.twist.linear.x = a_linear_ * cosf(w_ * t_);
        msg.twist.linear.y = 0.0;
        msg.twist.angular.z = a_angular_ * cosf(w_ * t_);

        pub_->publish(msg);

        t_ += (float)period_ms_ / 1000.0f;
    }

    // rclcpp::Subscription<ateam_msgs::msg::VisionStateRobot>::SharedPtr sub_;
    rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    float a_linear_;
    float a_angular_;
    float w_;
    int64_t period_ms_;
    float t_;

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
