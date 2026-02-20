#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/geometry_msgs/msg/quaternion.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/vision_state_robot.hpp>
#include <cmath>

class BangBangNode : public rclcpp::Node
{
public:
    BangBangNode()
    : Node("bangbang_node")
    {
        declare_parameter<float>("amp", 0.2f);
        declare_parameter<std::string>("dimension", "x");  // x, y, or theta
        declare_parameter<std::string>("fn_type", "oscillate");  // pulse, oscillate, step
        declare_parameter<float>("freq", 1.0f);  // hz (for oscillate)
        declare_parameter<float>("width", 0.5f);  // seconds (for pulse)
        declare_parameter<float>("duration", 0.0f);  // 0 = run forever
        declare_parameter<int>("robot_id", 2);
        declare_parameter<float>("startup_delay", 3.0f);

        int robot_id;
        this->get_parameter("robot_id", robot_id);

        // Publisher
        std::string topic = "/robot_motion_commands/robot" + std::to_string(robot_id);
        pub_ = this->create_publisher<ateam_msgs::msg::RobotMotionCommand>(topic, 10);

        this->get_parameter("amp", amp_);
        this->get_parameter("dimension", dimension_);
        this->get_parameter("fn_type", fn_type_);
        this->get_parameter("duration", duration_);
        this->get_parameter("width", width_);
        RCLCPP_INFO(this->get_logger(), "BangBangNode: amp = %f", amp_);
        RCLCPP_INFO(this->get_logger(), "BangBangNode: dimension = %s", dimension_.c_str());
        RCLCPP_INFO(this->get_logger(), "BangBangNode: fn_type = %s", fn_type_.c_str());
        if (dimension_ == "theta") {
            amp_ = amp_ * M_PI / 180.0f; // convert to rad
            RCLCPP_INFO(this->get_logger(), "BangBangNode: amp (rad) = %f", amp_);
        }
        this->get_parameter("freq", freq_);
        RCLCPP_INFO(this->get_logger(), "BangBangNode: freq = %f Hz", freq_);
        w_ = 2.0f * M_PI * freq_;
        RCLCPP_INFO(this->get_logger(), "BangBangNode: w = %f rad/s", w_);


        // 100 Hz = 10 ms period
        period_ms_ = 10;

        float startup_delay;
        get_parameter("startup_delay", startup_delay);

        RCLCPP_INFO(this->get_logger(), "Waiting %.1f s for subscribers...", startup_delay);
        t_ = -(float)period_ms_ / 1000.0f; // start slightly negative
        fn_started_ = false;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms_),
            std::bind(&BangBangNode::publish_latest, this)
        );
        startup_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(startup_delay * 1000)),
            [this]() {
                startup_timer_->cancel();  // fire only once
                RCLCPP_INFO(this->get_logger(), "Starting control function now...");
                fn_started_ = true;
            }
        );
    }

private:

    void publish_latest()
    {
        float fn_period = 10.0f;  // seconds
        if (fn_started_) {
            ateam_msgs::msg::RobotMotionCommand msg;

            msg.twist.linear.x = 0.0;
            msg.twist.linear.y = 0.0;
            msg.twist.angular.z = 0.0;

            t_ += (float)period_ms_ / 1000.0f;

            // Compute control value based on fn_type
            float val = 0.0f;
            if (fn_type_ == "oscillate") {
                val = 0.5 * (amp_ - amp_ * cosf(w_ * t_));  // not really amplitute, but just go from 0 to amp and back in a cosine wave
            } else if (fn_type_ == "step") {
                val = amp_;
            } else if (fn_type_ == "pulse") {
                val = (fmodf(t_, 1.0f/freq_) <= width_) ? amp_ : 0.0f;
            }

            if (dimension_ == "x") {
                msg.twist.linear.x = val;
            } else if (dimension_ == "y") {
                msg.twist.linear.y = val;
            } else if (dimension_ == "theta") {
                msg.twist.angular.z = val;
            }

            pub_->publish(msg);
        }

        // Auto-shutdown after duration (if set)
        if (duration_ > 0.0f && t_ >= duration_) {
            RCLCPP_INFO(this->get_logger(), "BangBangNode: Duration %.2f s reached, shutting down.", duration_);
            rclcpp::shutdown();
        }
    }

    // rclcpp::Subscription<ateam_msgs::msg::VisionStateRobot>::SharedPtr sub_;
    rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    float amp_;
    std::string dimension_;
    std::string fn_type_;
    float freq_;
    float w_;
    float width_;
    float duration_;
    int64_t period_ms_;
    float t_;
    bool fn_started_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BangBangNode>());
    rclcpp::shutdown();
    return 0;
}
