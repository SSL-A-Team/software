#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/geometry_msgs/msg/quaternion.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/vision_state_robot.hpp>
#include <ateam_controls/ateam_controls.h>

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
        // if (!have_msg_) {
        //     // Don’t publish until the first message arrives
        //     return;
        // }

        // double x = latest_msg_.pose.position.x;
        // double y = latest_msg_.pose.position.y;
        // // Convert to tf2 quaternion
        // tf2::Quaternion q(
        //     latest_msg_.pose.orientation.x,
        //     latest_msg_.pose.orientation.y,
        //     latest_msg_.pose.orientation.z,
        //     latest_msg_.pose.orientation.w
        // );
        // // Convert to roll, pitch, yaw
        // double roll, pitch, yaw;
        // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // GlobalState_t current_state = {x, y, yaw, 0.0, 0.0, 0.0};
        // GlobalState_t target_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // // ateam_controls_add();
        // float dt = 0.1;  // Because instantaneous velocity change can't be achieved, lead the state by dt
        // BangBangTraj3D_t traj = ateam_controls_compute_optimal_bang_bang_traj_3d(current_state, target_state);
        // GlobalState_t next_step_state = ateam_controls_compute_bang_bang_traj_3d_state_at_t(traj, current_state, 0.0, dt);

        // ateam_msgs::msg::RobotMotionCommand msg;
        // msg.twist.linear.x = next_step_state.xd;
        // msg.twist.linear.y = next_step_state.yd;
        // msg.twist.angular.z = next_step_state.zd;

        ateam_msgs::msg::RobotMotionCommand msg;
        msg.pose.position.x = 1.0;
        msg.pose.position.y = 2.0;
        msg.pose.orientation = geometry_msgs::msg::Quaternion();
        // msg.twist.angular.z = 3.0;


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
