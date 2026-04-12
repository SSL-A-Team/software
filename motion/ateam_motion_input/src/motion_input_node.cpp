#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/vision_state_robot.hpp>
#include <ateam_controls/ateam_controls.h>
#include <nlohmann/json.hpp>
#include <cmath>
#include <fstream>

class MotionInputNode : public rclcpp::Node
{
public:
    MotionInputNode()
    : Node("motion_input_node")
    {
        declare_parameter<float>("amp", 0.2f);
        declare_parameter<std::string>("dimension", "x");  // x, y, xy, or theta
        declare_parameter<std::string>("fn_type", "oscillate");  // pulse, oscillate, step, bangbang_pose, bangbang_accel
        declare_parameter<float>("freq", 1.0f);  // hz (for oscillate)
        declare_parameter<float>("width", 0.5f);  // seconds (for pulse)
        declare_parameter<float>("duration", 0.0f);  // 0 = run forever
        declare_parameter<int>("robot_id", 2);
        declare_parameter<float>("startup_delay", 3.0f);
        declare_parameter<std::string>("param_json", "");  // path to robot_params.json

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
        RCLCPP_INFO(this->get_logger(), "MotionInputNode: amp = %f", amp_);
        RCLCPP_INFO(this->get_logger(), "MotionInputNode: dimension = %s", dimension_.c_str());
        RCLCPP_INFO(this->get_logger(), "MotionInputNode: fn_type = %s", fn_type_.c_str());
        if (dimension_ == "theta") {
            amp_ = amp_ * M_PI / 180.0f; // convert to rad
            RCLCPP_INFO(this->get_logger(), "MotionInputNode: amp (rad) = %f", amp_);
        }
        this->get_parameter("freq", freq_);
        RCLCPP_INFO(this->get_logger(), "MotionInputNode: freq = %f Hz", freq_);
        w_ = 2.0f * M_PI * freq_;
        RCLCPP_INFO(this->get_logger(), "MotionInputNode: w = %f rad/s", w_);

        // Compute bang-bang trajectory if selected
        if (fn_type_ == "bangbang_pose") {
            Vector6C_t init_state = {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
            Vector3C_t target_pose = {0.0f, 0.0f, 0.0f};
            if (dimension_ == "x") {
                target_pose.x = amp_;
            } else if (dimension_ == "y") {
                target_pose.y = amp_;
            } else if (dimension_ == "theta") {
                target_pose.z = amp_;
            }
            TrajectoryParams_t traj_params = load_traj_params();
            ateam_controls_traj_from_target_pose(init_state, target_pose, traj_params, &traj_);
            float traj_end_time = ateam_controls_traj_end_time(traj_);
            RCLCPP_INFO(this->get_logger(), "BangBang trajectory end time: %.3f s", traj_end_time);
        } else if (fn_type_ == "bangbang_accel") {
            RCLCPP_INFO(this->get_logger(), "BangBang accel: amp=%.3f, width=%.3f s", amp_, width_);
        }


        // 100 Hz = 10 ms period
        period_ms_ = 10;

        float startup_delay;
        get_parameter("startup_delay", startup_delay);

        RCLCPP_INFO(this->get_logger(), "Waiting %.1f s for subscribers...", startup_delay);
        t_ = -(float)period_ms_ / 1000.0f; // start slightly negative
        fn_started_ = false;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms_),
            std::bind(&MotionInputNode::publish_latest, this)
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

    TrajectoryParams_t load_traj_params()
    {
        TrajectoryParams_t params = ateam_controls_traj_params_default();
        std::string param_json;
        this->get_parameter("param_json", param_json);
        if (param_json.empty()) {
            RCLCPP_INFO(this->get_logger(), "No param_json provided, using default trajectory params");
            return params;
        }
        std::ifstream f(param_json);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open param_json: %s", param_json.c_str());
            return params;
        }
        nlohmann::json j = nlohmann::json::parse(f);
        auto get_vec = [&](const char* key, size_t idx, float& out) {
            if (j.contains(key) && j[key].is_array() && j[key].size() > idx) {
                out = j[key][idx].get<float>();
            }
        };
        // TRAJ_MAX [MAX_VEL_LINEAR, MAX_VEL_ANGULAR, MAX_ACCEL_LINEAR, MAX_ACCEL_ANGULAR]
        get_vec("TRAJ_MAX", 0, params.max_vel_linear);
        get_vec("TRAJ_MAX", 1, params.max_vel_angular);
        get_vec("TRAJ_MAX", 2, params.max_accel_linear);
        get_vec("TRAJ_MAX", 3, params.max_accel_angular);
        RCLCPP_INFO(this->get_logger(), "Loaded trajectory params from %s", param_json.c_str());
        return params;
    }

    void publish_latest()
    {
        if (fn_started_) {
            ateam_msgs::msg::RobotMotionCommand msg;

            msg.twist.linear.x = 0.0;
            msg.twist.linear.y = 0.0;
            msg.twist.angular.z = 0.0;

            t_ += (float)period_ms_ / 1000.0f;

            // Compute control value based on fn_type
            float val = 0.0f;
            if (fn_type_ == "bangbang_pose") {
                // Use planned position from bang-bang trajectory
                Vector6C_t init_state = {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
                Vector6C_t state;
                ateam_controls_traj_state_at(traj_, init_state, 0.0f, t_, &state);
                if (dimension_ == "x") {
                    val = state.data[0];
                } else if (dimension_ == "y") {
                    val = state.data[1];
                } else if (dimension_ == "theta") {
                    val = state.data[2];
                }
            } else if (fn_type_ == "bangbang_accel") {
                // Simple bang-bang: +amp for width seconds, then -amp for width seconds
                if (t_ < width_) {
                    val = amp_;
                } else if (t_ < 2.0f * width_) {
                    val = -amp_;
                } else {
                    val = 0.0f;
                }
            } else if (fn_type_ == "oscillate") {
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
            } else if (dimension_ == "xy") {
                msg.twist.linear.x = val;
                msg.twist.linear.y = val;
            } else if (dimension_ == "theta") {
                msg.twist.angular.z = val;
            }

            pub_->publish(msg);
        }

        // Auto-shutdown after duration (if set)
        if (duration_ > 0.0f && t_ >= duration_) {
            RCLCPP_INFO(this->get_logger(), "MotionInputNode: Duration %.2f s reached, shutting down.", duration_);
            rclcpp::shutdown();
        }
    }

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
    BangBangTraj3D_t traj_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionInputNode>());
    rclcpp::shutdown();
    return 0;
}
