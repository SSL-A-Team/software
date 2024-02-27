#include <ateam_data_conversion/ssl_log_reader.hpp>
#include <ateam_common/topic_names.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <rclcpp_components/register_node_macro.hpp>

namespace ateam_data_conversion
{

template<typename ... Ts>
struct overloaded : Ts ... {
  using Ts::operator() ...;
};
// explicit deduction guide (not needed as of C++20)

class SSLLogReplayerNode : public rclcpp::Node
{
public:
  explicit SSLLogReplayerNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ssl_log_replayer", options)
  {

    vision_publisher_ = create_publisher<ssl_league_msgs::msg::VisionWrapper>(
      std::string(Topics::kVisionMessages),
      rclcpp::SystemDefaultsQoS());

    referee_publisher_ = create_publisher<ssl_league_msgs::msg::Referee>(
      std::string(Topics::kRefereeMessages),
      rclcpp::SystemDefaultsQoS());

    clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>(
      std::string("/clock"),
      rclcpp::SystemDefaultsQoS());

    SSLLogReader log_reader(declare_parameter<std::string>("ssl_log_path", ""));
    while (log_reader.has_next()) {
      auto maybe_msg = log_reader.next();
      if (maybe_msg.has_value()) {
        // overload never works as soon as you get any sort of complicated arg
        std::visit(overloaded{
          [this](ssl_league_msgs::msg::VisionWrapper arg) {
            vision_publisher_->publish(arg);
            rosgraph_msgs::msg::Clock clock_msg;
            if (arg.detection.size() > 0) {
              clock_msg.clock = arg.detection.at(0).t_sent;
              clock_publisher_->publish(clock_msg);
            }
          },
          [this](ssl_league_msgs::msg::Referee arg) {
            referee_publisher_->publish(arg);
            rosgraph_msgs::msg::Clock clock_msg;
            clock_msg.clock = arg.timestamp;
            clock_publisher_->publish(clock_msg);
          },
        }, maybe_msg.value());

      }
    }
  }

private:
  rclcpp::Publisher<ssl_league_msgs::msg::VisionWrapper>::SharedPtr vision_publisher_;
  rclcpp::Publisher<ssl_league_msgs::msg::Referee>::SharedPtr referee_publisher_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
};

} // namespace ateam_data_conversion

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_data_conversion::SSLLogReplayerNode)