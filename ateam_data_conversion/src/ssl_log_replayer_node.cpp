#include <ateam_data_conversion/ssl_log_reader.hpp>
#include <ateam_common/topic_names.hpp>

namespace ateam_data_conversion
{
class SSLLogReplayerNode : public rclcpp::Node
{

template<typename ... Ts>
struct Overload : Ts ... {
  using Ts::operator() ...;
};

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

    SSLLogReader log_reader(declare_parameter<std::string>("ssl_log_path", ""));
    while (!log_reader.has_next()) {
      auto msg = log_reader.next();
      std::visit(Overload{
        [this](ssl_league_msgs::msg::VisionWrapper arg) { vision_publisher_->publish(arg); },
        [this](ssl_league_msgs::msg::Referee arg) { referee_publisher_->publish(arg); },
      }, msg);
    }
  }

private:
  rclcpp::Publisher<ssl_league_msgs::msg::VisionWrapper>::SharedPtr vision_publisher_;
  rclcpp::Publisher<ssl_league_msgs::msg::Referee>::SharedPtr referee_publisher_;
};

} // namespace ateam_data_conversion

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_data_conversion::SSLLogReplayerNode)