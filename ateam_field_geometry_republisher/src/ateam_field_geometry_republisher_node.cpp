#include <chrono>
#include <functional>
#include <mutex>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ssl_league_msgs/msg/vision_geometry_data.hpp>
#include <ssl_league_msgs/msg/vision_geometry_field_size.hpp>
#include <ssl_league_msgs/msg/vision_wrapper.hpp>

#include <ateam_common/topic_names.hpp>

using namespace std::chrono_literals;

namespace ateam_field_geometry_republisher
{

class FieldGeometryRepublisherNode : public rclcpp::Node
{
public:
  explicit FieldGeometryRepublisherNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ateam_field_geometry_republisher", options)
  {
    timer_ = create_wall_timer(
      1000ms, std::bind(
        &FieldGeometryRepublisherNode::timer_callback,
        this));

    field_publisher_ = create_publisher<ssl_league_msgs::msg::VisionGeometryFieldSize>(
      std::string(Topics::kField),
      rclcpp::SystemDefaultsQoS());

    ssl_vision_subscription_ =
      create_subscription<ssl_league_msgs::msg::VisionWrapper>(
      std::string(Topics::kVisionMessages),
      10,
      std::bind(&FieldGeometryRepublisherNode::message_callback, this, std::placeholders::_1));
  }

  // Wrapper comes with both robot state and field dimensions this node just splits out the field dims so the vision filter doesnt need to and other nodes have a common subscription for the field
  void message_callback(
    const ssl_league_msgs::msg::VisionWrapper::SharedPtr vision_wrapper_msg)
  {
    ssl_league_msgs::msg::VisionGeometryFieldSize & field_report =
      vision_wrapper_msg->geometry.field;
    if (field_report.field_length < 1.0 || field_report.field_width < 1.0) {
      return;
    }

    const std::lock_guard<std::mutex> lock(field_mutex_);
    field_ = field_report;
  }

  void timer_callback()
  {
    const std::lock_guard<std::mutex> lock(field_mutex_);
    field_publisher_->publish(field_);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ssl_league_msgs::msg::VisionGeometryFieldSize>::SharedPtr field_publisher_;
  rclcpp::Subscription<ssl_league_msgs::msg::VisionWrapper>::SharedPtr ssl_vision_subscription_;

  std::mutex field_mutex_;
  ssl_league_msgs::msg::VisionGeometryFieldSize field_;
};
}  // namespace ateam_field_geometry_republisher
