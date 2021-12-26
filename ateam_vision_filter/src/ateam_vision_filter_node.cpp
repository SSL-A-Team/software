// Copyright 2021 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <functional>
#include <mutex>

#include "world.hpp"
#include "message_conversions.hpp"

using namespace std::chrono_literals;

namespace ateam_vision_filter
{

class VisionFilterNode : public rclcpp::Node
{
public:
  explicit VisionFilterNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ateam_vision_filter", options)
  {
    timer_ = create_wall_timer(10ms, std::bind(&VisionFilterNode::timer_callback, this));
    ssl_vision_subscription_ =
      create_subscription<ssl_league_msgs::msg::VisionWrapper>(
      "~/vision_messages",
      10,
      std::bind(&VisionFilterNode::message_callback, this, std::placeholders::_1));
  }

  void message_callback(
    const ssl_league_msgs::msg::VisionWrapper::SharedPtr vision_wrapper_msg)
  {
    int camera_id = vision_wrapper_msg->detection.camera_id;
    CameraMeasurement camera_measurement = message_conversions::fromMsg(*vision_wrapper_msg);

    world_mutex_.lock();
    world_.update_camera(camera_id, camera_measurement);
  }

  void timer_callback()
  {
    world_mutex_.lock();
    world_.predict();
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<ssl_league_msgs::msg::VisionWrapper>::SharedPtr ball_publisher_;
  // std::array<rclcpp::Publisher<ssl_league_msgs::msg::VisionWrapper>
  //   ::SharedPtr, 16> blue_robots_publisher_;
  // std::array<rclcpp::Publisher<ssl_league_msgs::msg::VisionWrapper>
  //   ::SharedPtr, 16> yellow_robots_publisher_;
  rclcpp::Subscription<ssl_league_msgs::msg::VisionWrapper>::SharedPtr ssl_vision_subscription_;

  std::mutex world_mutex_;
  World world_;
};
}  // namespace ateam_vision_filter

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_vision_filter::VisionFilterNode)
