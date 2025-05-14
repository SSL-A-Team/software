// Copyright 2024 A Team
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

#include <chrono>
#include <fstream>
#include <functional>
#include <mutex>
#include <iostream>
#include <string>

#include <ateam_common/cache_directory.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_msgs/srv/set_ignore_field_side.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "message_conversions.hpp"

using namespace std::chrono_literals;

namespace ateam_field_manager
{

class FieldManagerNode : public rclcpp::Node
{
public:
  explicit FieldManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("ateam_field_manager", options),
    game_controller_listener_(*this)
  {
    loadCachedState();

    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();
    field_publisher_ = create_publisher<ateam_msgs::msg::FieldInfo>(
      std::string(Topics::kField),
      qos);

    ssl_vision_subscription_ =
      create_subscription<ssl_league_msgs::msg::VisionWrapper>(
      std::string(Topics::kVisionMessages),
      10,
      std::bind(&FieldManagerNode::vision_callback, this, std::placeholders::_1));

    set_ignore_field_side_service_ =
      create_service<ateam_msgs::srv::SetIgnoreFieldSide>("~/set_ignore_field_side",
        std::bind(&FieldManagerNode::handle_set_ignore_field_side, this, std::placeholders::_1,
        std::placeholders::_2));
  }

  ~FieldManagerNode()
  {
    saveCachedState();
  }

  void vision_callback(
    const ssl_league_msgs::msg::VisionWrapper::SharedPtr vision_wrapper_msg)
  {
    const auto team_side = game_controller_listener_.GetTeamSide();

    if (!vision_wrapper_msg->geometry.empty()) {
      field_publisher_->publish(
        message_conversions::fromMsg(
          vision_wrapper_msg->geometry.front(), team_side,
          ignore_side_));
    }
  }

  void handle_set_ignore_field_side(
    const ateam_msgs::srv::SetIgnoreFieldSide::Request::SharedPtr request,
    ateam_msgs::srv::SetIgnoreFieldSide::Response::SharedPtr response)
  {
    if (request->ignore_side == 0) {
      ignore_side_ = 0;
    } else if (request->ignore_side < 0) {
      ignore_side_ = -1;
    } else {
      ignore_side_ = 1;
    }

    response->success = true;
  }

private:
  const char * ignore_side_cache_filename_ = "ignore_side.txt";
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ateam_msgs::msg::FieldInfo>::SharedPtr field_publisher_;
  rclcpp::Subscription<ssl_league_msgs::msg::VisionWrapper>::SharedPtr ssl_vision_subscription_;
  rclcpp::Subscription<ssl_league_msgs::msg::VisionWrapper>::SharedPtr ssl_vision_subs_;
  rclcpp::Service<ateam_msgs::srv::SetIgnoreFieldSide>::SharedPtr set_ignore_field_side_service_;
  ateam_common::GameControllerListener game_controller_listener_;

  int ignore_side_ = 0;

  std::filesystem::path getCacheDirectory()
  {
    return ateam_common::getCacheDirectory() / "field_manager";
  }

  void loadCachedState()
  {
    const auto ignore_side_path = getCacheDirectory() / ignore_side_cache_filename_;
    if(std::filesystem::exists(ignore_side_path)) {
      RCLCPP_INFO(get_logger(), "Loading cache from %s", ignore_side_path.c_str());
      std::ifstream cache_file(ignore_side_path);
      cache_file >> ignore_side_;
    }
  }

  void saveCachedState()
  {
    const auto cache_dir = getCacheDirectory();
    const auto ignore_side_path = cache_dir / ignore_side_cache_filename_;
    RCLCPP_INFO(get_logger(), "Saving state to %s", ignore_side_path.c_str());
    std::filesystem::create_directories(cache_dir);
    std::ofstream cache_file(ignore_side_path);
    cache_file << ignore_side_;
  }
};
}  // namespace ateam_field_manager

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_field_manager::FieldManagerNode)
