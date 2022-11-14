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

#ifndef ATEAM_COMMON__INDEXED_TOPIC_HELPERS_HPP_
#define ATEAM_COMMON__INDEXED_TOPIC_HELPERS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <functional>
#include <string>

namespace ateam_common::indexed_topic_helpers
{

const int kRobotCount = 16;

template<typename MessageType, typename NodeType>
void create_indexed_subscribers(
  std::array<typename rclcpp::Subscription<MessageType>::SharedPtr, kRobotCount> & destination,
  const std::string & topic_base,
  const rclcpp::QoS & qos,
  void (NodeType::* callback_pointer)(const typename MessageType::SharedPtr, const int),
  NodeType * node
)
{
  for (int robot_id = 0; robot_id < kRobotCount; ++robot_id) {
    std::function<void(const typename MessageType::SharedPtr)> callback =
      std::bind(callback_pointer, node, std::placeholders::_1, robot_id);
    destination.at(robot_id) = node->template create_subscription<MessageType>(
      topic_base + std::to_string(
        robot_id), qos, callback);
  }
}

template<typename MessageType, typename NodeType>
void create_indexed_publishers(
  std::array<typename rclcpp::Publisher<MessageType>::SharedPtr, kRobotCount> & destination,
  const std::string & topic_base,
  const rclcpp::QoS & qos,
  NodeType * node
)
{
  for (int robot_id = 0; robot_id < kRobotCount; ++robot_id) {
    destination.at(robot_id) = node->template create_publisher<MessageType>(
      topic_base + std::to_string(
        robot_id), qos);
  }
}

}  // namespace ateam_common::indexed_topic_helpers

#endif  // ATEAM_COMMON__INDEXED_TOPIC_HELPERS_HPP_
