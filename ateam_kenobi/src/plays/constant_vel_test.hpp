// Copyright 2023 A Team
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

#ifndef PLAYS__CONSTANT_VEL_TEST_HPP_
#define PLAYS__CONSTANT_VEL_TEST_HPP_

#include "base_play.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace ateam_kenobi::plays
{
class ConstantVelTest : public BasePlay
{
public:
  ConstantVelTest(visualization::OverlayPublisher & overlay_publisher, visualization::PlayInfoPublisher & play_info_publisher);

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(const World & world) override;

private:
  void joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult onSetParametersCallback(const std::vector<rclcpp::Parameter> &);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_handle;

  bool motion_enabled = false;

  double vel_x = 0.0;
  double vel_y = 0.0;
  double vel_t = 0.0;
};
}

#endif  // PLAYS__CONSTANT_VEL_TEST_HPP_
