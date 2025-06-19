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


#ifndef FIRMWARE_PARAMETER_SERVER_HPP_
#define FIRMWARE_PARAMETER_SERVER_HPP_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <ateam_radio_msgs/packets/robot_parameters.h>
#include <ateam_msgs/srv/set_firmware_parameter.hpp>
#include <ateam_msgs/srv/get_firmware_parameter.hpp>
#include <ateam_common/bi_directional_udp.hpp>

namespace ateam_radio_bridge
{

class FirmwareParameterServer
{
public:
  explicit FirmwareParameterServer(rclcpp::Node & node, std::array<std::unique_ptr<ateam_common::BiDirectionalUDP>, 16>& connections);

  void HandleIncomingParameterPacket(const int robot_id, const ParameterCommand & packet);

private:
  rclcpp::Logger logger_;
  std::array<std::unique_ptr<ateam_common::BiDirectionalUDP>, 16>& connections_;
  ParameterCommand response_;
  int active_robot_id_;
  std::atomic_bool response_ready_ = false;
  std::mutex response_mutex_;
  std::condition_variable response_cv_;
  rclcpp::Service<ateam_msgs::srv::GetFirmwareParameter>::SharedPtr get_param_service_;
  rclcpp::Service<ateam_msgs::srv::SetFirmwareParameter>::SharedPtr set_param_service_;

  void GetFirmwareParameterCallback(const ateam_msgs::srv::GetFirmwareParameter::Request::SharedPtr request, ateam_msgs::srv::GetFirmwareParameter::Response::SharedPtr response);
  void SetFirmwareParameterCallback(const ateam_msgs::srv::SetFirmwareParameter::Request::SharedPtr request, ateam_msgs::srv::SetFirmwareParameter::Response::SharedPtr response);

};

}  // namespace ateam_radio_bridge

#endif  // FIRMWARE_PARAMETER_SERVER_HPP_