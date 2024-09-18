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

#include "firmware_parameter_server.hpp"
#include "rnp_packet_helpers.hpp"

using namespace std::string_literals;

namespace ateam_radio_bridge
{

FirmwareParameterServer::FirmwareParameterServer(rclcpp::Node & node, std::array<std::unique_ptr<ateam_common::BiDirectionalUDP>, 16>& connections)
  : logger_(node.get_logger()),
    connections_(connections)
{
  get_param_service_ = node.create_service<ateam_msgs::srv::GetFirmwareParameter>("get_firmware_param", std::bind(&FirmwareParameterServer::GetFirmwareParameterCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_param_service_ = node.create_service<ateam_msgs::srv::SetFirmwareParameter>("set_firmware_param", std::bind(&FirmwareParameterServer::SetFirmwareParameterCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void FirmwareParameterServer::HandleIncomingParameterPacket(const int robot_id, const ParameterCommand & packet) 
{
  std::unique_lock<std::mutex> lock{response_mutex_};
  if(robot_id != active_robot_id_)
  {
    RCLCPP_WARN(logger_, "Ignoring parameter command response from wrong robot. Expected %d but %d replied.", active_robot_id_, robot_id);
    return;
  }
  response_ = packet;
  response_ready_ = true;
  lock.unlock();
  response_cv_.notify_one();
}

void FirmwareParameterServer::GetFirmwareParameterCallback(const ateam_msgs::srv::GetFirmwareParameter::Request::SharedPtr request, ateam_msgs::srv::GetFirmwareParameter::Response::SharedPtr response)
{
  try {
    const auto robot_id = request->robot_id;
    if(robot_id < 0 || robot_id > 15) {
      response->success = false;
      response->reason = "Invalid robot ID";
      return;
    }
    if(!connections_[robot_id]) {
      response->success = false;
      response->reason = "Selected robot is not connected.";
      return;
    }
    ParameterCommand command;
    command.command_code = PCC_READ;
    command.parameter_name = static_cast<ParameterName>(request->parameter_id);
    command.data_format = GetParameterDataFormatForParameter(command.parameter_name);
    const auto command_packet = CreatePacket(CC_ROBOT_PARAMETER_COMMAND, command);
    std::unique_lock<std::mutex> lock{response_mutex_};
    active_robot_id_ = robot_id;
    response_ready_ = false;
    connections_[robot_id]->send(
      reinterpret_cast<const uint8_t *>(&command_packet),
      GetPacketSize(command_packet.command_code));
    if(!response_cv_.wait_for(lock, std::chrono::milliseconds(300), [this]{ return response_ready_.load(); })) {
      response->success = false;
      response->reason = "Timed out waiting for reply.";
      return;
    }
    if(!CheckParameterPacketAck(response_, response->reason)) {
      return;
    }
    RCLCPP_WARN_EXPRESSION(logger_, response_.parameter_name != command.parameter_name, "Got a parameter ACK for a different parameter than was asked for. Expected %d but got %d", command.parameter_name, response_.parameter_name);
    const float * packet_data = GetParameterDataForSetFormat(response_);
    std::copy_n(packet_data, GetDataSizeForParameterFormat(response_.data_format), std::back_inserter(response->data));
    response->success = true;
  } catch (std::exception & e) {
    response->success = false;
    response->reason = "Exception thrown: "s + e.what();
    return;
  }
}

void FirmwareParameterServer::SetFirmwareParameterCallback(const ateam_msgs::srv::SetFirmwareParameter::Request::SharedPtr request, ateam_msgs::srv::SetFirmwareParameter::Response::SharedPtr response)
{
  try {
    const auto robot_id = request->robot_id;
    if(robot_id < 0 || robot_id > 15) {
      response->success = false;
      response->reason = "Invalid robot ID";
      return;
    }
    if(!connections_[robot_id]) {
      response->success = false;
      response->reason = "Selected robot is not connected.";
      return;
    }
    ParameterCommand command;
    command.command_code = PCC_WRITE;
    command.parameter_name = static_cast<ParameterName>(request->parameter_id);
    command.data_format = GetParameterDataFormatForParameter(command.parameter_name);
    const auto data_size = GetDataSizeForParameterFormat(command.data_format);
    if(request->data.size() != data_size) {
      response->success = false;
      response->reason = "Wrong data size. Expected "s + std::to_string(data_size) + " elements but got " + std::to_string(request->data.size()) + ".";
      return;
    }
    float * data_slot = GetParameterDataForSetFormat(command);
    std::copy_n(request->data.begin(), data_size, data_slot);
    const auto command_packet = CreatePacket(CC_ROBOT_PARAMETER_COMMAND, command);
    std::unique_lock<std::mutex> lock{response_mutex_};
    active_robot_id_ = robot_id;
    response_ready_ = false;
    connections_[robot_id]->send(
      reinterpret_cast<const uint8_t *>(&command_packet),
      GetPacketSize(command_packet.command_code));
    if(!response_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]{ return response_ready_.load(); })) {
      response->success = false;
      response->reason = "Timed out waiting for reply.";
      return;
    }
    if(!CheckParameterPacketAck(response_, response->reason)) {
      return;
    }
    RCLCPP_WARN_EXPRESSION(logger_, response_.parameter_name != command.parameter_name, "Got a parameter ACK for a different parameter than was asked for. Expected %d but got %d", command.parameter_name, response_.parameter_name);
    if(response_.data_format != command.data_format) {
      response->success = false;
      response->reason = "Ack'ed data format does not equal sent data format.";
      return;
    }
    const float * response_data_slot = GetParameterDataForSetFormat(response_);
    if(!std::equal(request->data.begin(), request->data.end(), response_data_slot)) {
      response->success = false;
      response->reason = "Ack'ed paramter value does not equal sent data.";
      return;
    }
    response->success = true;
  } catch (std::exception & e) {
    response->success = false;
    response->reason = "Exception thrown: "s + e.what();
    return;
  }
}

}  // namespace ateam_radio_bridge
