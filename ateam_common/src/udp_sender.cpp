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

#include "ateam_common/udp_sender.hpp"

#include <boost/bind.hpp>

#include <algorithm>
#include <iostream>
#include <string>

namespace ateam_common
{

UDPSender::UDPSender(
  const std::string & udp_address_string,
  const int16_t udp_port)
: udp_socket_(io_service_),
  receiver_endpoint_(
    boost::asio::ip::make_address(udp_address_string),
    udp_port)
{
  udp_socket_.open(receiver_endpoint_.protocol());

  io_service_thread_ = std::thread(
    [this]() {
      io_service_.run();
    });
}

UDPSender::~UDPSender()
{
  io_service_.stop();
  if (io_service_thread_.joinable()) {
    io_service_thread_.join();
  }
}

void UDPSender::send(const char * const data, const size_t length)
{
  if (length >= buffer_.size()) {
    // RCLCPP_ERROR(get_logger(), "UDP send data length is larger than buffer");
    std::cout << "WARNING: UDP send data length is larger than buffer" << std::endl;

    return;
  }

  // Copy to buffer
  // Better to send an invalid packet than to overrun the buffer
  // With the if statement above, this should never happen
  memcpy(buffer_.data(), data, std::min(buffer_.size(), length));

  udp_socket_.async_send_to(
    boost::asio::buffer(buffer_, length),
    receiver_endpoint_,
    boost::bind(
      &UDPSender::HandleUDPSendTo, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void UDPSender::HandleUDPSendTo(
  const boost::system::error_code & error,
  std::size_t /** bytes_transferred **/)
{
  if (error) {
    // RCLCPP_ERROR(get_logger(), "Error during udp send");
    std::cout << "WARNING: Error during udp send" << std::endl;
  }
}

}  // namespace ateam_common
