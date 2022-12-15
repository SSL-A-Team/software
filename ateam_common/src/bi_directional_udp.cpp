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

#include "ateam_common/bi_directional_udp.hpp"

#include <boost/bind/bind.hpp>

#include <algorithm>
#include <iostream>
#include <string>

namespace ateam_common
{

BiDirectionalUDP::BiDirectionalUDP(
  const std::string & udp_address_string,
  const int16_t udp_port,
  ReceiveCallback receive_callback)
: receive_callback_(receive_callback),
  udp_socket_(io_service_),
  endpoint_(
    boost::asio::ip::make_address(udp_address_string),
    udp_port)
{
  udp_socket_.open(endpoint_.protocol());

  udp_socket_.async_receive_from(
    boost::asio::buffer(receive_buffer_, receive_buffer_.size()),
    endpoint_,
    boost::bind(
      &BiDirectionalUDP::HandleUDPReceiveFrom, this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));

  io_service_thread_ = std::thread(
    [this]() {
      io_service_.run();
    });
}

BiDirectionalUDP::~BiDirectionalUDP()
{
  io_service_.stop();
  if (io_service_thread_.joinable()) {
    io_service_thread_.join();
  }
}

void BiDirectionalUDP::send(const char * const data, const size_t length)
{
  if (length >= send_buffer_.size()) {
    // RCLCPP_ERROR(get_logger(), "UDP send data length is larger than buffer");
    std::cout << "WARNING: UDP send data length is larger than buffer" << std::endl;

    return;
  }

  // Copy to buffer
  // Better to send an invalid packet than to overrun the buffer
  // With the if statement above, this should never happen
  memcpy(send_buffer_.data(), data, std::min(send_buffer_.size(), length));

  udp_socket_.async_send_to(
    boost::asio::buffer(send_buffer_, length),
    endpoint_,
    boost::bind(
      &BiDirectionalUDP::HandleUDPSendTo, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void BiDirectionalUDP::HandleUDPSendTo(
  const boost::system::error_code & error,
  std::size_t /** bytes_transferred **/)
{
  if (error) {
    // RCLCPP_ERROR(get_logger(), "Error during udp send");
    std::cout << "WARNING: Error during udp send" << std::endl;
  }
}

void BiDirectionalUDP::HandleUDPReceiveFrom(
  const boost::system::error_code & error,
  std::size_t bytes_transferred)
{
  if (!error) {
    receive_callback_(receive_buffer_.data(), bytes_transferred);

    udp_socket_.async_receive_from(
      boost::asio::buffer(receive_buffer_, receive_buffer_.size()),
      endpoint_,
      boost::bind(
        &BiDirectionalUDP::HandleUDPReceiveFrom, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  } else {
    // RCLCPP_ERROR(get_logger(), "Error during udp receive");
    std::cout << "WARNING: Error during udp receive" << std::endl;
  }
}


}  // namespace ateam_common
