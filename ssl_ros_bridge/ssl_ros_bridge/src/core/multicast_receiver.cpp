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

#include "multicast_receiver.hpp"


#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <algorithm>
#include <iostream>
#include <string>

#include <boost/bind/bind.hpp>
#include <boost/exception_ptr.hpp>

#include "get_ip_addresses.hpp"

namespace ssl_ros_bridge::core
{

MulticastReceiver::MulticastReceiver(
  std::string multicast_address_string,
  uint16_t multicast_port,
  ReceiveCallback receive_callback,
  std::string interface_address)
: receive_callback_(receive_callback),
  multicast_socket_(io_service_)
{
  const auto multicast_address = boost::asio::ip::make_address(multicast_address_string).to_v4();
  const boost::asio::ip::udp::endpoint multicast_endpoint(multicast_address, multicast_port);
  multicast_socket_.open(multicast_endpoint.protocol());
  multicast_socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
  multicast_socket_.bind(multicast_endpoint);
  if (interface_address.empty()) {
    // If no interface specified, join on all interfaces
    const auto available_interface_addresses = GetIpAdresses(false);
    for(const auto & address : available_interface_addresses) {
      try {
        multicast_socket_.set_option(
        boost::asio::ip::multicast::join_group(
          multicast_address,
          boost::asio::ip::make_address_v4(address)));
      } catch (const boost::system::system_error & e) {
        /* Ignore "address already in use" exceptions. This just indicates multiple addresses
         * assigned to the same interface.
         */
        if(e.code().value() != EADDRINUSE) {
          boost::rethrow_exception(boost::current_exception());
        }
      }
    }
  } else {
    multicast_socket_.set_option(
      boost::asio::ip::multicast::join_group(
        multicast_address,
        boost::asio::ip::make_address_v4(interface_address)));
  }
  multicast_socket_.async_receive_from(
    boost::asio::buffer(buffer_), sender_endpoint_,
    boost::bind(
      &MulticastReceiver::HandleMulticastReceiveFrom, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

  io_service_thread_ = std::thread(
    [this]() {
      io_service_.run();
    });
}

MulticastReceiver::~MulticastReceiver()
{
  io_service_.stop();
  if (io_service_thread_.joinable()) {
    io_service_thread_.join();
  }
}

void MulticastReceiver::SendTo(
  const std::string & address, const uint16_t port,
  const char * const data, const size_t length)
{
  if (length >= send_buffer_.size()) {
    std::cout << "WARNING: UDP send data length is larger than buffer" << std::endl;

    return;
  }

  // Copy to buffer
  // Better to send an invalid packet than to overrun the buffer
  // With the if statement above, this should never happen
  memcpy(send_buffer_.data(), data, std::min(send_buffer_.size(), length));

  boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(address), port);

  multicast_socket_.async_send_to(
    boost::asio::buffer(send_buffer_, length),
    endpoint,
    boost::bind(
      &MulticastReceiver::HandleUDPSendTo, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void MulticastReceiver::HandleMulticastReceiveFrom(
  const boost::system::error_code & error,
  size_t bytes_received)
{
  if (error) {
    std::cerr << "Receive from error: " << error.message() << std::endl;
    return;
  }

  receive_callback_(
    sender_endpoint_.address().to_string(), sender_endpoint_.port(),
    buffer_.data(), bytes_received);

  multicast_socket_.async_receive_from(
    boost::asio::buffer(buffer_), sender_endpoint_,
    boost::bind(
      &MulticastReceiver::HandleMulticastReceiveFrom, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}


void MulticastReceiver::HandleUDPSendTo(const boost::system::error_code & error, size_t)
{
  if (error) {
    std::cerr << "Error sending UDP data: " << error.message() << std::endl;
  }
}

}  // namespace ssl_ros_bridge::core
