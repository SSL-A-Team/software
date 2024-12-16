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

#include "get_ip_addresses.hpp"

#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace ssl_ros_bridge::core
{

std::vector<std::string> GetIpAdresses(const bool include_ipv6)
{
  std::vector<std::string> addresses;
  ifaddrs * iface_info = nullptr;
  getifaddrs(&iface_info);
  auto iface_info_current = iface_info;
  while(iface_info_current != nullptr) {
    if(!iface_info_current->ifa_addr) {
      continue;
    }

    if(iface_info_current->ifa_addr->sa_family == AF_INET) {
            // IPv4 address
      void * net_address =
        &(reinterpret_cast<sockaddr_in *>(iface_info_current->ifa_addr)->sin_addr);
      char buffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, net_address, buffer, INET_ADDRSTRLEN);
      addresses.emplace_back(buffer);
    } else if(include_ipv6 && iface_info_current->ifa_addr->sa_family == AF_INET6) {
            // IPv6 address
      void * net_address =
        &(reinterpret_cast<sockaddr_in *>(iface_info_current->ifa_addr)->sin_addr);
      char buffer[INET6_ADDRSTRLEN];
      inet_ntop(AF_INET6, net_address, buffer, INET6_ADDRSTRLEN);
      addresses.emplace_back(buffer);
    }

    iface_info_current = iface_info_current->ifa_next;
  }
  if(iface_info != nullptr) {
    freeifaddrs(iface_info);
  }
  return addresses;
}

}  // namespace ssl_ros_bridge::core
