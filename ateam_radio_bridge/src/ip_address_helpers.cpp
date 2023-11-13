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


#include "ip_address_helpers.hpp"
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <algorithm>

std::vector<std::string> GetIpAddresses()
{
  std::vector<std::string> addresses;
  ifaddrs * iface_info = nullptr;
  getifaddrs(&iface_info);
  auto iface_info_current = iface_info;
  while (iface_info_current != nullptr) {
    if (!iface_info_current->ifa_addr) {
      continue;
    }

    if (iface_info_current->ifa_addr->sa_family == AF_INET) {
      // IPv4 address
      void * net_address =
        &(reinterpret_cast<sockaddr_in *>(iface_info_current->ifa_addr)->sin_addr);
      char buffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, net_address, buffer, INET_ADDRSTRLEN);
      addresses.emplace_back(buffer);
    } else if (iface_info_current->ifa_addr->sa_family == AF_INET6) {
      // IPv6 address
      void * net_address =
        &(reinterpret_cast<sockaddr_in *>(iface_info_current->ifa_addr)->sin_addr);
      char buffer[INET6_ADDRSTRLEN];
      inet_ntop(AF_INET6, net_address, buffer, INET6_ADDRSTRLEN);
      addresses.emplace_back(buffer);
    }

    iface_info_current = iface_info_current->ifa_next;
  }
  if (iface_info != nullptr) {
    freeifaddrs(iface_info);
  }
  return addresses;
}

const std::string & GetClosestIpAddress(
  const std::vector<std::string> & addresses,
  const std::string & target)
{
  std::vector<std::size_t> address_overlaps;
  std::transform(
    addresses.begin(), addresses.end(), std::back_inserter(address_overlaps),
    [&target](const auto & address) {
      return std::distance(
        target.begin(),
        std::mismatch(target.begin(), target.end(), address.begin(), address.end()).first);
    });
  return addresses.at(
    std::distance(
      address_overlaps.begin(),
      std::max_element(address_overlaps.begin(), address_overlaps.end())));
}
