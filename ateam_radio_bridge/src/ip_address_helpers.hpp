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


#ifndef GET_IP_ADDRESSES_HPP__
#define GET_IP_ADDRESSES_HPP__

#include <vector>
#include <string>

/**
 * @brief Get all IP addresses for the local machine
 *
 * @return std::vector<std::string> IP addresses in dot decimal notation
 */
std::vector<std::string> GetIpAddresses();

/**
 * @brief Get the IP address from @c addresses with the largest common prefix with @c target .
 *
 * The intent is to get the address that shares the most specific subnet with the target address.
 *
 * @param addresses IP addresses to search, in dot decimal notation
 * @param target IP address to match against, in dot decimal notation
 * @return std::string& reference to closest address
 */
const std::string & GetClosestIpAddress(
  const std::vector<std::string> & addresses,
  const std::string & target);

#endif  // GET_IP_ADDRESSES_HPP__
