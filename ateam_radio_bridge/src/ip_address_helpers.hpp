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
