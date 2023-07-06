#include "ateam_common/get_ip_addresses.hpp"

#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace ateam_common
{

std::vector<std::string> GetIpV4Addresses()
{
    std::vector<std::string> addresses;
    ifaddrs * iface_info=nullptr;
    getifaddrs(&iface_info);
    auto iface_info_current = iface_info;
    while(iface_info_current != nullptr)
    {
        if(!iface_info_current->ifa_addr) {
            continue;
        }

        if(iface_info_current->ifa_addr->sa_family == AF_INET) {
            // IPv4 address
            void* net_address = &(reinterpret_cast<sockaddr_in*>(iface_info_current->ifa_addr)->sin_addr);
            char buffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, net_address, buffer, INET_ADDRSTRLEN);
            addresses.emplace_back(buffer);
        }

        iface_info_current = iface_info_current->ifa_next;
    }
    if(iface_info != nullptr) {
        freeifaddrs(iface_info);
    }
    return addresses;
}

}  // namespace ateam_common
