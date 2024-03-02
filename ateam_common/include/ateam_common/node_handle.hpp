// The literal worst piece of code ever but without some form of locator service
// which is probably what I should do for this setup in the future its incredibly annoying to
// get a node handle in anything but the top level
// https://gameprogrammingpatterns.com/service-locator.html

#ifndef ATEAM_COMMON__NODE_HANDLE_HPP_
#define ATEAM_COMMON__NODE_HANDLE_HPP_


#include <rclcpp/rclcpp.hpp>

// could make a class here who other nodes can extend from and will do setup for service locator for you
namespace ateam_common::node_handle
{
    extern rclcpp::Node::SharedPtr node_handle;

    static rclcpp::Time now() {
        return node_handle ? node_handle->get_clock()->now() : rclcpp::Time(0);
    }
}

#endif  // ATEAM_COMMON__NODE_HANDLE_HPP_

