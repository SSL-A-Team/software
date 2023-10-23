#ifndef ATEAM_DATA_CONVERSION__SSL_TO_ROSBAG_HPP_
#define ATEAM_DATA_CONVERSION__SSL_TO_ROSBAG_HPP_

#include "ssl_log_replayer.hpp"
#include "ssl_vision_adapter.hpp"

#include <ofstream>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_common/topic_names.hpp>

namespace ateam_data_conversion
{

class SslToRosbag
{
public: // ctors
    explict SslToRosbag(std::string input_ssl_filename = "", std::string out_bag_filename = "./rosbag");
};

} // namespace
#endif // ATEAM_DATA_CONVERSION__SSL_TO_ROSBAG_HPP_