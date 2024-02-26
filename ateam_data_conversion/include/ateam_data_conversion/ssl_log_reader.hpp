#ifndef ATEAM_DATA_CONVERSION__SSL_LOG_READER_HPP_
#define ATEAM_DATA_CONVERSION__SSL_LOG_READER_HPP_

#include <variant>
#include <optional>
#include <functional>
#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <ssl_league_msgs/msg/referee.hpp>
#include <ssl_league_msgs/msg/vision_wrapper.hpp>
#include <ateam_ssl_vision_bridge/message_conversions.hpp>
#include <ateam_game_controller_bridge/message_conversions.hpp>

namespace ateam_data_conversion
{
using MessageVariant_t = std::variant<ssl_league_msgs::msg::VisionWrapper, ssl_league_msgs::msg::Referee>;

enum MessageType
{
    MESSAGE_BLANK = 0,
    MESSAGE_UNKNOWN = 1,
    MESSAGE_SSL_VISION_2010 = 2,
    MESSAGE_SSL_REFBOX_2013 = 3,
    MESSAGE_SSL_VISION_2013 = 4,
    MESSAGE_SSL_VISION_2014 = 5

};

// really nothing here is needed when we go straight to ros
struct FileHeader
{
    std::string name; // SSL_LOG_FILE
    int32_t version; // Default file format is version 1
};

// ok example definetly used int64_t but like wut.... why
struct DataHeader
{
    uint64_t timestamp_ns; // Timestamp in ns
    int32_t type; // Message type
    int32_t message_size; // Size of protobuf message in bytes
};

// struct LogMessage
// {
//     DataHeader header;
//     MessageVariant_t msg;
// };

class SSLLogReader
{
public: // ctors
    explicit SSLLogReader(std::string filename);
    // fstream raii no need to close my brain is still stuck in c land

public: // funcs
    bool has_next();
    std::optional<MessageVariant_t> next();

private: // vars
    FileHeader file_header_;
    std::ifstream in_{};
};

} // namespace ateam_data_conversion

#endif // ATEAM_DATA_CONVERSION__SSL_LOG_READER_HPP_