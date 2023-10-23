#ifndef ATEAM_DATA_CONVERSION__SSL_LOG_REPLAYER_HPP_
#define ATEAM_DATA_CONVERSION__SSL_LOG_REPLAYER_HPP_

#include <variant>
#include <optional>
#include <function>

#include <ssl_league_msgs/msg/vision_wrapper.hpp>
#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>

#include <ssl_league_msgs/msg/referee.hpp>
#include <ssl_league_protobufs/ssl_gc_referee_message.pb.h>

#include <ateam_data_conversion/ssl_vision_adapter.hpp>

namespace ateam_data_conversion
{
using MessageVariant_t = std::variant<SSL_WrapperPacket, Referee>;

enum MessageType
{
    MESSAGE_BLANK = 0,
    MESSAGE_UNKNOWN = 1,
    MESSAGE_SSL_VISION_2010 = 2,
    MESSAGE_SSL_REFBOX_2013 = 3
};

struct FileHeader
{
    char name[12]; // SSL_LOG_FILE
    int32_t version; // Default file format is version 1
};

// ok example definetly used int64_t but like wut.... why
struct DataHeader
{
    uint64_t timestamp_ns; // Timestamp in ns
    int32_t messageType; // Message type
    int32_t messageSize; // Size of protobuf message in bytes
};

struct LogMessage
{
    DataHeader header;
    MessageVariant_t msg;
};

// this could be a func not a class
// I mean so could all of this its a pipeline with callbacks is what im writing
class SslLogReplayer
{
public: // vars
    FileHeader file_header;

public: // ctors
    explict SslLogReplayer(std::string filename, vision_ufunc_t);

private: // funcs
    // should probably have made two funcs one for header one for data and just always pass in only bytes not file
    // just all stolen from example and copy pasted
    std::optional<LogMessage> extract_message_(std::ifstream&);

    // just copy pasta from example
    static constexpr std::string DEFAULT_FILE_HEADER_NAME = "SSL_LOG_FILE";
    static constexpr int32_t DEFAULT_FILE_VERSION = 1;
};

} // namespace
#endif // ATEAM_DATA_CONVERSION__SSL_LOG_REPLAYER_HPP_