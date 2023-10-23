#include <variant>
#include <vector>
#include <array>
#include <optional>
#include <function>
#include <iostream>
#include <fstream>

#include <ssl_league_msgs/msg/vision_wrapper.hpp>
#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>

#include <ssl_league_msgs/msg/referee.hpp>
#include <ssl_league_protobufs/ssl_gc_referee_message.pb.h>


// ssl_league_msgs::msg::VisionWrapper
// ssl_league_msgs::msg::Referee


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


const std::string DEFAULT_FILE_HEADER_NAME = "SSL_LOG_FILE";
const int32_t DEFAULT_FILE_VERSION = 1;

struct FileHeader
{
    char name[12]; // SSL_LOG_FILE
    int32_t version; // Default file format is version 1
};

struct DataHeader
{
    int64_t timestamp; // Timestamp in ns
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
SslLogReplayer::SslLogReplayer(std::string filename, vision_ufunc_t ufunc) {
    std::ifstream in(filename, std::ios_base::in | std::ios_base::binary);

    if (!in.is_open()) {
        std::cerr << "Error opening log file \"" << filename << "\"!" << std::endl;
    }

    in.read((char*) &fileHeader, sizeof(fileHeader));
    // Log data is stored big endian, convert to host byte order
    fileHeader.version = be32toh(fileHeader.version);

    if (strncmp(fileHeader.name, DEFAULT_FILE_HEADER_NAME, sizeof(fileHeader.name)) == 0) {
        std::cout << "File format version " << fileHeader.version << " detected." << std::endl;

        if (fileHeader.version == DEFAULT_FILE_VERSION) {
            unsigned long refereePackets = 0;
            unsigned long visionPackets = 0;

            while (!in.eof()) {
                if (auto maybe_msg = extract_message_(in)) {
                  ufunc(maybe_msg.value());
                }
            }
        }
    } else {
        std::cerr << "Error log file is unsupported or corrupted!" << std::endl;
    }
}

private: // funcs
// should probably have made two funcs one for header one for data and just always pass in only bytes not file
// just all stolen from example and copy pasted
std::optional<LogMessage> extract_message_(std::ifstream& in) {
    // process each message
    LogMessage msg;
    in.read((uint8_t*) &msg.header, sizeof(msg.header));
    // Log data is stored big endian, convert to host byte order
    msg.header.timestamp = be64toh(msg.header.timestamp);
    msg.header.type = be32toh(msg.header.type);
    msg.header.messageSize = be32toh(msg.header.messageSize);

    // TODO Probably should just have a fixed size array
    std::vector<uint8_t> data {msg.header.messageSize};
    in.read(data.data(), msg.header.messageSize);

    SSL_WrapperPacket vision_proto;
    Referee referee_proto;

    if (msg.header.type == MessageType::MESSAGE_BLANK) {
        // ignore
    } else if (msg.header.type == MessageType::MESSAGE_UNKNOWN) {
        // OK, let's try to figure this out by parsing the message
        if (referee_proto.ParseFromArray(data.data(), data.size())) {
            msg.msg = referee_proto;
        } else if (vision_proto.ParseFromArray(data.data(), data.size())) {
            msg.msg = vision_proto;
        } else {
            std::cout << "Error unsupported or corrupt packet found in log file!" << std::endl;
            return std::nullopt;
        }
    } else if (msg.header.type == MessageType::MESSAGE_SSL_VISION_2010) {
        msg.msg = vision_proto;
    } else if (msg.header.type == MessageType::MESSAGE_SSL_REFBOX_2013) {
        msg.msg = vision_proto;
    } else if (msg.header.type == MessageType::MESSAGE_SSL_VISION_2014) {
        msg.msg = vision_proto;
    } else {
        std::cout << "Error unsupported message type found in log file!" << std::endl;
        return std::nullopt;
    }
    return msg
}
};
