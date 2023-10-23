#include <ateam_data_conversion/ssl_log_replayer.hpp>

#include <iostream>
#include <fstream>
#include <array>

namespace ateam_data_conversion
{

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

std::optional<LogMessage> SslLogReplayer::extract_message_(std::ifstream& in) {
    // process each message
    LogMessage msg;
    in.read((uint8_t*) &msg.header, sizeof(msg.header));
    // Log data is stored big endian, convert to host byte order
    msg.header.timestamp_ns = be64toh(msg.header.timestamp_ns);
    msg.header.type = be32toh(msg.header.type);
    msg.header.messageSize = be32toh(msg.header.messageSize);

    std::array<uint8_t> data {msg.header.messageSize};
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

} // ateam_data_conversion