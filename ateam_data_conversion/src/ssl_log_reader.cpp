#include <ateam_data_conversion/ssl_log_reader.hpp>

#include <vector>

// only needed in implementation
#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>
#include <ssl_league_protobufs/ssl_gc_referee_message.pb.h>

#include <cstring> // For strerror
#include <iostream> // For std::cerr

namespace ateam_data_conversion
{

SSLLogReader::SSLLogReader(std::string filename) {
    in_.open(filename, std::ios_base::in | std::ios_base::binary);

    if (!in_.is_open()) {
        std::cerr << "Error opening log file \"" << filename << "  . Error code: " << strerror(errno) << std::endl;
    }

    // Yes this is exactly how the example does it a raw cast read that depends on abi
    in_.read((char*) &file_header_, sizeof(file_header_));
    // Log data is stored big endian, convert to host byte order
    file_header_.version = be32toh(file_header_.version);

    if (strncmp(file_header_.name, DEFAULT_FILE_HEADER_NAME, sizeof(file_header_.name)) == 0) {
        std::cout << "File format version " << file_header_.version << " detected." << std::endl;
    } else {
        std::cerr << "Error log file is unsupported or corrupted!" << std::endl;
    }
}

bool SSLLogReader::has_next() {
    return !in_.eof();
}

std::optional<MessageVariant_t> SSLLogReader::next() {
    // TODO(Collin) Should I just skip nullopts and always give the user something

    DataHeader header;
    in_.read((char*) &header, sizeof(header));
    // Log data is stored big endian, convert to host byte order
    // from comparing times sent in the vision filter and ref this is time logged basically
    header.timestamp_ns = be64toh(header.timestamp_ns);
    header.type = be32toh(header.type);
    header.message_size = be32toh(header.message_size);

    std::vector<char> data(header.message_size, 0);
    in_.read(data.data(), header.message_size);

    SSL_WrapperPacket vision_proto;
    Referee referee_proto;
    // Do people stylistically care about break here since everything returns?
    switch (header.type) {
        case MessageType::MESSAGE_UNKNOWN:
        {
            // Have to figure out type by trying to parse the message
            if (referee_proto.ParseFromArray(data.data(), data.size())) {
                auto msg = ateam_game_controller_bridge::message_conversions::fromProto(referee_proto);
                msg.timestamp = rclcpp::Time(header.timestamp_ns);
                return msg;
            } else if (vision_proto.ParseFromArray(data.data(), data.size())) {
                // vision timestamps
                auto msg = ateam_ssl_vision_bridge::message_conversions::fromProto(vision_proto);
                if (msg.detection.size() > 0) {
                    msg.detection.at(0).t_sent = rclcpp::Time(header.timestamp_ns);
                }
                return msg;
            } else {
                std::cerr << "Error unsupported or corrupt packet found in log file!" << std::endl;
                return std::nullopt;
            }
        }
        case MessageType::MESSAGE_SSL_REFBOX_2013:
        {
            referee_proto.ParseFromArray(data.data(), data.size());
            auto msg = ateam_game_controller_bridge::message_conversions::fromProto(referee_proto);
            msg.timestamp = rclcpp::Time(header.timestamp_ns);
            return msg;
        }
        case MessageType::MESSAGE_SSL_VISION_2010:
        case MessageType::MESSAGE_SSL_VISION_2014:
        {
            vision_proto.ParseFromArray(data.data(), data.size());
            auto msg = ateam_ssl_vision_bridge::message_conversions::fromProto(vision_proto);
            if (msg.detection.size() > 0) {
                msg.detection.at(0).t_sent = rclcpp::Time(header.timestamp_ns);
            }
            return msg;
        }
        case MessageType::MESSAGE_SSL_VISION_TRACKER_2020:
            // TODO(Collin) currently just discarded
            return std::nullopt;
        case MessageType::MESSAGE_BLANK:
            return std::nullopt;
        default:
            std::cerr << "Error unsupported message type found in log file!" << std::endl;
            return std::nullopt;
    }
}

} // ateam_data_conversion