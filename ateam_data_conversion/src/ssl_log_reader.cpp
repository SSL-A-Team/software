#include <ateam_data_conversion/ssl_log_reader.hpp>

#include <vector>

// only needed in implementation
#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>
#include <ssl_league_protobufs/ssl_gc_referee_message.pb.h>

namespace ateam_data_conversion
{

SSLLogReader::SSLLogReader(std::string filename) {
    in_.open(filename, std::ios_base::in | std::ios_base::binary);

    if (!in_.is_open()) {
        std::cerr << "Error opening log file \"" << filename << "\"!" << std::endl;
    }

    in_.read((char*) &file_header_, sizeof(file_header_));
    // Log data is stored big endian, convert to host byte order
    file_header_.version = be32toh(file_header_.version);

    // if (strncmp(file_header_.name, DEFAULT_FILE_HEADER_NAME, sizeof(file_header_.name)) == 0) {
    //     std::cout << "File format version " << file_header_.version << " detected." << std::endl;
    // } else {
    //     std::cerr << "Error log file is unsupported or corrupted!" << std::endl;
    // }
}

bool SSLLogReader::has_next() {
    return !in_.eof();
}

std::optional<MessageVariant_t> SSLLogReader::next() {
    // TODO(Collin) Should I just skip nullopts and always give the user something

    DataHeader header;
    in_.read((uint8_t*) &header, sizeof(header));
    // Log data is stored big endian, convert to host byte order
    header.timestamp_ns = be64toh(header.timestamp_ns);
    header.type = be32toh(header.type);
    header.message_size = be32toh(header.message_size);

    std::vector<uint8_t> data(header.message_size, 0);
    in_.read(data.data(), header.message_size);

    SSL_WrapperPacket vision_proto;
    Referee referee_proto;
    // Do people stylistically care about break here since everything returns?
    switch (header.type) {
        case MessageType::MESSAGE_UNKNOWN:
            // Have to figure out type by trying to parse the message
            if (referee_proto.ParseFromArray(data.data(), data.size())) {
                // ref has its own timestamp
                return ateam_game_controller_bridge::message_conversions::fromProto(referee_proto);
            } else if (vision_proto.ParseFromArray(data.data(), data.size())) {
                return ateam_ssl_vision_bridge::message_conversions::fromProto(vision_proto);
                // ros_msg.timestamp = rclcpp::Time(msg.header.timestamp.ns * 1000);
            } else {
                std::cout << "Error unsupported or corrupt packet found in log file!" << std::endl;
                return std::nullopt;
            }
        case MessageType::MESSAGE_SSL_VISION_2010:
        case MessageType::MESSAGE_SSL_VISION_2013:
        case MessageType::MESSAGE_SSL_VISION_2014:
            // could be done in one line
            vision_proto.ParseFromArray(data.data(), data.size());
            return ateam_ssl_vision_bridge::message_conversions::fromProto(vision_proto);
            // ros_msg.timestamp = rclcpp::Time(msg.header.timestamp.ns * 1000);
            // TODO(Collin) Unsure if this timestamp is useful because i didnt realize ssl vision has
            // a time sent and a time captured
            // See if this is the same. This is probably just time logged which is not useful then
        case MessageType::MESSAGE_BLANK:
            return std::nullopt;
        default:
            std::cout << "Error unsupported message type found in log file!" << std::endl;
            return std::nullopt;
    }
}

} // ateam_data_conversion