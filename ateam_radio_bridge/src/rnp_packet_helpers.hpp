#ifndef RNP_PACKET_HELPERS_HPP_
#define RNP_PACKET_HELPERS_HPP_

#include <algorithm>
#include <map>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <variant>
#include <radio.h>
#include <hello_data.h>
#include <basic_control.h>
#include <basic_telemetry.h>

namespace ateam_radio_bridge
{

std::size_t GetPacketSize(const CommandCode & packet);

constexpr std::size_t kPacketHeaderSize = 12;

void SetCRC(RadioPacket & packet);

bool HasCorrectCRC(const RadioPacket & packet);

template<typename DataTypeType>
void SetDataPayload(RadioPacket & packet, const DataTypeType & payload)
{
  packet.data_length = sizeof(DataTypeType);
  auto payload_ptr = reinterpret_cast<const uint8_t *>(&payload);
  std::copy_n(payload_ptr, packet.data_length, reinterpret_cast<uint8_t *>(&packet.data));
}

template<typename DataTypeType>
RadioPacket CreatePacket(const CommandCode command_code, const DataTypeType & data)
{
  RadioPacket packet{
    0,
    kProtocolVersionMajor,
    kProtocolVersionMinor,
    command_code,
    0,
    {}
  };

  SetDataPayload(packet, data);

  SetCRC(packet);

  return packet;
}

RadioPacket CreateEmptyPacket(const CommandCode command_code);

RadioPacket ParsePacket(const uint8_t * data, const std::size_t data_length, std::string & error);

using PacketDataVariant = std::variant<std::monostate, HelloRequest, BasicTelemetry,
    BasicControl>;

PacketDataVariant ExtractData(const RadioPacket & packet, std::string & error);

}  // ateam_radio_bridge

#endif  // RNP_PACKET_HELPERS_HPP_