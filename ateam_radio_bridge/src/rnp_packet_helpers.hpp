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

constexpr std::size_t kPacketHeaderSize = 8;

inline const std::map<std::type_index, DataType_t> DataTypeCodeMap = {
  {std::type_index(typeid(void)), DT_NO_DATA},
  {std::type_index(typeid(HelloData_t)), DT_HELLO_DATA},
  {std::type_index(typeid(BasicTelemetry_t)), DT_BASIC_TELEMETRY},
  {std::type_index(typeid(BasicControl_t)), DT_BASIC_CONTROL}
};

template<typename DataTypeType>
void SetDataPayload(RadioPacket & packet, const DataTypeType & payload)
{
  packet.data_type = DataTypeCodeMap.at(std::type_index(typeid(DataTypeType)));
  packet.data_length = sizeof(DataTypeType);
  auto payload_ptr = reinterpret_cast<const uint8_t *>(&payload);
  std::copy_n(payload_ptr, packet.data_length, packet.data);
}

template<typename DataTypeType>
RadioPacket_t CreatePacket(const CommandCode_t command_code, const DataTypeType & data)
{
  RadioPacket_t packet{
    kProtocolVersionMajor,
    kProtocolVersionMinor,
    command_code,
    DT_NO_DATA,
    0,
    {}
  };

  SetDataPayload(packet, data);

  return packet;
}

RadioPacket_t CreateEmptyPacket(const CommandCode_t command_code);

RadioPacket_t ParsePacket(const uint8_t * data, const std::size_t data_length, std::string & error);

using PacketDataVariant = std::variant<std::monostate, HelloData_t, BasicTelemetry_t,
    BasicControl_t>;

PacketDataVariant ExtractData(const RadioPacket_t & packet, std::string & error);

}  // ateam_radio_bridge

#endif  // RNP_PACKET_HELPERS_HPP_
