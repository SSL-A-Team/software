#include "rnp_packet_helpers.hpp"

namespace ateam_radio_bridge
{

RadioPacket_t CreateEmptyPacket(const CommandCode_t command_code)
{
  RadioPacket_t packet{
    kProtocolVersionMajor,
    kProtocolVersionMinor,
    command_code,
    DT_NO_DATA,
    0,
    {}
  };

  return packet;
}

RadioPacket_t ParsePacket(const uint8_t * data, const std::size_t data_length, std::string & error)
{
  RadioPacket_t packet;

  std::copy_n(data, kPacketHeaderSize, reinterpret_cast<uint8_t *>(&packet));
  if ((packet.data_length + kPacketHeaderSize) > data_length) {
    error = "Claimed packet size is larger than UDP packet size.";
    return {};
  }
  std::copy_n(data + kPacketHeaderSize, packet.data_length, packet.data);

  if (packet.major_version != kProtocolVersionMajor ||
    packet.minor_version != kProtocolVersionMinor)
  {
    // TODO(barulicm) What should our version compatability rules actually be? This assumes they must match.
    error = "Protocol versions do not match.";
    return {};
  }

  return packet;
}

PacketDataVariant ExtractData(const RadioPacket_t & packet, std::string & error)
{
  PacketDataVariant var;

  switch (packet.data_type) {
    case DT_NO_DATA:
      break;
    case DT_HELLO_DATA:
      {
        if (packet.data_length != sizeof(HelloData_t)) {
          error = "Incorrect data length for HelloData type.";
          break;
        }
        HelloData_t hello_data;
        std::copy_n(packet.data, sizeof(HelloData_t), reinterpret_cast<uint8_t *>(&hello_data));
        var = hello_data;
        break;
      }
    case DT_BASIC_TELEMETRY:
      {
        if (packet.data_length != sizeof(BasicTelemetry_t)) {
          error = "Incorrect data length for BasicTelemetry type.";
          break;
        }
        BasicTelemetry_t basic_telemetry;
        std::copy_n(
          packet.data, sizeof(BasicTelemetry_t),
          reinterpret_cast<uint8_t *>(&basic_telemetry));
        var = basic_telemetry;
        break;
      }
    case DT_BASIC_CONTROL:
      {
        if (packet.data_length != sizeof(BasicControl_t)) {
          error = "Incorrect data length for BasicControl type.";
          break;
        }
        BasicControl_t basic_control;
        std::copy_n(
          packet.data, sizeof(BasicControl_t),
          reinterpret_cast<uint8_t *>(&basic_control));
        var = basic_control;
        break;
      }
  }

  return var;
}

}  // namespace ateam_radio_bridge
