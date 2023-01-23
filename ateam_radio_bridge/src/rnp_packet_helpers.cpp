#include "rnp_packet_helpers.hpp"

namespace ateam_radio_bridge
{

RadioPacket CreateEmptyPacket(const CommandCode command_code)
{
  RadioPacket packet{
    0,
    kProtocolVersionMajor,
    kProtocolVersionMinor,
    command_code,
    0,
    {}
  };

  return packet;
}

RadioPacket ParsePacket(const uint8_t * data, const std::size_t data_length, std::string & error)
{
  RadioPacket packet;

  std::copy_n(data, kPacketHeaderSize, reinterpret_cast<uint8_t *>(&packet));
  if ((packet.data_length + kPacketHeaderSize) > data_length) {
    error = "Claimed packet size is larger than UDP packet size.";
    return {};
  }
  std::copy_n(data + kPacketHeaderSize, packet.data_length, &packet.data);

  if (packet.major_version != kProtocolVersionMajor ||
    packet.minor_version != kProtocolVersionMinor)
  {
    // TODO(barulicm) What should our version compatability rules actually be? This assumes they must match.
    error = "Protocol versions do not match.";
    return {};
  }

  return packet;
}

PacketDataVariant ExtractData(const RadioPacket & packet, std::string & error)
{
  PacketDataVariant var;

  switch (packet.command_code) {
    case CC_HELLO_REQ:
      {
        if (packet.data_length != sizeof(HelloRequest)) {
          error = "Incorrect data length for HelloRequest type.";
          break;
        }
        var = packet.data.hello_request;
        break;
      }
    case CC_TELEMETRY:
      {
        if (packet.data_length != sizeof(BasicTelemetry)) {
          error = "Incorrect data length for BasicTelemetry type.";
          break;
        }
        var = packet.data.telemetry;
        break;
      }
    case CC_CONTROL:
      {
        if (packet.data_length != sizeof(BasicControl)) {
          error = "Incorrect data length for BasicControl type.";
          break;
        }
        var = packet.data.control;
        break;
      }
  }

  return var;
}

}  // namespace ateam_radio_bridge
