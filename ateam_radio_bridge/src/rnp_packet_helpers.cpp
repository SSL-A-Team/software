#include "rnp_packet_helpers.hpp"

#include <iostream>

#include <boost/crc.hpp>

namespace ateam_radio_bridge
{

std::size_t GetPacketSize(const CommandCode & command_code)
{
  // For now, packet size only depends on command code
  switch(command_code)
  {
    case CC_ACK:
      return kPacketHeaderSize;
      break;
    case CC_NACK:
      return kPacketHeaderSize;
      break;
    case CC_GOODBYE:
      return kPacketHeaderSize;
      break;
    case CC_KEEPALIVE:
      return kPacketHeaderSize;
      break;
    case CC_HELLO_REQ:
      return kPacketHeaderSize + sizeof(HelloRequest);
      break;
    case CC_TELEMETRY:
      return kPacketHeaderSize + sizeof(BasicTelemetry);
      break;
    case CC_CONTROL:
      return kPacketHeaderSize + sizeof(BasicControl);
      break;
    case CC_HELLO_RESP:
      return kPacketHeaderSize + sizeof(HelloResponse);
      break;
    default:
      throw std::invalid_argument("Unrecognized command code.");
  }
}

void SetCRC(RadioPacket & packet)
{
  const auto crc_size = sizeof(packet.crc32);
  const auto packet_size = GetPacketSize(packet.command_code);
  boost::crc_32_type crc;
  crc.process_bytes(reinterpret_cast<void*>(reinterpret_cast<uint8_t*>(&packet)+crc_size), packet_size-crc_size);
  packet.crc32 = crc.checksum();
}

bool HasCorrectCRC(const RadioPacket & packet)
{
  const auto crc_size = sizeof(packet.crc32);
  const auto packet_size = GetPacketSize(packet.command_code);
  boost::crc_32_type crc;
  crc.process_bytes(reinterpret_cast<const void*>(reinterpret_cast<const uint8_t*>(&packet)+crc_size), packet_size-crc_size);
  return packet.crc32 == crc.checksum();
}

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

  SetCRC(packet);

  return packet;
}

RadioPacket ParsePacket(const uint8_t * data, const std::size_t data_length, std::string & error)
{
  RadioPacket packet;

  std::copy_n(data, kPacketHeaderSize, reinterpret_cast<uint8_t *>(&packet));

  const auto packet_size = GetPacketSize(packet.command_code);

  if(data_length != packet_size) {
    error = "Wrong number of bytes. Expected " + std::to_string(packet_size) + " but got " + std::to_string(data_length) + ".";
    return {};
  }

  std::copy_n(data+kPacketHeaderSize, packet_size - kPacketHeaderSize, reinterpret_cast<uint8_t*>(&packet.data));

  if (packet.major_version != kProtocolVersionMajor ||
    packet.minor_version != kProtocolVersionMinor)
  {
    // TODO(barulicm) What should our version compatability rules actually be? This assumes they must match.
    error = "Protocol versions do not match.";
    return {};
  }

  // TODO(barulicm) Firmware doesn't implement CRCs yet
  // if(!HasCorrectCRC(packet)) {
  //   error = "CRC value incorrect.";
  //   return {};
  // }

  return packet;
}

PacketDataVariant ExtractData(const RadioPacket & packet, std::string & error)
{
  PacketDataVariant var;

  switch (packet.command_code) {
    case CC_HELLO_REQ:
      {
        if (packet.data_length != sizeof(HelloRequest)) {
          error = "Incorrect data length for HelloData type.";
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
      default:
        // No data payload associated with given command code
        break;
  }

  return var;
}

}  // namespace ateam_radio_bridge
