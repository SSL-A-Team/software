// Copyright 2021 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include "rnp_packet_helpers.hpp"

#include <iostream>

#include <boost/crc.hpp>

namespace ateam_radio_bridge
{

/**
 * @brief Get the size of a radio packet based on its command code.
 *
 * Determines the size of a radio packet by inspecting its command code.
 * Different command codes correspond to different packet sizes, including the header size.
 * If the command code is not recognized, an exception is thrown.
 *
 * @param command_code The command code of the radio packet.
 * @return The total size of the radio packet in bytes.
 * @throws std::invalid_argument if the command code is unrecognized.
 */
std::size_t GetPacketSize(const CommandCode & command_code)
{
  // For now, packet size only depends on command code
  switch (command_code) {
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

/**
 * @brief Calculate CRC32 checksum for the provided RadioPacket.
 *
 * Calculates the CRC32 checksum for a RadioPacket by processing
 * the bytes from the end of the CRC field to the end of the packet. The result
 * is stored in the packet's crc32 field.
 *
 * @param packet The RadioPacket for which the CRC32 checksum is to be calculated.
 */
void SetCRC(RadioPacket & packet)
{
  const auto crc_size = sizeof(packet.crc32);
  const auto packet_size = GetPacketSize(packet.command_code);
  boost::crc_32_type crc;
  crc.process_bytes(
    reinterpret_cast<void *>(reinterpret_cast<uint8_t *>(&packet) + crc_size),
    packet_size - crc_size);
  packet.crc32 = crc.checksum();
}

/**
 * @brief Validate the CRC32 checksum of a given RadioPacket.
 *
 * Checks whether the CRC32 checksum of a RadioPacket is correct
 * by recalculating the checksum and comparing it with the stored value in the packet.
 *
 * @param packet The RadioPacket to validate.
 * @return true if the CRC32 checksum is correct, false otherwise.
 */
bool HasCorrectCRC(const RadioPacket & packet)
{
  const auto crc_size = sizeof(packet.crc32);
  const auto packet_size = GetPacketSize(packet.command_code);
  boost::crc_32_type crc;
  crc.process_bytes(
    reinterpret_cast<const void *>(reinterpret_cast<const uint8_t *>(&packet) +
    crc_size), packet_size - crc_size);
  return packet.crc32 == crc.checksum();
}

/**
 * @brief Create an empty radio packet with the specified command code.
 *
 * Generates a RadioPacket with minimal content. See
   software-communication/ateam-common-packets/radio.h for the packet
   definition.
 *
 * @param command_code The command code to be assigned to the created packet.
 * @return A RadioPacket with default values and the specified command code.
 *         The CRC32 checksum is computed and set.
 */
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

/**
 * @brief Parse a raw byte stream into a RadioPacket.
 *
 * Extracts the header information and populates a RadioPacket structure. 
 * from a raw byte stream. We validate the packet by checking for the correct
 * number of bytes, and matching protocol versions.
 *
 * @param data A pointer to the raw byte stream containing the packet data.
 * @param data_length The length of the raw byte stream.
 * @param error A reference to a string that will store an error message if parsing fails.
 * @return A RadioPacket structure representing the parsed packet. If parsing fails,
 *         an empty RadioPacket is returned, and the error parameter is set.
 */
RadioPacket ParsePacket(const uint8_t * data, const std::size_t data_length, std::string & error)
{
  RadioPacket packet;

  std::copy_n(data, kPacketHeaderSize, reinterpret_cast<uint8_t *>(&packet));

  const auto packet_size = GetPacketSize(packet.command_code);

  if (data_length != packet_size) {
    error = "Wrong number of bytes. Expected " + std::to_string(packet_size) + " but got " +
      std::to_string(data_length) + ".";
    return {};
  }

  if (packet.major_version != kProtocolVersionMajor ||
    packet.minor_version != kProtocolVersionMinor)
  {
    // TODO(barulicm) What should our version compatability rules actually be? This assumes they must match.
    error = "Protocol versions do not match.";
    return {};
  }

  std::copy_n(
    data + kPacketHeaderSize, packet_size - kPacketHeaderSize,
    reinterpret_cast<uint8_t *>(&packet.data));

  // TODO(barulicm) Firmware doesn't implement CRCs yet
  // if(!HasCorrectCRC(packet)) {
  //   error = "CRC value incorrect.";
  //   return {};
  // }

  return packet;
}

/**
 * @brief Extract payload data from a RadioPacket based on its command code.
 *
 * Extracts the payload data from a RadioPacket based on its command code
 * and returns it as a PacketDataVariant. We perform basic validation, checking the
 * correctness of the data length for specific command codes. If validation fails, an
 * error message is set, and an empty PacketDataVariant is returned.
 *
 * @param packet The RadioPacket from which to extract payload data.
 * @param error A reference to a string that will store an error message if extraction fails.
 * @return A PacketDataVariant containing the extracted payload data.
 *         If extraction fails, an empty PacketDataVariant is returned, and the error parameter is set.
 */
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
    case CC_CONTROL_DEBUG_TELEMETRY:
      {
        if (packet.data_length != sizeof(ControlDebugTelemetry)) {
          error = "Incorrect data length for ControlDebugTelemetry type.";
          break;
        }
        var = packet.data.control_debug_telemetry;
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
