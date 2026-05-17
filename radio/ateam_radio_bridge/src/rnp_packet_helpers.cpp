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
  const auto packet_header_size = sizeof(RadioHeader);
  // For now, packet size only depends on command code
  switch (command_code) {
    case CC_ACK:
      return packet_header_size;
      break;
    case CC_NACK:
      return packet_header_size;
      break;
    case CC_GOODBYE:
      return packet_header_size;
      break;
    case CC_KEEPALIVE:
      return packet_header_size;
      break;
    case CC_HELLO_REQ:
      return packet_header_size + sizeof(HelloRequest);
      break;
    case CC_HELLO_RESP:
      return packet_header_size + sizeof(HelloResponse);
      break;
    case CC_TELEMETRY:
      return packet_header_size + sizeof(BasicTelemetry);
      break;
    case CC_CONTROL_DEBUG_TELEMETRY:
      return packet_header_size + sizeof(ExtendedTelemetry);
      break;
    case CC_ROBOT_PARAMETER_COMMAND:
      return packet_header_size + sizeof(ParameterCommand);
      break;
    case CC_ERROR_TELEMETRY:
      return packet_header_size + sizeof(ErrorTelemetry);
    case CC_CONTROL:
      return packet_header_size + sizeof(BasicControl);
      break;
    default:
      throw std::invalid_argument("Unrecognized command code: " + std::to_string(command_code));
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
  const auto crc_size = sizeof(packet.header.crc32);
  const auto packet_size = GetPacketSize(packet.header.command_code);
  boost::crc_32_type crc;
  crc.process_bytes(
    reinterpret_cast<void *>(reinterpret_cast<uint8_t *>(&packet) + crc_size),
    packet_size - crc_size);
  packet.header.crc32 = crc.checksum();
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
  const auto crc_size = sizeof(packet.header.crc32);
  const auto packet_size = GetPacketSize(packet.header.command_code);
  boost::crc_32_type crc;
  crc.process_bytes(
    reinterpret_cast<const void *>(reinterpret_cast<const uint8_t *>(&packet) +
    crc_size), packet_size - crc_size);
  return packet.header.crc32 == crc.checksum();
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
    RadioHeader{
      0,  // crc32
      command_code,
      0,  // reserved
      0  // data length
    },
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
  const auto packet_header_size = sizeof(RadioHeader);

  RadioPacket packet;

  std::copy_n(data, packet_header_size, reinterpret_cast<uint8_t *>(&packet));

  std::size_t packet_size = 0;
  try {
    packet_size = GetPacketSize(packet.header.command_code);
  } catch (const std::invalid_argument & e) {
    error = "Error getting packet size: " + std::string(e.what());
    return {};
  }

  if (data_length != packet_size) {
    error = "Wrong number of bytes. Expected " + std::to_string(packet_size) + " but got " +
      std::to_string(data_length) + ".";
    return {};
  }

  std::copy_n(
    data + packet_header_size, packet_size - packet_header_size,
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

  switch (packet.header.command_code) {
    case CC_HELLO_REQ:
      {
        if (packet.header.data_length != sizeof(HelloRequest)) {
          error = "Incorrect data length for HelloData type.";
          break;
        }
        var = packet.data.hello_request;
        break;
      }
    case CC_TELEMETRY:
      {
        if (packet.header.data_length != sizeof(BasicTelemetry)) {
          error = "Incorrect data length for BasicTelemetry type.";
          break;
        }
        var = packet.data.telemetry;
        break;
      }
    case CC_CONTROL_DEBUG_TELEMETRY:
      {
        if (packet.header.data_length != sizeof(ExtendedTelemetry)) {
          error = "Incorrect data length for ExtendedTelemetry type.";
          break;
        }
        var = packet.data.extended_telemetry;
        break;
      }
    case CC_ROBOT_PARAMETER_COMMAND:
      {
        if (packet.header.data_length != sizeof(ParameterCommand)) {
          error = "Incorrect data length for ParameterCommand type.";
          break;
        }
        var = packet.data.robot_parameter_command;
        break;
      }
    case CC_CONTROL:
      {
        if (packet.header.data_length != sizeof(BasicControl)) {
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

ParameterDataFormat GetParameterDataFormatForParameter(const ParameterName & parameter)
{
  switch(parameter) {
    case KF_PROCESS_STD:
      return VEC4_F32;
    case KF_MEASUREMENT_STD:
      return VEC4_F32;
    case KF_MAX_STATE:
      return VEC4_F32;
    case PHYS_WHEEL:
      return VEC4_F32;
    case PHYS_INERTIA:
      return VEC2_F32;
    case PHYS_MOTOR_MODEL:
      return VEC2_F32;
    case PHYS_FRICTION_MODEL:
      return VEC4_F32;
    case POSE_CONTROL_GAIN:
      return VEC2_F32;
    case TRAJ_RECOMPUTE_ERROR:
      return VEC4_F32;
    case POSE_FB_PIDII_LINEAR:
      return VEC5_F32;
    case POSE_FB_PIDII_ANGULAR:
      return VEC5_F32;
    case TWIST_FB_PIDII_LINEAR:
      return VEC5_F32;
    case TWIST_FB_PIDII_ANGULAR:
      return VEC5_F32;
    default:
      throw std::invalid_argument("GetParameterDataFormatForParameter: Unrecognized parameter name.");
  }
}

std::size_t GetDataSizeForParameterFormat(const ParameterDataFormat & format)
{
  switch(format) {
    case F32:
      return 1;
    case VEC2_F32:
      return 2;
    case VEC3_F32:
      return 3;
    case VEC4_F32:
      return 4;
    case VEC5_F32:
      return 5;
    default:
      throw std::invalid_argument("GetDataSizeForParameterFormat: Unrecognized parameter data format.");
  }
}


float* GetParameterDataForSetFormat(ParameterCommand & command)
{
  switch(command.data_format) {
    case F32:
      return &command.data.f32;
    case VEC2_F32:
      return command.data.vec2_f32;
    case VEC3_F32:
      return command.data.vec3_f32;
    case VEC4_F32:
      return command.data.vec4_f32;
    case VEC5_F32:
      return command.data.vec5_f32;
    default:
      throw std::invalid_argument("GetParameterDataForSetFormat: Unrecognized parameter data format.");
  }
}


bool CheckParameterPacketAck(const ParameterCommand & packet, std::string & error_reason)
{
  if(packet.command_code == PCC_NACK_INVALID_NAME) {
    error_reason = "Invalid parameter name.";
    return false;
  }
  if(packet.command_code == PCC_NACK_INVALID_TYPE_FOR_NAME) {
    error_reason = "Wrong parameter type for given parameter.";
    return false;
  }
  if(packet.command_code != PCC_ACK) {
    error_reason = "Non ack/nack command code recieved.";
    return false;
  }
  return true;
}

}  // namespace ateam_radio_bridge
