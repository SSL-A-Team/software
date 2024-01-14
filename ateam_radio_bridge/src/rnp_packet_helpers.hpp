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
    BasicControl, ControlDebugTelemetry>;

PacketDataVariant ExtractData(const RadioPacket & packet, std::string & error);

}  // ateam_radio_bridge

#endif  // RNP_PACKET_HELPERS_HPP_
