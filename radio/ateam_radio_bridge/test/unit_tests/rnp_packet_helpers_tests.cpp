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


#include <gtest/gtest.h>

#include <rnp_packet_helpers.hpp>

// TODO(barulicm) expand test coverage

// Can we correctly set the CRC?
TEST(SetCRC, BasicTest)
{
  RadioPacket packet{
    {0, CC_ACK, 0, 0},
    {}
  };
  ateam_radio_bridge::SetCRC(packet);
  EXPECT_TRUE(ateam_radio_bridge::HasCorrectCRC(packet));
}

// Do we reject (some) bad CRC values?
TEST(HasCorrectCRC, BasicTest)
{
  RadioPacket packet{
    {0, CC_ACK, 0, 0},
    {}
  };
  ateam_radio_bridge::SetCRC(packet);
  EXPECT_TRUE(ateam_radio_bridge::HasCorrectCRC(packet));

  packet.header.crc32 = 12;  // arbitrary bad CRC value
  EXPECT_FALSE(ateam_radio_bridge::HasCorrectCRC(packet));
}

// Can we correctly create a data payload for a HelloRequest?
TEST(SetDataPayload, HelloRequest)
{
  RadioPacket packet{};
  HelloRequest payload{};
  payload.robot_id = 8;
  payload.color = TC_BLUE;
  ateam_radio_bridge::SetDataPayload(packet, payload);
  EXPECT_EQ(packet.header.data_length, sizeof(HelloRequest));
  EXPECT_EQ(packet.data.hello_request.robot_id, 8);
  EXPECT_EQ(packet.data.hello_request.color, TC_BLUE);
}

// Do we create a packet correctly given a basic payload?
TEST(CreatePacket, HelloRequest)
{
  HelloRequest payload{};
  payload.robot_id = 12;
  payload.color = TC_YELLOW;
  RadioPacket packet = ateam_radio_bridge::CreatePacket(CC_HELLO_REQ, payload);
  EXPECT_EQ(packet.header.command_code, CC_HELLO_REQ);
  EXPECT_EQ(packet.header.data_length, sizeof(HelloRequest));
  EXPECT_EQ(packet.data.hello_request.robot_id, 12);
  EXPECT_EQ(packet.data.hello_request.color, TC_YELLOW);
  EXPECT_TRUE(ateam_radio_bridge::HasCorrectCRC(packet));
}

// Is the format of our empty radio packets correct?
TEST(CreateEmptyPacket, Ack)
{
  RadioPacket packet = ateam_radio_bridge::CreateEmptyPacket(CC_ACK);
  EXPECT_EQ(packet.header.command_code, CC_ACK);
  EXPECT_EQ(packet.header.data_length, 0);
  EXPECT_TRUE(ateam_radio_bridge::HasCorrectCRC(packet));
}

// Are we parsing packets correctly?
TEST(ParsePacket, Ack)
{
  RadioPacket source = ateam_radio_bridge::CreateEmptyPacket(CC_ACK);
  const uint8_t * data = reinterpret_cast<const uint8_t *>(&source);
  std::string error_msg;
  RadioPacket packet = ateam_radio_bridge::ParsePacket(
    data, ateam_radio_bridge::kPacketHeaderSize, error_msg);
  EXPECT_TRUE(error_msg.empty()) << "Unexpected error message: " << error_msg;
  EXPECT_EQ(packet.header.command_code, CC_ACK);
  EXPECT_EQ(packet.header.data_length, 0);
}

// Are we able to extract data from a packet correctly?
TEST(ExtractData, HelloRequest)
{
  HelloRequest hello{};
  hello.robot_id = 4;
  hello.color = TC_BLUE;
  RadioPacket packet = ateam_radio_bridge::CreatePacket(CC_HELLO_REQ, hello);
  std::string error_msg;
  ateam_radio_bridge::PacketDataVariant data_var =
    ateam_radio_bridge::ExtractData(packet, error_msg);
  EXPECT_TRUE(error_msg.empty()) << "Unexpected error message: " << error_msg;
  ASSERT_TRUE(std::holds_alternative<HelloRequest>(data_var));
  HelloRequest data = std::get<HelloRequest>(data_var);
  EXPECT_EQ(data.robot_id, 4);
  EXPECT_EQ(data.color, TC_BLUE);
}
