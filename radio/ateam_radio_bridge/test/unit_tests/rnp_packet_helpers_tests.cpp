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
#include <gmock/gmock.h>

#include <rnp_packet_helpers.hpp>

// TODO(barulicm) expand test coverage

// Can we correctly set the CRC?
TEST(SetCRC, BasicTest)
{
  RadioPacket packet{
    {
      0,
      CC_ACK,
      0,
      0
    },
    {}
  };
  ateam_radio_bridge::SetCRC(packet);
  EXPECT_EQ(packet.header.crc32, 2583214201u);
}

// Do we reject (some) bad CRC values?
TEST(HasCorrectCRC, BasicTest)
{
  RadioPacket packet{
    {
      2583214201,
      CC_ACK,
      0,
      0
    },
    {}
  };
  EXPECT_TRUE(ateam_radio_bridge::HasCorrectCRC(packet));


  packet.header.crc32 = 12; // arbitrary bad CRC value
  EXPECT_FALSE(ateam_radio_bridge::HasCorrectCRC(packet));
}

// Can we correctly create a data payload for a HelloRequest?
TEST(SetDataPayload, HelloRequest)
{
  RadioPacket packet;
  HelloRequest payload{
    8,  // robot ID
    TC_BLUE,
    0,  // coms repo dirty
    0,  // controls repo dirty
    0,  // firmware repo dirty
    0,  // bitfield reserved
    0,  // reserved
    {0, 0, 0, 0},  // coms hash
    {0, 0, 0, 0},  // controls hash
    {0, 0, 0, 0}   // firmware hash
  };
  ateam_radio_bridge::SetDataPayload(packet, payload);
  EXPECT_EQ(packet.header.data_length, 16);
  EXPECT_EQ(packet.data.hello_request.robot_id, 8);
  EXPECT_EQ(packet.data.hello_request.color, TC_BLUE);
}

// Do we create a packet correctly given a basic payload?
TEST(CreatePacket, HelloRequest)
{
  HelloRequest payload{
    12,  // robot ID
    TC_YELLOW,
    0,  // coms repo dirty
    0,  // controls repo dirty
    0,  // firmware repo dirty
    0,  // bitfield reserved
    0,  // reserved
    {0, 0, 0, 0},  // coms hash
    {0, 0, 0, 0},  // controls hash
    {0, 0, 0, 0}   // firmware hash
  };
  RadioPacket packet = ateam_radio_bridge::CreatePacket(CC_HELLO_REQ, payload);
  EXPECT_EQ(packet.header.crc32, 1293447015u);
  EXPECT_EQ(packet.header.command_code, CC_HELLO_REQ);
  EXPECT_EQ(packet.header.data_length, 16);
  EXPECT_EQ(packet.data.hello_request.robot_id, 12);
  EXPECT_EQ(packet.data.hello_request.color, TC_YELLOW);
}

// Is the format of our empty radio packets correct?
TEST(CreateEmptyPacket, Ack)
{
  RadioPacket packet = ateam_radio_bridge::CreateEmptyPacket(CC_ACK);
  EXPECT_EQ(packet.header.crc32, 2583214201u);
  EXPECT_EQ(packet.header.command_code, CC_ACK);
  EXPECT_EQ(packet.header.data_length, 0);
}

// Are we parsing packets correctly?
TEST(ParsePacket, HelloRequest)
{
  std::array<uint8_t, 24> data = {
    0, 0, 0, 0,  // crc
    CC_HELLO_REQ,  // command code
    0,  // reserved
    16, 0, // data length
    15,  // robot ID
    1,  // team color
    5,  // dirty flags
    0,  // reserved
    1, 2, 3, 4,  // coms hash
    5, 6, 7, 8,  // controls hash
    9, 10, 11, 12   // firmware hash
  };
  std::string error_msg;
  RadioPacket packet = ateam_radio_bridge::ParsePacket(data.data(), data.size(), error_msg);
  EXPECT_TRUE(error_msg.empty()) << "Unexpected error message: " << error_msg;
  EXPECT_EQ(packet.header.crc32, 0u);
  EXPECT_EQ(packet.header.command_code, CC_HELLO_REQ);
  EXPECT_EQ(packet.header.data_length, 16);
  EXPECT_EQ(packet.data.hello_request.robot_id, 15);
  EXPECT_EQ(packet.data.hello_request.color, TC_BLUE);
  EXPECT_EQ(packet.data.hello_request.coms_repo_dirty, 1);
  EXPECT_EQ(packet.data.hello_request.controls_repo_dirty, 0);
  EXPECT_EQ(packet.data.hello_request.firmware_repo_dirty, 1);
  EXPECT_THAT(packet.data.hello_request.coms_hash, testing::ElementsAreArray({1, 2, 3, 4}));
  EXPECT_THAT(packet.data.hello_request.controls_hash, testing::ElementsAreArray({5, 6, 7, 8}));
  EXPECT_THAT(packet.data.hello_request.firmware_hash, testing::ElementsAreArray({9, 10, 11, 12}));
}

// Are we able to extract data from a packet correctly?
TEST(ExtractData, HelloRequest)
{
  RadioPacket packet{
    {
      0,
      CC_HELLO_REQ,
      0,
      16
    },
    HelloRequest{
      4,  // robot ID
      TC_BLUE,
      0,  // coms repo dirty
      0,  // controls repo dirty
      0,  // firmware repo dirty
      0,  // bitfield reserved
      0,  // reserved
      {0, 0, 0, 0},  // coms hash
      {0, 0, 0, 0},  // controls hash
      {0, 0, 0, 0}   // firmware hash
    }
  };
  std::string error_msg;
  ateam_radio_bridge::PacketDataVariant data_var =
    ateam_radio_bridge::ExtractData(packet, error_msg);
  EXPECT_TRUE(error_msg.empty()) << "Unexpected error message: " << error_msg;
  ASSERT_TRUE(std::holds_alternative<HelloRequest>(data_var));
  HelloRequest data = std::get<HelloRequest>(data_var);
  EXPECT_EQ(data.robot_id, 4);
  EXPECT_EQ(data.color, TC_BLUE);
}
