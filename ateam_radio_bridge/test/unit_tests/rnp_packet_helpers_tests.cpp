#include <gtest/gtest.h>

#include <rnp_packet_helpers.hpp>

// TODO(barulicm) expand test coverage

TEST(SetCRC, BasicTest)
{
  RadioPacket packet{
    0,
    1,
    2,
    CC_ACK,
    0,
    {}
  };
  ateam_radio_bridge::SetCRC(packet);
  EXPECT_EQ(packet.crc32, 1560025497u);
}

TEST(HasCorrectCRC, BasicTest)
{
  RadioPacket packet{
    1560025497,
    1,
    2,
    CC_ACK,
    0,
    {}
  };
  EXPECT_TRUE(ateam_radio_bridge::HasCorrectCRC(packet));


  packet.crc32 = 12; // arbitrary bad CRC value
  EXPECT_FALSE(ateam_radio_bridge::HasCorrectCRC(packet));
}

TEST(SetDataPayload, HelloRequest)
{
  RadioPacket packet;
  HelloRequest payload{
    8,
    TC_BLUE
  };
  ateam_radio_bridge::SetDataPayload(packet, payload);
  EXPECT_EQ(packet.data_length, 2);
  EXPECT_EQ(packet.data.hello_request.robot_id, 8);
  EXPECT_EQ(packet.data.hello_request.color, TC_BLUE);
}

TEST(CreatePacket, HelloRequest)
{
  HelloRequest payload{
    12,
    TC_YELLOW
  };
  RadioPacket packet = ateam_radio_bridge::CreatePacket(CC_HELLO_REQ, payload);
  EXPECT_EQ(packet.crc32, 3171905816u);
  EXPECT_EQ(packet.major_version, kProtocolVersionMajor);
  EXPECT_EQ(packet.minor_version, kProtocolVersionMinor);
  EXPECT_EQ(packet.command_code, CC_HELLO_REQ);
  EXPECT_EQ(packet.data_length, 2);
  EXPECT_EQ(packet.data.hello_request.robot_id, 12);
  EXPECT_EQ(packet.data.hello_request.color, TC_YELLOW);
}

TEST(CreateEmptyPacket, Ack)
{
  RadioPacket packet = ateam_radio_bridge::CreateEmptyPacket(CC_ACK);
  EXPECT_EQ(packet.crc32, 3718166540u);
  EXPECT_EQ(packet.major_version, kProtocolVersionMajor);
  EXPECT_EQ(packet.minor_version, kProtocolVersionMinor);
  EXPECT_EQ(packet.command_code, CC_ACK);
  EXPECT_EQ(packet.data_length, 0);
}

TEST(ParsePacket, HelloRequest)
{
  std::array<uint8_t, 14> data = {
    0,
    0,
    0,
    0,
    kProtocolVersionMajor & 0xFF,
    kProtocolVersionMajor >> 8,
    kProtocolVersionMinor & 0xFF,
    kProtocolVersionMinor >> 8,
    CC_HELLO_REQ,
    0,
    2,
    0,
    15,
    1
  };
  std::string error_msg;
  RadioPacket packet = ateam_radio_bridge::ParsePacket(data.data(), data.size(), error_msg);
  EXPECT_TRUE(error_msg.empty()) << "Unexpected error message: " << error_msg;
  EXPECT_EQ(packet.crc32, 0u);
  EXPECT_EQ(packet.major_version, kProtocolVersionMajor);
  EXPECT_EQ(packet.minor_version, kProtocolVersionMinor);
  EXPECT_EQ(packet.command_code, CC_HELLO_REQ);
  EXPECT_EQ(packet.data_length, 2);
  EXPECT_EQ(packet.data.hello_request.robot_id, 15);
  EXPECT_EQ(packet.data.hello_request.color, TC_BLUE);
}

TEST(ExtractData, HelloRequest)
{
  RadioPacket packet{
    0,
    0,
    0,
    CC_HELLO_REQ,
    2,
    HelloRequest{4, TC_BLUE}
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