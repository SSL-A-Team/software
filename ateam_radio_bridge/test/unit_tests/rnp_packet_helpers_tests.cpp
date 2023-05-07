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

}

TEST(CreatePacket, HelloRequest)
{

}

TEST(CreateEmptyPacket, Ack)
{

}

TEST(ParsePacket, HelloRequest)
{

}

TEST(ExtractData, HelloRequest)
{
  
}