#include <gtest/gtest.h>
#include "conversion.hpp"

TEST(Convert, BasicTelemmetry)
{
  BasicTelemetry telemetry {
    0,
    1,
    2,
    3.4,
    5.6,
    0,
    1,
    0,
    1,
    0,
    0,
    1,
    1,
    0,
    1,
    1,
    0,
    1,
    1,
    0,
    1,
    0,
    0,
    1,
    0,
    0,
    1.2,
    3.4,
    5.6,
    7.8,
    9.0,
    1.2
  };
  const auto feedback_msg = ateam_radio_bridge::Convert(telemetry);
  EXPECT_EQ(feedback_msg.sequence_number, 0);
  EXPECT_EQ(feedback_msg.robot_revision_major, 1);
  EXPECT_EQ(feedback_msg.robot_revision_minor, 2);
  EXPECT_FLOAT_EQ(feedback_msg.battery_level, 3.4);
  EXPECT_FLOAT_EQ(feedback_msg.battery_temperature, 5.6);
  EXPECT_EQ(feedback_msg.power_error, 0);
  EXPECT_EQ(feedback_msg.tipped_error, 1);
  EXPECT_EQ(feedback_msg.breakbeam_error, 0);
  EXPECT_EQ(feedback_msg.breakbeam_ball_detected, 1);
  EXPECT_EQ(feedback_msg.accelerometer_0_error, 0);
  EXPECT_EQ(feedback_msg.accelerometer_1_error, 0);
  EXPECT_EQ(feedback_msg.gyroscope_0_error, 1);
  EXPECT_EQ(feedback_msg.gyroscope_1_error, 1);
  EXPECT_EQ(feedback_msg.motor_0_general_error, 0);
  EXPECT_EQ(feedback_msg.motor_0_hall_error, 1);
  EXPECT_EQ(feedback_msg.motor_1_general_error, 1);
  EXPECT_EQ(feedback_msg.motor_1_hall_error, 0);
  EXPECT_EQ(feedback_msg.motor_2_general_error, 1);
  EXPECT_EQ(feedback_msg.motor_2_hall_error, 1);
  EXPECT_EQ(feedback_msg.motor_3_general_error, 0);
  EXPECT_EQ(feedback_msg.motor_3_hall_error, 1);
  EXPECT_EQ(feedback_msg.motor_4_general_error, 0);
  EXPECT_EQ(feedback_msg.motor_4_hall_error, 0);
  EXPECT_EQ(feedback_msg.chipper_available, 1);
  EXPECT_EQ(feedback_msg.kicker_available, 0);
  EXPECT_FLOAT_EQ(feedback_msg.motor_0_temperature, 1.2);
  EXPECT_FLOAT_EQ(feedback_msg.motor_1_temperature, 3.4);
  EXPECT_FLOAT_EQ(feedback_msg.motor_2_temperature, 5.6);
  EXPECT_FLOAT_EQ(feedback_msg.motor_3_temperature, 7.8);
  EXPECT_FLOAT_EQ(feedback_msg.motor_4_temperature, 9.0);
  EXPECT_FLOAT_EQ(feedback_msg.kicker_charge_level, 1.2);
}
