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
