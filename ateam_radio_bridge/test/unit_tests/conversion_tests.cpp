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

TEST(ConvertBasicTelemmetry, PacketConversions)
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
  const auto feedback_msg = ateam_radio_bridge::ConvertBasicTelemetry(telemetry);
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

TEST(ConvertControlDebugTelemetry, PacketConversions) {
  ControlDebugTelemetry control_debug_telemetry {
    MotorDebugTelemetry {
      3.14,
      3.12,
      2.0
    },

    MotorDebugTelemetry {
      3.14,
      3.14,
      3.0
    },

    MotorDebugTelemetry {
      3.14,
      3.15,
      2.0
    },

    MotorDebugTelemetry {
      3.14,
      3.20,
      1.0
    },

    {1.6, 1.7, 6.28},
    {1.5, 1.4, 6.00},
    {1.2, 1.1, 5.00},
    {1.8, 1.9, 7.00},

    {4.0, 5.0, 6.0, 7.0},
    {1.0, 2.0, 3.0, 4.0}
  };

  const auto motion_feedback_msg = ateam_radio_bridge::ConvertControlDebugTelemetry(control_debug_telemetry);
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.FRONT_LEFT_MOTOR].setpoint, 3.14);
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.FRONT_LEFT_MOTOR].velocity, 3.12);
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.FRONT_LEFT_MOTOR].torque, 2.0);

  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.FRONT_RIGHT_MOTOR].setpoint, 3.14);
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.FRONT_RIGHT_MOTOR].velocity, 3.14);
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.FRONT_RIGHT_MOTOR].torque, 3.0);

  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.BACK_RIGHT_MOTOR].setpoint, 3.14);
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.BACK_RIGHT_MOTOR].velocity, 3.15);
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.BACK_RIGHT_MOTOR].torque, 2.0);

  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.BACK_LEFT_MOTOR].setpoint, 3.14);
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.BACK_LEFT_MOTOR].velocity, 3.20);
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.BACK_LEFT_MOTOR].torque, 1.0);

  EXPECT_FLOAT_EQ(motion_feedback_msg.body_velocity_setpoint.linear.x, 1.6);
  EXPECT_FLOAT_EQ(motion_feedback_msg.body_velocity_setpoint.linear.y, 1.7);
  EXPECT_FLOAT_EQ(motion_feedback_msg.body_velocity_setpoint.angular.z, 6.28);

  EXPECT_FLOAT_EQ(motion_feedback_msg.clamped_body_velocity_setpoint.linear.x, 1.5);
  EXPECT_FLOAT_EQ(motion_feedback_msg.clamped_body_velocity_setpoint.linear.y, 1.4);
  EXPECT_FLOAT_EQ(motion_feedback_msg.clamped_body_velocity_setpoint.angular.z, 6.0);

  EXPECT_FLOAT_EQ(motion_feedback_msg.body_velocity_state_estimate.linear.x, 1.2);
  EXPECT_FLOAT_EQ(motion_feedback_msg.body_velocity_state_estimate.linear.y, 1.1);
  EXPECT_FLOAT_EQ(motion_feedback_msg.body_velocity_state_estimate.angular.z, 5.0);

  EXPECT_FLOAT_EQ(motion_feedback_msg.body_velocity_control_variable.linear.x, 1.8);
  EXPECT_FLOAT_EQ(motion_feedback_msg.body_velocity_control_variable.linear.y, 1.9);
  EXPECT_FLOAT_EQ(motion_feedback_msg.body_velocity_control_variable.angular.z, 7.0);

  EXPECT_FLOAT_EQ(motion_feedback_msg.wheel_velocity_control_variable[motion_feedback_msg.FRONT_LEFT_MOTOR], 4.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.wheel_velocity_control_variable[motion_feedback_msg.FRONT_RIGHT_MOTOR], 5.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.wheel_velocity_control_variable[motion_feedback_msg.BACK_RIGHT_MOTOR], 6.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.wheel_velocity_control_variable[motion_feedback_msg.BACK_LEFT_MOTOR], 7.0);

  EXPECT_FLOAT_EQ(motion_feedback_msg.clamped_wheel_velocity_control_variable[motion_feedback_msg.FRONT_LEFT_MOTOR], 1.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.clamped_wheel_velocity_control_variable[motion_feedback_msg.FRONT_RIGHT_MOTOR], 2.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.clamped_wheel_velocity_control_variable[motion_feedback_msg.BACK_RIGHT_MOTOR], 3.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.clamped_wheel_velocity_control_variable[motion_feedback_msg.BACK_LEFT_MOTOR], 4.0);
}