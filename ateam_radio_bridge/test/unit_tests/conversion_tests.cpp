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

TEST(ConvertControlDebugTelemetry, PacketConversions) {
  MotorResponse_Motion_Packet front_left_motor_packet;
  front_left_motor_packet.vel_setpoint = 1.0f;
  MotorResponse_Motion_Packet back_left_motor_packet;
  back_left_motor_packet.vel_setpoint = 2.0f;
  MotorResponse_Motion_Packet back_right_motor_packet;
  back_right_motor_packet.vel_setpoint = 3.0f;
  MotorResponse_Motion_Packet front_right_motor_packet;
  front_right_motor_packet.vel_setpoint = 4.0f;

  ControlDebugTelemetry control_debug_telemetry {
    front_left_motor_packet,
    back_left_motor_packet,
    back_right_motor_packet,
    front_right_motor_packet,

    {10.0, 20.0, 30.0},
    {100.0, 200.0, 300.0},

    {1.6, 1.7, 6.28},
    {1.5, 1.4, 6.00},
    {1.2, 1.1, 5.00},
    {1.8, 1.9, 7.00},

    {4.0, 5.0, 6.0, 7.0},
    {1.0, 2.0, 3.0, 4.0}
  };

  const auto motion_feedback_msg = ateam_radio_bridge::Convert(control_debug_telemetry);

  // This just checks that the motor ordering is correct
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.FRONT_LEFT_MOTOR].vel_setpoint, 1.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.BACK_LEFT_MOTOR].vel_setpoint, 2.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.BACK_RIGHT_MOTOR].vel_setpoint, 3.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.motors[motion_feedback_msg.FRONT_RIGHT_MOTOR].vel_setpoint, 4.0);

  EXPECT_FLOAT_EQ(motion_feedback_msg.imu.orientation_covariance[0], -1.0);

  EXPECT_FLOAT_EQ(motion_feedback_msg.imu.angular_velocity.x, 10.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.imu.angular_velocity.y, 20.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.imu.angular_velocity.z, 30.0);

  EXPECT_FLOAT_EQ(motion_feedback_msg.imu.linear_acceleration.x, 100.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.imu.linear_acceleration.y, 200.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.imu.linear_acceleration.z, 300.0);

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
  EXPECT_FLOAT_EQ(motion_feedback_msg.wheel_velocity_control_variable[motion_feedback_msg.BACK_LEFT_MOTOR], 5.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.wheel_velocity_control_variable[motion_feedback_msg.BACK_RIGHT_MOTOR], 6.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.wheel_velocity_control_variable[motion_feedback_msg.FRONT_RIGHT_MOTOR], 7.0);

  EXPECT_FLOAT_EQ(motion_feedback_msg.clamped_wheel_velocity_control_variable[motion_feedback_msg.FRONT_LEFT_MOTOR], 1.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.clamped_wheel_velocity_control_variable[motion_feedback_msg.BACK_LEFT_MOTOR], 2.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.clamped_wheel_velocity_control_variable[motion_feedback_msg.BACK_RIGHT_MOTOR], 3.0);
  EXPECT_FLOAT_EQ(motion_feedback_msg.clamped_wheel_velocity_control_variable[motion_feedback_msg.FRONT_RIGHT_MOTOR], 4.0);
}


TEST(ConvertMotorFeedback, PacketConversions) {
  const MotorResponse_Motion_Packet src {
    0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1,
    0,
    1.0f,
    2.0f,
    3,
    4.0f,
    5.0f,
    6.0f,
    7.0f,
    8.0f,
    9.0f,
    10.0f,
    11.0f
  };

  const auto dst = ateam_radio_bridge::Convert(src);

  EXPECT_FALSE(dst.master_error);
  EXPECT_TRUE(dst.hall_power_error);
  EXPECT_FALSE(dst.hall_disconnected_error);
  EXPECT_TRUE(dst.bldc_transition_error);
  EXPECT_FALSE(dst.bldc_commutation_watchdog_error);
  EXPECT_FALSE(dst.enc_disconnected_error);
  EXPECT_TRUE(dst.enc_decoding_error);
  EXPECT_TRUE(dst.hall_enc_vel_disagreement_error);
  EXPECT_FALSE(dst.overcurrent_error);
  EXPECT_TRUE(dst.undervoltage_error);
  EXPECT_TRUE(dst.overvoltage_error);
  EXPECT_FALSE(dst.torque_limited);
  EXPECT_TRUE(dst.control_loop_time_error);
  EXPECT_FALSE(dst.reset_watchdog_independent);
  EXPECT_FALSE(dst.reset_watchdog_window);
  EXPECT_TRUE(dst.reset_low_power);
  EXPECT_FALSE(dst.reset_software);
  EXPECT_TRUE(dst.reset_pin);

  EXPECT_FLOAT_EQ(dst.vel_setpoint, 1.0f);
  EXPECT_FLOAT_EQ(dst.vel_setpoint_clamped, 2.0f);
  EXPECT_EQ(dst.encoder_delta, 3);
  EXPECT_FLOAT_EQ(dst.vel_enc_estimate, 4.0f);
  EXPECT_FLOAT_EQ(dst.vel_computed_error, 5.0f);
  EXPECT_FLOAT_EQ(dst.vel_computed_setpoint, 6.0f);

  EXPECT_FLOAT_EQ(dst.torque_setpoint, 7.0f);
  EXPECT_FLOAT_EQ(dst.current_estimate, 8.0f);
  EXPECT_FLOAT_EQ(dst.torque_estimate, 9.0f);
  EXPECT_FLOAT_EQ(dst.torque_computed_error, 10.0f);
  EXPECT_FLOAT_EQ(dst.torque_computed_setpoint, 11.0f);
}
