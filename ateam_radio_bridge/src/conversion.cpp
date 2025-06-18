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


#include "conversion.hpp"

#include <span>

namespace ateam_radio_bridge
{

ateam_msgs::msg::RobotFeedback Convert(const BasicTelemetry & basic_telemetry)
{
  ateam_msgs::msg::RobotFeedback robot_feedback;

  robot_feedback.sequence_number = basic_telemetry.sequence_number;
  robot_feedback.robot_revision_major = basic_telemetry.robot_revision_major;
  robot_feedback.robot_revision_minor = basic_telemetry.robot_revision_minor;
  robot_feedback.power_error = basic_telemetry.power_error;
  robot_feedback.tipped_error = basic_telemetry.tipped_error;
  robot_feedback.breakbeam_error = basic_telemetry.breakbeam_error;
  robot_feedback.breakbeam_ball_detected = basic_telemetry.breakbeam_ball_detected;
  robot_feedback.accelerometer_0_error = basic_telemetry.accelerometer_0_error;
  robot_feedback.accelerometer_1_error = basic_telemetry.accelerometer_1_error;
  robot_feedback.gyroscope_0_error = basic_telemetry.gyroscope_0_error;
  robot_feedback.gyroscope_1_error = basic_telemetry.gyroscope_1_error;
  robot_feedback.motor_0_general_error = basic_telemetry.motor_fl_general_error;
  robot_feedback.motor_0_hall_error = basic_telemetry.motor_fl_hall_error;
  robot_feedback.motor_1_general_error = basic_telemetry.motor_bl_general_error;
  robot_feedback.motor_1_hall_error = basic_telemetry.motor_bl_hall_error;
  robot_feedback.motor_2_general_error = basic_telemetry.motor_br_general_error;
  robot_feedback.motor_2_hall_error = basic_telemetry.motor_br_hall_error;
  robot_feedback.motor_3_general_error = basic_telemetry.motor_fr_general_error;
  robot_feedback.motor_3_hall_error = basic_telemetry.motor_fr_hall_error;
  robot_feedback.motor_4_general_error = basic_telemetry.motor_drib_general_error;
  robot_feedback.motor_4_hall_error = basic_telemetry.motor_drib_hall_error;
  robot_feedback.chipper_available = basic_telemetry.chipper_available;
  robot_feedback.kicker_available = basic_telemetry.kicker_available;


  return robot_feedback;
}

geometry_msgs::msg::Vector3 ConvertFloatArrayToVec3(const float (&float_arr_3)[3]) {
  geometry_msgs::msg::Vector3 vec;

  vec.x = float_arr_3[0];
  vec.y = float_arr_3[1];
  vec.z = float_arr_3[2];

  return vec;
}

geometry_msgs::msg::Twist ConvertFloatArrayToTwist(const float (&fw_state_space_array)[3]) {
  geometry_msgs::msg::Twist twist;

  twist.linear.x = fw_state_space_array[0];
  twist.linear.y = fw_state_space_array[1];
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = fw_state_space_array[2];

  return twist;
}

ateam_msgs::msg::RobotMotorFeedback Convert(const MotorTelemetry & motor_debug_telemetry) {
  ateam_msgs::msg::RobotMotorFeedback robot_motor_feedback;

  robot_motor_feedback.master_error = motor_debug_telemetry.master_error;
  robot_motor_feedback.hall_power_error = motor_debug_telemetry.hall_power_error;
  robot_motor_feedback.hall_disconnected_error = motor_debug_telemetry.hall_disconnected_error;
  robot_motor_feedback.bldc_transition_error = motor_debug_telemetry.bldc_transition_error;
  robot_motor_feedback.bldc_commutation_watchdog_error = motor_debug_telemetry.bldc_commutation_watchdog_error;
  robot_motor_feedback.enc_disconnected_error = motor_debug_telemetry.enc_disconnected_error;
  robot_motor_feedback.enc_decoding_error = motor_debug_telemetry.enc_decoding_error;
  robot_motor_feedback.hall_enc_vel_disagreement_error = motor_debug_telemetry.hall_enc_vel_disagreement_error;
  robot_motor_feedback.overcurrent_error = motor_debug_telemetry.overcurrent_error;
  robot_motor_feedback.undervoltage_error = motor_debug_telemetry.undervoltage_error;
  robot_motor_feedback.overvoltage_error = motor_debug_telemetry.overvoltage_error;
  robot_motor_feedback.torque_limited = motor_debug_telemetry.torque_limited;
  robot_motor_feedback.control_loop_time_error = motor_debug_telemetry.control_loop_time_error;
  robot_motor_feedback.reset_watchdog_independent = motor_debug_telemetry.reset_watchdog_independent;
  robot_motor_feedback.reset_watchdog_window = motor_debug_telemetry.reset_watchdog_window;
  robot_motor_feedback.reset_low_power = motor_debug_telemetry.reset_low_power;
  robot_motor_feedback.reset_software = motor_debug_telemetry.reset_software;
  robot_motor_feedback.reset_pin = motor_debug_telemetry.reset_pin;

  robot_motor_feedback.vel_setpoint = motor_debug_telemetry.vel_setpoint;
  robot_motor_feedback.vel_setpoint_clamped = motor_debug_telemetry.vel_setpoint_clamped;
  robot_motor_feedback.encoder_delta = motor_debug_telemetry.encoder_delta;
  robot_motor_feedback.vel_enc_estimate = motor_debug_telemetry.vel_enc_estimate;
  robot_motor_feedback.vel_computed_error = motor_debug_telemetry.vel_computed_error;
  robot_motor_feedback.vel_computed_setpoint = motor_debug_telemetry.vel_computed_setpoint;
  
  robot_motor_feedback.torque_setpoint = motor_debug_telemetry.torque_setpoint;
  robot_motor_feedback.current_estimate = motor_debug_telemetry.current_estimate;
  robot_motor_feedback.torque_estimate = motor_debug_telemetry.torque_estimate;
  robot_motor_feedback.torque_computed_error = motor_debug_telemetry.torque_computed_error;
  robot_motor_feedback.torque_computed_setpoint = motor_debug_telemetry.torque_computed_setpoint;
  
  return robot_motor_feedback;
}

ateam_msgs::msg::RobotMotionFeedback Convert(const ExtendedTelemetry & extended_telemetry) {
  ateam_msgs::msg::RobotMotionFeedback robot_motion_feedback;

  robot_motion_feedback.motors[robot_motion_feedback.FRONT_LEFT_MOTOR] = Convert(extended_telemetry.front_left_motor);
  robot_motion_feedback.motors[robot_motion_feedback.BACK_LEFT_MOTOR] = Convert(extended_telemetry.back_left_motor);
  robot_motion_feedback.motors[robot_motion_feedback.BACK_RIGHT_MOTOR] = Convert(extended_telemetry.back_right_motor);
  robot_motion_feedback.motors[robot_motion_feedback.FRONT_RIGHT_MOTOR] = Convert(extended_telemetry.front_right_motor);

  robot_motion_feedback.imu.orientation_covariance[0] = -1.0;  // ROS2 docs say if a sensor doesn't provide a data point, then set element '0' of it's covariance to -1
  robot_motion_feedback.imu.angular_velocity = ConvertFloatArrayToVec3(extended_telemetry.imu_gyro);
  robot_motion_feedback.imu.linear_acceleration = ConvertFloatArrayToVec3(extended_telemetry.imu_accel);

  robot_motion_feedback.body_velocity_setpoint = ConvertFloatArrayToTwist(extended_telemetry.commanded_body_velocity);
  robot_motion_feedback.clamped_body_velocity_setpoint = ConvertFloatArrayToTwist(extended_telemetry.clamped_commanded_body_velocity);
  robot_motion_feedback.body_velocity_state_estimate = ConvertFloatArrayToTwist(extended_telemetry.cgkf_body_velocity_state_estimate);
  robot_motion_feedback.body_velocity_control_variable = ConvertFloatArrayToTwist(extended_telemetry.body_velocity_u);

  std::span wheel_velocity_u_span{extended_telemetry.wheel_velocity_u};
  static_assert(wheel_velocity_u_span.size() == robot_motion_feedback.wheel_velocity_control_variable.size());
  std::ranges::copy(wheel_velocity_u_span, robot_motion_feedback.wheel_velocity_control_variable.begin());

  std::span wheel_velocity_clamped_u_span{extended_telemetry.wheel_velocity_clamped_u};
  static_assert(wheel_velocity_clamped_u_span.size() == robot_motion_feedback.clamped_wheel_velocity_control_variable.size());
  std::ranges::copy(wheel_velocity_clamped_u_span, robot_motion_feedback.clamped_wheel_velocity_control_variable.begin());

  return robot_motion_feedback;
}

}  // namespace ateam_radio_bridge
