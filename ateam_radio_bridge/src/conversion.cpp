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

namespace ateam_radio_bridge
{

ateam_msgs::msg::RobotFeedback ConvertBasicTelemetry(const BasicTelemetry & basic_telemetry)
{
  ateam_msgs::msg::RobotFeedback robot_feedback;

  robot_feedback.sequence_number = basic_telemetry.sequence_number;
  robot_feedback.robot_revision_major = basic_telemetry.robot_revision_major;
  robot_feedback.robot_revision_minor = basic_telemetry.robot_revision_minor;
  robot_feedback.battery_level = basic_telemetry.battery_level;
  robot_feedback.battery_temperature = basic_telemetry.battery_temperature;
  robot_feedback.power_error = basic_telemetry.power_error;
  robot_feedback.tipped_error = basic_telemetry.tipped_error;
  robot_feedback.breakbeam_error = basic_telemetry.breakbeam_error;
  robot_feedback.breakbeam_ball_detected = basic_telemetry.breakbeam_ball_detected;
  robot_feedback.accelerometer_0_error = basic_telemetry.accelerometer_0_error;
  robot_feedback.accelerometer_1_error = basic_telemetry.accelerometer_1_error;
  robot_feedback.gyroscope_0_error = basic_telemetry.gyroscope_0_error;
  robot_feedback.gyroscope_1_error = basic_telemetry.gyroscope_1_error;
  robot_feedback.motor_0_general_error = basic_telemetry.motor_0_general_error;
  robot_feedback.motor_0_hall_error = basic_telemetry.motor_0_hall_error;
  robot_feedback.motor_1_general_error = basic_telemetry.motor_1_general_error;
  robot_feedback.motor_1_hall_error = basic_telemetry.motor_1_hall_error;
  robot_feedback.motor_2_general_error = basic_telemetry.motor_2_general_error;
  robot_feedback.motor_2_hall_error = basic_telemetry.motor_2_hall_error;
  robot_feedback.motor_3_general_error = basic_telemetry.motor_3_general_error;
  robot_feedback.motor_3_hall_error = basic_telemetry.motor_3_hall_error;
  robot_feedback.motor_4_general_error = basic_telemetry.motor_4_general_error;
  robot_feedback.motor_4_hall_error = basic_telemetry.motor_4_hall_error;
  robot_feedback.chipper_available = basic_telemetry.chipper_available;
  robot_feedback.kicker_available = basic_telemetry.kicker_available;
  robot_feedback.motor_0_temperature = basic_telemetry.motor_0_temperature;
  robot_feedback.motor_1_temperature = basic_telemetry.motor_1_temperature;
  robot_feedback.motor_2_temperature = basic_telemetry.motor_2_temperature;
  robot_feedback.motor_3_temperature = basic_telemetry.motor_3_temperature;
  robot_feedback.motor_4_temperature = basic_telemetry.motor_4_temperature;
  robot_feedback.kicker_charge_level = basic_telemetry.kicker_charge_level;


  return robot_feedback;
}

void ConvertFloatArrayToVec3(const float (&float_arr_3)[3], geometry_msgs::msg::Vector3 & vec) {
  vec.x = float_arr_3[0];
  vec.y = float_arr_3[1];
  vec.z = float_arr_3[2];
}

geometry_msgs::msg::Vector3 ConvertFloatArrayToVec3(const float (&float_arr_3)[3]) {
  geometry_msgs::msg::Vector3 vec;

  ConvertFloatArrayToVec3(float_arr_3, vec);

  return vec;
}

void ConvertFloatArrayToTwist(const float (&fw_state_space_array)[3], geometry_msgs::msg::Twist & twist) {
  twist.linear.x = fw_state_space_array[0];
  twist.linear.y = fw_state_space_array[1];
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = fw_state_space_array[2];
}

geometry_msgs::msg::Twist ConvertFloatArrayToTwist(const float (&fw_state_space_array)[3]) {
  geometry_msgs::msg::Twist twist;

  ConvertFloatArrayToTwist(fw_state_space_array, twist);

  return twist;
}

void ConvertMotorDebugTelemetry(const MotorDebugTelemetry & motor_debug_telemetry, ateam_msgs::msg::RobotMotorFeedback & ateam_motor_feedback) {
  ateam_motor_feedback.setpoint = motor_debug_telemetry.wheel_setpoint;
  ateam_motor_feedback.velocity = motor_debug_telemetry.wheel_velocity;
  ateam_motor_feedback.torque = motor_debug_telemetry.wheel_torque;
}

ateam_msgs::msg::RobotMotorFeedback ConvertMotorDebugTelemetry(const MotorDebugTelemetry & motor_debug_telemetry) {
  ateam_msgs::msg::RobotMotorFeedback robot_motor_feedback;

  ConvertMotorDebugTelemetry(motor_debug_telemetry, robot_motor_feedback);

  return robot_motor_feedback;
}

ateam_msgs::msg::RobotMotionFeedback ConvertControlDebugTelemetry(const ControlDebugTelemetry & control_debug_telemetry) {
  ateam_msgs::msg::RobotMotionFeedback robot_motion_feedback;

  robot_motion_feedback.motors[robot_motion_feedback.FRONT_LEFT_MOTOR] = ConvertMotorDebugTelemetry(control_debug_telemetry.motor_fl);
  robot_motion_feedback.motors[robot_motion_feedback.FRONT_RIGHT_MOTOR] = ConvertMotorDebugTelemetry(control_debug_telemetry.motor_fr);
  robot_motion_feedback.motors[robot_motion_feedback.BACK_RIGHT_MOTOR] = ConvertMotorDebugTelemetry(control_debug_telemetry.motor_br);
  robot_motion_feedback.motors[robot_motion_feedback.BACK_LEFT_MOTOR] = ConvertMotorDebugTelemetry(control_debug_telemetry.motor_bl);

  robot_motion_feedback.imu.orientation_covariance[0] = -1.0;  // ROS2 docs say if a sensor doesn't provide a data point, then set element '0' of it's covariance to
  robot_motion_feedback.imu.angular_velocity = ConvertFloatArrayToVec3(control_debug_telemetry.imu_gyro);
  robot_motion_feedback.imu.linear_acceleration = ConvertFloatArrayToVec3(control_debug_telemetry.imu_accel);

  robot_motion_feedback.body_velocity_setpoint = ConvertFloatArrayToTwist(control_debug_telemetry.commanded_body_velocity);
  robot_motion_feedback.clamped_body_velocity_setpoint = ConvertFloatArrayToTwist(control_debug_telemetry.clamped_commanded_body_velocity);
  robot_motion_feedback.body_velocity_state_estimate = ConvertFloatArrayToTwist(control_debug_telemetry.cgkf_body_velocity_state_estimate);
  robot_motion_feedback.body_velocity_control_variable = ConvertFloatArrayToTwist(control_debug_telemetry.body_velocity_u);

  // copying from some C arrays here, not a struct so add some sanity checks
  assert((sizeof(control_debug_telemetry.wheel_velocity_u) / sizeof(control_debug_telemetry.wheel_velocity_u[0]) <= robot_motion_feedback.wheel_velocity_control_variable.size()));
  assert((sizeof(control_debug_telemetry.wheel_velocity_clamped_u) / sizeof(control_debug_telemetry.wheel_velocity_clamped_u[0]) <= robot_motion_feedback.clamped_wheel_velocity_control_variable.size()));
  std::copy(std::begin(control_debug_telemetry.wheel_velocity_u), std::end(control_debug_telemetry.wheel_velocity_u), std::begin(robot_motion_feedback.wheel_velocity_control_variable));
  std::copy(std::begin(control_debug_telemetry.wheel_velocity_clamped_u), std::end(control_debug_telemetry.wheel_velocity_clamped_u), std::begin(robot_motion_feedback.clamped_wheel_velocity_control_variable));

  return robot_motion_feedback;
}

}  // namespace ateam_radio_bridge
