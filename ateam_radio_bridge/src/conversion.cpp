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

ateam_msgs::msg::RobotMotionFeedback ConvertControlDebugTelemetry(const ControlDebugTelemetry & control_debug_telemetry) {
  
}

}  // namespace ateam_radio_bridge
