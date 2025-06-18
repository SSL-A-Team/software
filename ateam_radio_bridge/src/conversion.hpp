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


#ifndef CONVERSION_HPP_
#define CONVERSION_HPP_

#include <ateam_msgs/msg/robot_feedback.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/robot_motion_feedback.hpp>
#include <basic_control.h>
#include <basic_telemetry.h>
#include <extended_telemetry.h>
#include <hello_data.h>

namespace ateam_radio_bridge
{

ateam_msgs::msg::RobotFeedback Convert(const BasicTelemetry & basic_telemetry);
ateam_msgs::msg::RobotMotorFeedback Convert(const MotorTelemetry & motor_debug_telemetry);
ateam_msgs::msg::RobotMotionFeedback Convert(const ExtendedTelemetry & control_debug_telemetry);

}

#endif  // CONVERSION_HPP_
