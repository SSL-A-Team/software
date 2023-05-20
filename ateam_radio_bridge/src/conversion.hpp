#ifndef CONVERSION_HPP_
#define CONVERSION_HPP_

#include <ateam_msgs/msg/robot_feedback.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <basic_control.h>
#include <basic_telemetry.h>
#include <hello_data.h>

namespace ateam_radio_bridge
{

ateam_msgs::msg::RobotFeedback Convert(const BasicTelemetry & basic_telemetry);


}

#endif  // CONVERSION_HPP_
