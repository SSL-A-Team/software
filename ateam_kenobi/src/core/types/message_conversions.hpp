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


#ifndef TYPES__MESSAGE_CONVERSIONS_HPP_
#define TYPES__MESSAGE_CONVERSIONS_HPP_

#include <ateam_msgs/msg/world.hpp>
#include <ateam_msgs/msg/behavior_executor_state.hpp>
#include <ateam_msgs/msg/trajectory.hpp>
#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <ateam_msgs/msg/field_sided_info.hpp>
#include <ateam_msgs/msg/referee_info.hpp>

#include "core/types/world.hpp"
#include "core/types/field.hpp"
#include "core/types/referee_info.hpp"

namespace ateam_kenobi::message_conversions
{
ateam_msgs::msg::BallState toMsg(const Ball & obj);
ateam_msgs::msg::RobotState toMsg(const Robot & obj);

ateam_msgs::msg::World toMsg(const World & obj);

}  // namespace ateam_kenobi::message_conversions

#endif  // TYPES__MESSAGE_CONVERSIONS_HPP_
