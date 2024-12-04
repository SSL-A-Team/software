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

#ifndef ATEAM_COMMON__TOPIC_NAMES_HPP_
#define ATEAM_COMMON__TOPIC_NAMES_HPP_

#include <string_view>

namespace Topics
{
// Input from other systems
constexpr std::string_view kVisionMessages = "/vision_messages";  // Raw vision protobufs
constexpr std::string_view kRefereeMessages = "/referee_messages";  // Raw ref protobufs

// Input from robots
constexpr std::string_view kRobotFeedbackPrefix = "/robot_feedback/status/robot";
constexpr std::string_view kRobotMotionFeedbackPrefix = "/robot_feedback/motion/robot";

// Output from vision filter
constexpr std::string_view kBall = "/ball";
constexpr std::string_view kYellowTeamRobotPrefix = "/yellow_team/robot";
constexpr std::string_view kBlueTeamRobotPrefix = "/blue_team/robot";
constexpr std::string_view kVisionState = "/vision_state";  // Internal vision filter state

// from field republisher
constexpr std::string_view kField = "/field";  // Internal vision filter state

// Output from joysticks
constexpr std::string_view kJoystick = "/joy";

// Output from AI
constexpr std::string_view kRobotMotionCommandPrefix = "/robot_motion_commands/robot";
}  // namespace Topics

#endif  // ATEAM_COMMON__TOPIC_NAMES_HPP_
