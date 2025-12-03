// Copyright 2025 A Team
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

#ifndef CORE__TYPES__ROBOT_COMMAND_HPP_
#define CORE__TYPES__ROBOT_COMMAND_HPP_

#include "core/motion/motion_intent.hpp"

namespace ateam_kenobi
{

enum class KickState : uint8_t
{
  Arm = 0,
  Disable = 1,
  KickNow = 2,
  KickOnTouch = 3,
  KickOnCaptured = 4,
  ChipNow = 5,
  ChipOnTouch = 6,
  ChipOnCaptured = 7
};

struct RobotCommand
{
  motion::MotionIntent motion_intent;

  KickState kick = KickState::Arm;
  double kick_speed = 0.0;

  double dribbler_speed = 0.0;
};

}  // namespace ateam_kenobi

#endif  // CORE__TYPES__ROBOT_COMMAND_HPP_
