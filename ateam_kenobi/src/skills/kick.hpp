// Copyright 2023 A Team
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

#ifndef SKILLS__KICK_HPP_
#define SKILLS__KICK_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>

namespace ateam_kenobi::skills
{
ateam_msgs::msg::RobotMotionCommand send_kick_command()
{
  auto kick_command = ateam_msgs::msg::RobotMotionCommand{};
  kick_command.kick = true;
  // Set kick speed to 5 m/s. Current max is 5.5 m/s (will
  // be clamped if above).
  kick_command.kick_speed = 5;
  return kick_command;
}
} // namespace ateam_kenobi::skills

#endif // SKILLS__KICK_HPP_
