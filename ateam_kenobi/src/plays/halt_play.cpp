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

#include "halt_play.hpp"
#include <limits>
#include <ateam_msgs/msg/robot_motion_command.hpp>

namespace ateam_kenobi::plays
{
HaltPlay::HaltPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options)
{
}

stp::PlayScore HaltPlay::getScore(const World & world)
{
  switch (world.referee_info.running_command) {
    case ateam_common::GameCommand::Halt:
    case ateam_common::GameCommand::TimeoutOurs:
    case ateam_common::GameCommand::TimeoutTheirs:
    case ateam_common::GameCommand::GoalOurs:
    case ateam_common::GameCommand::GoalTheirs:
      return stp::PlayScore::Max();
    default:
      return stp::PlayScore::Min();
  }
}

void HaltPlay::reset()
{
}

std::array<std::optional<RobotCommand>, 16> HaltPlay::runFrame(
  const World &)
{
  std::array<std::optional<RobotCommand>, 16> halt_motion_commands;
  RobotCommand command;
  command.motion_intent.linear = motion::intents::linear::VelocityIntent{ateam_geometry::Vector{0.0, 0.0}};
  command.motion_intent.angular = motion::intents::angular::VelocityIntent{0.0};
  for (size_t i = 0; i < 16; ++i) {
    halt_motion_commands[i] = command;
  }
  return halt_motion_commands;
}
}  // namespace ateam_kenobi::plays
