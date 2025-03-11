// Copyright 2024 A Team
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

#include "pass_to_segment.hpp"
#include "core/play_helpers/window_evaluation.hpp"
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::tactics
{

PassToSegment::PassToSegment(stp::Options stp_options)
: stp::Tactic(stp_options),
  pass_tactic_(createChild<Pass>("Pass"))
{}

void PassToSegment::reset()
{
  pass_tactic_.reset();
}

ateam_geometry::Point PassToSegment::getKickerAssignmentPoint(const World & world)
{
  return pass_tactic_.getKickerAssignmentPoint(world);
}

ateam_geometry::Point PassToSegment::getReceiverAssignmentPoint(const World & world)
{
  return getTargetPointOnSegment(world);
}

void PassToSegment::runFrame(
  const World & world, const Robot & kicker_bot, const Robot & receiver_bot,
  ateam_msgs::msg::RobotMotionCommand & kicker_command,
  ateam_msgs::msg::RobotMotionCommand & receiver_command)
{
  if (ateam_geometry::norm(world.ball.vel) < 0.01) {
    pass_tactic_.setTarget(getTargetPointOnSegment(world));
  }
  pass_tactic_.runFrame(world, kicker_bot, receiver_bot, kicker_command, receiver_command);
}


ateam_geometry::Point PassToSegment::getTargetPointOnSegment(const World & world)
{
  const auto windows = play_helpers::window_evaluation::getWindows(
    target_, world.ball.pos,
    play_helpers::getVisibleRobots(world.their_robots));

  const auto largest_window = play_helpers::window_evaluation::getLargestWindow(windows);

  if (!largest_window) {
    // If no windows exist, fallback to target segment midpoint
    return CGAL::midpoint(target_);
  }

  return CGAL::midpoint(*largest_window);
}

}  // namespace ateam_kenobi::tactics
