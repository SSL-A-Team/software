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

#include "lane_idler.hpp"
#include "core/play_helpers/window_evaluation.hpp"
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/lanes.hpp"

namespace ateam_kenobi::skills
{

LaneIdler::LaneIdler(stp::Options stp_options)
: stp::Skill(stp_options),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo"))
{}

void LaneIdler::Reset()
{
  easy_move_to_.reset();
}

ateam_geometry::Point LaneIdler::GetAssignmentPoint(const World & world)
{
  return GetIdlingPosition(world);
}

ateam_msgs::msg::RobotMotionCommand LaneIdler::RunFrame(const World & world, const Robot & robot)
{
  easy_move_to_.setTargetPosition(GetIdlingPosition(world));
  easy_move_to_.face_point(world.ball.pos);
  return easy_move_to_.runFrame(robot, world, extra_obstacles_);
}

ateam_geometry::Point LaneIdler::GetIdlingPosition(const World & world)
{
  const auto lane_segment = play_helpers::lanes::GetLaneLongitudinalMidSegment(world, lane_);

  const auto windows = play_helpers::window_evaluation::getWindows(
    lane_segment, world.ball.pos, play_helpers::getVisibleRobots(
      world.their_robots));

  const auto largest_window = play_helpers::window_evaluation::getLargestWindow(windows);

  if (!largest_window) {
    // Fallback to lane midpoint. Might collide, but path planning will handle that.
    return CGAL::midpoint(lane_segment);
  }

  return ateam_geometry::nearestPointOnSegment(*largest_window, world.ball.pos);
}

}  // namespace ateam_kenobi::skills
