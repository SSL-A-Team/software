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


#ifndef PLAYS__TEST_PLAYS__TEST_WINDOW_EVAL_HPP_
#define PLAYS__TEST_PLAYS__TEST_WINDOW_EVAL_HPP_

#include "core/stp/play.hpp"
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/window_evaluation.hpp"

namespace ateam_kenobi::plays
{

class TestWindowEvalPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "TestWindowEvalPlay";

  explicit TestWindowEvalPlay(stp::Options stp_options)
  : stp::Play(kPlayName, stp_options)
  {
  }

  void reset() override
  {
  }

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override
  {
    const auto source = world.ball.pos;

    const ateam_geometry::Segment target{
      ateam_geometry::Point{world.field.field_length / 2.0, -world.field.goal_width},
      ateam_geometry::Point{world.field.field_length / 2.0, world.field.goal_width}
    };

    const auto opponent_robots = play_helpers::getVisibleRobots(world.their_robots);

    const auto windows =
      play_helpers::window_evaluation::getWindows(target, source, opponent_robots);

    play_helpers::window_evaluation::drawWindows(windows, source, getOverlays());

    return {};
  }

private:
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__TEST_PLAYS__TEST_WINDOW_EVAL_HPP_
