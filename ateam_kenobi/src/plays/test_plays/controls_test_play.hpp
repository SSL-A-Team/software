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

#ifndef PLAYS__TEST_PLAYS__CONTROLS_TEST_PLAY_HPP_
#define PLAYS__TEST_PLAYS__CONTROLS_TEST_PLAY_HPP_

#include <array>
#include <vector>
#include "core/motion/motion_controller.hpp"
#include "core/stp/play.hpp"
#include "ateam_geometry/types.hpp"

namespace ateam_kenobi::plays
{
class ControlsTestPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "ControlsTestPlay";

  explicit ControlsTestPlay(stp::Options stp_options);

  void reset() override;

  std::array<std::optional<RobotCommand>,
    16> runFrame(const World & world) override;

private:
  struct Waypoint
  {
    ateam_geometry::Point position;
    motion::AngleMode angle_mode;
    double heading;
    double hold_time_sec;
  };

  motion::MotionController motion_controller_;
  motion::MotionOptions motion_options_;

  int index = 0;
  std::vector<Waypoint> waypoints;
  bool goal_hit;
  std::chrono::steady_clock::time_point goal_hit_time;
  double position_threshold = 0.01;
  double angle_threshold = 8.0;

  bool isGoalHit(const Robot & robot);
};
}  // namespace ateam_kenobi::plays
#endif  // PLAYS__TEST_PLAYS__CONTROLS_TEST_PLAY_HPP_
