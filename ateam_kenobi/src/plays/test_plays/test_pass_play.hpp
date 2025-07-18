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

#ifndef PLAYS__TEST_PLAYS__TEST_PASS_PLAY_HPP_
#define PLAYS__TEST_PLAYS__TEST_PASS_PLAY_HPP_

#include <vector>
#include "core/stp/play.hpp"
#include "tactics/pass.hpp"

namespace ateam_kenobi::plays
{
class TestPassPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "TestPassPlay";

  explicit TestPassPlay(stp::Options stp_options);

  void enter() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

private:
  tactics::Pass pass_tactic_;
  std::vector<ateam_geometry::Point> targets_;
  size_t target_ind_ = 0;
  bool prev_done_ = false;
  bool first_frame_ = true;
  std::chrono::steady_clock::time_point done_time_;
};
}  // namespace ateam_kenobi::plays
#endif  // PLAYS__TEST_PLAYS__TEST_PASS_PLAY_HPP_
