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


#ifndef PLAYS__TEST_PLAYS__TRIANGLE_PASS_PLAY_HPP_
#define PLAYS__TEST_PLAYS__TRIANGLE_PASS_PLAY_HPP_

#include <vector>
#include "core/stp/play.hpp"
#include "core/play_helpers/easy_move_to.hpp"
#include "tactics/pass.hpp"

namespace ateam_kenobi::plays
{

class TrianglePassPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "TrianglePassPlay";

  explicit TrianglePassPlay(stp::Options stp_options);

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

private:
  static constexpr double kKickSpeed = 3.0;
  tactics::Pass pass_tactic_;
  std::array<play_helpers::EasyMoveTo, 16> easy_move_tos_;
  std::vector<ateam_geometry::Point> positions;
  std::size_t kick_target_ind_ = 0;

  enum class State
  {
    Setup,
    Passing,
    BackOff
  } state_ = State::Setup;

  bool isReady(const World & world);

  void runSetup(
    const std::vector<Robot> & robots,
    const World & world, std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);

  void runPassing(
    const std::vector<Robot> & robots,
    const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);

  void runBackOff(
    const std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__TEST_PLAYS__TRIANGLE_PASS_PLAY_HPP_
