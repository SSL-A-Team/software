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

#ifndef TACTICS__DEFENDERS_HPP_
#define TACTICS__DEFENDERS_HPP_

#include <array>
#include <vector>
#include <ateam_geometry/types.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "core/types.hpp"
#include "core/stp/tactic.hpp"
#include "core/play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::tactics
{

class Defenders : public stp::Tactic
{
public:
  explicit Defenders(stp::Options stp_options);

  void reset();

  std::vector<ateam_geometry::Point> getAssignmentPoints(const World & world);

  void runFrame(
    const World & world,
    const std::vector<Robot> & robots,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & motion_commands);

private:
  static constexpr double kMargin = 0.05;
  static constexpr double kDefenseSegmentOffset = kRobotRadius + kMargin;
  std::array<play_helpers::EasyMoveTo, 16> easy_move_tos_;

  std::vector<ateam_geometry::Point> getDefenderPoints(const World & world);

  ateam_geometry::Point getBallBlockPoint(const World & world);

  ateam_geometry::Point getPassBlockPoint(const World & world);

  ateam_geometry::Point getAdjacentBlockPoint(
    const World & world,
    const ateam_geometry::Point & other_block_point);

  std::vector<ateam_geometry::Segment> getDefenseSegments(const World & world);

  void drawDefenseSegments(const World & world);

  bool isBallInDefenseArea(const World & world);
};

}  // namespace ateam_kenobi::tactics

#endif  // TACTICS__DEFENDERS_HPP_
