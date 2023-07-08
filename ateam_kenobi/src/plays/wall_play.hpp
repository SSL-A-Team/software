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

#ifndef PLAYS__WALL_PLAY_HPP_
#define PLAYS__WALL_PLAY_HPP_

#include <vector>
#include <optional>
#include <array>

#include "ateam_geometry/types.hpp"
#include "types/robot.hpp"
#include "base_play.hpp"
#include "play_helpers/easy_move_to.hpp"
#include "skills/goalie.hpp"

namespace ateam_kenobi::plays
{
inline std::vector<ateam_geometry::Point> get_equally_spaced_points_on_segment(
  ateam_geometry::Segment & segment, int num_points);

class WallPlay : public BasePlay
{
public:
  explicit WallPlay(
    visualization::OverlayPublisher & overlay_publisher,
    visualization::PlayInfoPublisher & play_info_publisher);
  void reset() override;
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

private:
  std::array<play_helpers::EasyMoveTo, 16> easy_move_tos_;
  skills::Goalie goalie_skill_;
};
} // namespace ateam_kenobi::plays

#endif // PLAYS__WALL_PLAY_HPP_
