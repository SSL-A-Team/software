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

#ifndef SKILLS__GOALIE_HPP_
#define SKILLS__GOALIE_HPP_

#include <ateam_geometry/types.hpp>
#include <ateam_geometry/nearest_points.hpp>
#include "play_helpers/easy_move_to.hpp"
#include "types/world.hpp"
#include "visualization/overlays.hpp"

namespace ateam_kenobi::skills
{
class Goalie
{
public:
  explicit Goalie(visualization::Overlays overlays);

  void reset();

  void runFrame(
    const World & world, std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);

private:
  visualization::Overlays overlays_;
  play_helpers::EasyMoveTo easy_move_to_;
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__GOALIE_HPP_
