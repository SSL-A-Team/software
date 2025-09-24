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

#include "standard_defense.hpp"
#include <vector>

namespace ateam_kenobi::tactics
{

StandardDefense::StandardDefense(stp::Options stp_options)
: stp::Tactic(stp_options),
  goalie_(createChild<skills::Goalie>("goalie")),
  defenders_(createChild<tactics::Defenders>("defenders"))
{}

void StandardDefense::reset()
{
  goalie_.reset();
}

std::vector<ateam_geometry::Point> StandardDefense::getAssignmentPoints(const World & world)
{
  return defenders_.getAssignmentPoints(world);
}

void StandardDefense::runFrame(
  const World & world,
  const std::vector<Robot> & defender_bots,
  std::array<std::optional<RobotCommand>, 16> & motion_commands)
{
  goalie_.runFrame(world, motion_commands);
  ForwardPlayInfo(goalie_);
  defenders_.runFrame(world, defender_bots, motion_commands);
  ForwardPlayInfo(defenders_);
}

}  // namespace ateam_kenobi::tactics
