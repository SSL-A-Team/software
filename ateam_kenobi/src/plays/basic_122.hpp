// Copyright 2021 A Team
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


#ifndef PLAYS__BASIC_122_HPP_
#define PLAYS__BASIC_122_HPP_

#include <vector>
#include "base_play.hpp"
#include "skills/line_kick.hpp"
#include "skills/blockers.hpp"
#include "skills/goalie.hpp"

namespace ateam_kenobi::plays
{

class Basic122 : public BasePlay
{
public:
  Basic122();

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(const World & world);

private:
  skills::LineKick striker_skill_;
  skills::Blockers blockers_skill_;
  skills::Goalie goalie_skill_;

  void assignAndRunStriker(
    std::vector<Robot> available_robots, const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);

  void assignAndRunBlockers(
    std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__BASIC_122_HPP_
