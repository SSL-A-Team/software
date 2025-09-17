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

#ifndef TACTICS__STANDARD_DEFENSE_HPP_
#define TACTICS__STANDARD_DEFENSE_HPP_

#include <vector>
#include "core/types/world.hpp"
#include "core/types/robot.hpp"
#include "core/types/robot_command.hpp"
#include "core/stp/tactic.hpp"
#include "defenders.hpp"
#include "skills/goalie.hpp"

namespace ateam_kenobi::tactics
{

class StandardDefense : public stp::Tactic
{
public:
  explicit StandardDefense(stp::Options stp_options);

  void reset();

  /**
   * @note Goalie not included in assignment points b/c it is assigned by ID
   */
  std::vector<ateam_geometry::Point> getAssignmentPoints(const World & world);

  /**
   * @note Goalie not included in defender bots. It is chosen by ID.
   */
  void runFrame(
    const World & world,
    const std::vector<Robot> & defender_bots,
    std::array<std::optional<RobotCommand>, 16> & motion_commands);

private:
  skills::Goalie goalie_;
  tactics::Defenders defenders_;
};

}  // namespace ateam_kenobi::tactics

#endif  // TACTICS__STANDARD_DEFENSE_HPP_
