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

#include <vector>
#include <ateam_geometry/types.hpp>
#include "core/play_helpers/easy_move_to.hpp"
#include "core/types/world.hpp"
#include "core/stp/skill.hpp"
#include "pivot_kick.hpp"
#include "line_kick.hpp"

namespace ateam_kenobi::skills
{
class Goalie : public stp::Skill
{
public:
  explicit Goalie(stp::Options stp_options);

  void reset();

  void runFrame(
    const World & world, std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);

private:
  play_helpers::EasyMoveTo easy_move_to_;
  LineKick kick_;

  bool doesOpponentHavePossesion(const World & world);
  bool isBallHeadedTowardsGoal(const World & world);
  bool isBallInDefenseArea(const World & world);

  /**
   * @brief Default behavior of robot staying in line with ball
   * @return ateam_msgs::msg::RobotMotionCommand
   */
  ateam_msgs::msg::RobotMotionCommand runDefaultBehavior(const World & world, const Robot & goalie);

  /**
   * @brief Block a possible shot when opponents have the ball
   * @return ateam_msgs::msg::RobotMotionCommand
   */
  ateam_msgs::msg::RobotMotionCommand runBlockShot(const World & world, const Robot & goalie);

  /**
   * @brief Block ball when headed towards goal
   * @return ateam_msgs::msg::RobotMotionCommand
   */
  ateam_msgs::msg::RobotMotionCommand runBlockBall(const World & world, const Robot & goalie);

  /**
   * @brief Kick ball out of defense area
   * @return ateam_msgs::msg::RobotMotionCommand
   */
  ateam_msgs::msg::RobotMotionCommand runClearBall(const World & world, const Robot & goalie);

  std::vector<ateam_geometry::AnyShape> getCustomObstacles(const World & world);
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__GOALIE_HPP_
