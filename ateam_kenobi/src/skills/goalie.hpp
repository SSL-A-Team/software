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
#include "core/types/state_types.hpp"
#include "core/types/robot_command.hpp"
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

  void runFrame(const World & world, std::array<std::optional<RobotCommand>, 16> & motion_commands);

private:
  LineKick kick_;
  std::optional<int> last_enemy_id_closest_to_ball_;
  std::chrono::steady_clock::time_point ball_entered_def_area_time_;
  bool prev_ball_in_def_area_ = false;
  path_planning::PlannerOptions default_planner_options_;

  bool doesOpponentHavePossesion(const World & world);
  bool isBallHeadedTowardsGoal(const World & world, const Ball & ball_state);
  bool isBallInDefenseArea(const World & world, const Ball & ball_state);

  /**
   * @brief Default behavior of robot staying in line with ball
   * @return RobotCommand
   */
  RobotCommand runDefaultBehavior(
    const World & world, const Robot & goalie,
    const Ball & ball_state);

  /**
   * @brief Block a possible shot when opponents have the ball
   * @return RobotCommand
   */
  RobotCommand runBlockShot(
    const World & world, const Robot & goalie,
    const Ball & ball_state);

  /**
   * @brief Block ball when headed towards goal
   * @return RobotCommand
   */
  RobotCommand runBlockBall(
    const World & world, const Robot & goalie,
    const Ball & ball_state);

  /**
   * @brief Kick ball out of defense area
   * @return RobotCommand
   */
  RobotCommand runClearBall(
    const World & world, const Robot & goalie,
    const Ball & ball_state);

  RobotCommand runSideEjectBall(const World & world, const Robot & goalie);

  std::vector<ateam_geometry::AnyShape> getCustomObstacles(const World & world);
  std::optional<Robot> getClosestEnemyRobotToBall(const World & world);
  void glueBallToLastClosestEnemy(Ball & ball, const World & world);
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__GOALIE_HPP_
