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


#ifndef SKILLS__LINE_KICK_HPP_
#define SKILLS__LINE_KICK_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_common/robot_constants.hpp>
#include "kick_skill.hpp"
#include "types/world.hpp"
#include "play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::skills
{

class LineKick : public KickSkill
{
public:
  explicit LineKick(
    stp::Options stp_options,
    KickSkill::WaitType wait_type = KickSkill::WaitType::KickWhenReady);

  void Reset() override
  {
    KickSkill::Reset();
    state_ = State::MoveBehindBall;
  }

  void SetTargetPoint(ateam_geometry::Point point)
  {
    target_point_ = point;
  }

  ateam_geometry::Point GetAssignmentPoint(const World & world);

  ateam_msgs::msg::RobotMotionCommand RunFrame(const World & world, const Robot & robot);

  bool IsDone()
  {
    return state_ == State::Done;
  }

  /**
   * @brief Set the default obstacles planner option on the internal EasyMoveTo
   */
  void SetUseDefaultObstacles(bool use_obstacles)
  {
    path_planning::PlannerOptions options = easy_move_to_.getPlannerOptions();
    options.use_default_obstacles = use_obstacles;
    easy_move_to_.setPlannerOptions(options);
  }

private:
  const double kPreKickOffset = kRobotRadius + kBallRadius + 0.07;
  ateam_geometry::Point target_point_;
  play_helpers::EasyMoveTo easy_move_to_;

  enum class State
  {
    MoveBehindBall,
    FaceBall,
    KickBall,
    Done
  };
  State state_ = State::MoveBehindBall;

  ateam_geometry::Point GetPreKickPosition(const World & world);

  void ChooseState(const World & world, const Robot & robot);

  bool IsRobotBehindBall(const World & world, const Robot & robot, double hysteresis);
  bool IsRobotSettled(const World & world, const Robot & robot);
  bool IsRobotFacingBall(const Robot & robot);
  bool IsBallMoving(const World & world);

  ateam_msgs::msg::RobotMotionCommand RunMoveBehindBall(const World & world, const Robot & robot);
  ateam_msgs::msg::RobotMotionCommand RunFaceBall(const World & world, const Robot & robot);
  ateam_msgs::msg::RobotMotionCommand RunKickBall(const World & world, const Robot & robot);
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__LINE_KICK_HPP_
