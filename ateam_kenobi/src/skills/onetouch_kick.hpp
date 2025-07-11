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


#ifndef SKILLS__ONETOUCH_KICK_HPP_
#define SKILLS__ONETOUCH_KICK_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_common/robot_constants.hpp>
#include "kick_skill.hpp"
#include "core/types/world.hpp"
#include "core/play_helpers/easy_move_to.hpp"
#include "core/play_helpers/intercept_calculation.hpp"

namespace ateam_kenobi::skills
{

class OnetouchKick : public KickSkill
{
public:
  enum class KickType
  {
    Kick,
    Chip
  };

  explicit OnetouchKick(
    stp::Options stp_options,
    KickSkill::WaitType wait_type = KickSkill::WaitType::KickWhenReady);

  void Reset() override
  {
    KickSkill::Reset();
    prev_state_ = State::Wait;
    done_ = false;
  }

  void SetTargetPoint(ateam_geometry::Point point)
  {
    target_point_ = point;
  }

  ateam_geometry::Point GetAssignmentPoint(const World & world);

  ateam_msgs::msg::RobotMotionCommand RunFrame(const World & world, const Robot & robot);

  bool IsDone() const
  {
    return done_;
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

  void SetKickType(KickType type)
  {
    kick_type_ = type;
  }

private:
  KickType kick_type_ = KickType::Kick;
  ateam_geometry::Point prep_point_;
  ateam_geometry::Point target_point_;
  ateam_geometry::Point TEST_point_; // REMOVE THIS
  play_helpers::EasyMoveTo easy_move_to_;
  play_helpers::InterceptResults intercept_result_;

  bool done_ = false;

  enum class State
  {
    Wait,
    MoveToPrepPoint,
    InterceptKick
  };
  State prev_state_ = State::Wait;

  ateam_msgs::msg::RobotMotionCommand Wait(const World & world, const Robot & robot);

  ateam_msgs::msg::RobotMotionCommand MoveToPrepPoint(const World & world, const Robot & robot);

  ateam_msgs::msg::RobotMotionCommand InterceptKick(const World & world, const Robot & robot);
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__ONETOUCH_KICK_HPP_
