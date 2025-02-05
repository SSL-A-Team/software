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


#ifndef SKILLS__PIVOT_KICK_HPP_
#define SKILLS__PIVOT_KICK_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_common/robot_constants.hpp>
#include "kick_skill.hpp"
#include "types/world.hpp"
#include "play_helpers/easy_move_to.hpp"
#include "skills/capture.hpp"

namespace ateam_kenobi::skills
{

class PivotKick : public KickSkill
{
public:
  explicit PivotKick(
    stp::Options stp_options,
    KickSkill::WaitType wait_type = KickSkill::WaitType::KickWhenReady);

  void Reset() override
  {
    KickSkill::Reset();
    capture_.Reset();
    prev_state_ = State::Capture;
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
    capture_.SetUseDefaultObstacles(use_obstacles);
  }

  void SetCaptureSpeed(double speed)
  {
    capture_.SetCaptureSpeed(speed);
  }

  void SetPivotSpeed(double speed)
  {
    pivot_speed_ = speed;
  }

private:
  const double kPreKickOffset = kRobotRadius + 0.1;
  ateam_geometry::Point target_point_;
  play_helpers::EasyMoveTo easy_move_to_;
  skills::Capture capture_;
  bool done_ = false;
  double pivot_speed_ = 2.0;  // rad/s

  enum class State
  {
    Capture,
    Pivot,
    KickBall
  };
  State prev_state_ = State::Capture;

  ateam_msgs::msg::RobotMotionCommand Capture(const World & world, const Robot & robot);

  ateam_msgs::msg::RobotMotionCommand Pivot(const Robot & robot);

  ateam_msgs::msg::RobotMotionCommand KickBall(const World & world, const Robot & robot);
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__PIVOT_KICK_HPP_
