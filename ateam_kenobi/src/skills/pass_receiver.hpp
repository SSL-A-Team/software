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

#ifndef SKILLS__PASS_RECEIVER_HPP_
#define SKILLS__PASS_RECEIVER_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "core/play_helpers/easy_move_to.hpp"
#include "core/stp/skill.hpp"
#include "core/types/world.hpp"

namespace ateam_kenobi::skills
{

class PassReceiver : public stp::Skill
{
public:
  explicit PassReceiver(stp::Options stp_options);

  void reset();

  ateam_msgs::msg::RobotMotionCommand runFrame(const World & world, const Robot & robot);

  ateam_geometry::Point getAssignmentPoint()
  {
    return target_;
  }

  void setTarget(ateam_geometry::Point target)
  {
    target_ = target;
  }

  bool isDone()
  {
    return done_;
  }

  void setExpectedKickSpeed(double val)
  {
    expected_kick_speed_ = val;
  }

private:
  ateam_geometry::Point target_;
  play_helpers::EasyMoveTo easy_move_to_;
  bool done_ = false;
  bool kick_happened_ = false;
  double expected_kick_speed_ = 6.5;

  bool isBallFast(const World & world);
  bool isBallClose(const World & world, const Robot & robot);
  bool isBallStalledAndReachable(const World & world, const Robot & robot);
  bool isBallVelMatchingBotVel(const World & world, const Robot & robot);
  bool isBotCloseToTarget(const Robot & robot);
  bool hasBallBeenKicked(const World & world);

  ateam_msgs::msg::RobotMotionCommand runPrePass(const World & world, const Robot & robot);
  ateam_msgs::msg::RobotMotionCommand runPass(const World & world, const Robot & robot);
  ateam_msgs::msg::RobotMotionCommand runPostPass();
  ateam_msgs::msg::RobotMotionCommand runApproachBall(const World & world, const Robot & robot);
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__PASS_RECEIVER_HPP_
