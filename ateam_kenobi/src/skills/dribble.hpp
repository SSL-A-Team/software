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


#ifndef SKILLS__DRIBBLE_HPP_
#define SKILLS__DRIBBLE_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_common/robot_constants.hpp>
#include "visualization/overlays.hpp"
#include "types/world.hpp"
#include "play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::skills
{

class Dribble
{
public:
  explicit Dribble(visualization::Overlays overlays);

  void setTargetPoint(ateam_geometry::Point point)
  {
    target_point_ = point;
  }

  ateam_geometry::Point getAssignmentPoint(const World & world);

  ateam_msgs::msg::RobotMotionCommand runFrame(const World & world, const Robot & robot);

private:
  const double kOffset = kRobotRadius + kBallRadius + 0.07;
  visualization::Overlays overlays_;
  ateam_geometry::Point target_point_;
  play_helpers::EasyMoveTo easy_move_to_;

  enum class State
  {
    MoveBehindBall,
    Dribble
  };
  State state_ = State::MoveBehindBall;

  ateam_geometry::Point getStartPosition(const World & world);

  void chooseState(const World & world, const Robot & robot);

  bool isRobotBehindBall(const World & world, const Robot & robot, double hysteresis);
  bool isRobotSettled(const World & world, const Robot & robot);
  bool robotHasBall(const World & world, const Robot & robot);

  ateam_msgs::msg::RobotMotionCommand runMoveBehindBall(const World & world, const Robot & robot);
  ateam_msgs::msg::RobotMotionCommand runDribble(const World & world, const Robot & robot);
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__DRIBBLE_HPP_
