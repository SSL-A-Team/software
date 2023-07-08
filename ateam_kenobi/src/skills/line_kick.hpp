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
#include "visualization/overlay_publisher.hpp"
#include "types/world.hpp"
#include "play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::skills
{

class LineKick
{
public:
  LineKick(visualization::OverlayPublisher & overlay_publisher);

  void setTargetPoint(ateam_geometry::Point point) {
    target_point_ = point;
  }

  ateam_geometry::Point getAssignmentPoint(const World & world);

  ateam_msgs::msg::RobotMotionCommand runFrame(const World & world, const Robot & robot);

private:
  const double kPreKickOffset = kRobotRadius + 0.1;
  visualization::OverlayPublisher & overlay_publisher_;
  ateam_geometry::Point target_point_;
  play_helpers::EasyMoveTo easy_move_to_;

  enum class State {
    MoveToPreKick,
    FaceBall,
    KickBall
  };
  State prev_state_ = State::MoveToPreKick;

  ateam_geometry::Point getPreKickPosition(const World & world);

  ateam_msgs::msg::RobotMotionCommand moveToPreKick(const World & world, const Robot & robot);

  ateam_msgs::msg::RobotMotionCommand faceBall(const World & world, const Robot & robot);

  ateam_msgs::msg::RobotMotionCommand kickBall(const World & world, const Robot & robot);
};

}

#endif  // SKILLS__LINE_KICK_HPP_
