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


#ifndef TACTICS__PASS_HPP_
#define TACTICS__PASS_HPP_

#include <ateam_geometry/types.hpp>
#include "stp/tactic.hpp"
#include "types/world.hpp"
#include "skills/line_kick.hpp"
#include "skills/pass_receiver.hpp"

namespace ateam_kenobi::tactics
{

class Pass : public stp::Tactic
{
public:
  explicit Pass(stp::Options stp_options);

  void reset();

  ateam_geometry::Point getKickerAssignmentPoint(const World & world);

  ateam_geometry::Point getReceiverAssignmentPoint();

  void runFrame(
    const World & world, const Robot & kicker_bot, const Robot & receiver_bot,
    ateam_msgs::msg::RobotMotionCommand & kicker_command,
    ateam_msgs::msg::RobotMotionCommand & receiver_command);

  bool isDone();

  void setTarget(ateam_geometry::Point target)
  {
    target_ = target;
    receiver_.setTarget(target);
    kick_.SetTargetPoint(target);
  }

  void setKickSpeed(double speed)
  {
    speed_ = speed;
  }

private:
  const double kReceiverPositionThreshold = 0.1;
  std::optional<double> speed_;
  ateam_geometry::Point target_;
  skills::PassReceiver receiver_;
  skills::LineKick kick_;

  double calculateDefaultKickSpeed(const World & world);
};

}  // namespace ateam_kenobi::tactics

#endif  // TACTICS__PASS_HPP_
