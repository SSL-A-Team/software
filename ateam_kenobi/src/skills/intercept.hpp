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


#ifndef SKILLS__INTERCEPT_HPP_
#define SKILLS__INTERCEPT_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "core/play_helpers/easy_move_to.hpp"
#include <ateam_common/robot_constants.hpp>
#include "core/stp/skill.hpp"
#include "core/types/state_types.hpp"


namespace ateam_kenobi::skills
{

class Intercept : public stp::Skill
{
public:
  explicit Intercept(stp::Options stp_options);

  void Reset();

  ateam_geometry::Point getAssignmentPoint(const World & world)
  {
    return calculateEstimateInterceptPointnt(world);
  }

  bool isDone()
  {
    return done_;
  }


  ateam_msgs::msg::RobotMotionCommand runFrame(const World & world, const Robot & robot);

private:
  play_helpers::EasyMoveTo easy_move_to_;
  bool done_ = false;

  enum class State
  {
    MoveToBall,
    Intercept
  };
  State state_ = State::MoveToBall;

  void chooseState(const World & world, const Robot & robot);

  ateam_geometry::Point calculateEstimateInterceptPoint(const World & world);
  std::optional<ateam_geometry::Point> calculateInterceptPoint(
    const World & world,
    const Robot & robot);

  ateam_msgs::msg::RobotMotionCommand runMoveToBall(const World & world, const Robot & robot);
  ateam_msgs::msg::RobotMotionCommand runIntercept(const World & world, const Robot & robot);
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__INTERCEPT_HPP_
