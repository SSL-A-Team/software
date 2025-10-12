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


#ifndef SKILLS__CAPTURE_HPP_
#define SKILLS__CAPTURE_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_common/robot_constants.hpp>
#include "core/stp/skill.hpp"
#include "core/types.hpp"
#include "core/types/robot_command.hpp"


namespace ateam_kenobi::skills
{

class Capture : public stp::Skill
{
public:
  explicit Capture(stp::Options stp_options);

  void Reset();

  ateam_geometry::Point getAssignmentPoint(const World & world)
  {
    return world.ball.pos;
  }

  bool isDone()
  {
    return done_;
  }


  RobotCommand runFrame(const World & world, const Robot & robot);

  /**
   * @brief Set the capture speed used to approach the ball in the final phase
   *
   * @param speed Speed in meters per second
   */
  void SetCaptureSpeed(double speed)
  {
    capture_speed_ = speed;
  }

private:
  bool done_ = false;
  int ball_detected_filter_ = 0;
  double approach_radius_ = 0.3;  // m
  double capture_speed_ = 0.15;  // m/s
  double max_speed_ = 2.0;  // m/s
  double decel_limit_ = 3.0;  // m/s/s

  enum class State
  {
    MoveToBall,
    Capture
  };
  State state_ = State::MoveToBall;

  void chooseState(const World & world, const Robot & robot);

  RobotCommand runMoveToBall(const World & world, const Robot & robot);
  RobotCommand runCapture(const World & world, const Robot & robot);
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__CAPTURE_HPP_
