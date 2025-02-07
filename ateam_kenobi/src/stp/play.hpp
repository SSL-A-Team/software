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


#ifndef STP__PLAY_HPP_
#define STP__PLAY_HPP_

#include <array>
#include <optional>
#include <string>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "base.hpp"
#include "play_score.hpp"
#include "types/world.hpp"

namespace ateam_kenobi::stp
{

enum class PlayCompletionState
{
  NotApplicable,
  Done,
  Busy
};

class Play : public Base
{
public:
  Play(std::string name, Options options)
  : Base(name, options) {}

  virtual ~Play() = default;

  /**
   * @brief Get the play's validity / confidence score
   *
   * Plays should override this with logic that checks the game state and returns a number representing if the play should be run or not.
   *
   * The play selector will prefer plays with a higher score.
   *
   * If getScore() returns NaN, the play will never be executed unless specified via play override
   *
   * @return PlayScore
   */
  virtual PlayScore getScore(const World &)
  {
    return PlayScore::NaN();
  }

  /**
   * @brief Returns the completion state of the robot
   *
   * Should only be used by plays when interrupting them would be bad (ie. passing)
   */
  virtual PlayCompletionState getCompletionState()
  {
    return PlayCompletionState::NotApplicable;
  }

  /**
   * @deprecated Use @c enter and @c exit instead.
   */
  virtual void reset() {}

  /**
   * @brief Called before a play is run when the previous frame was running a different frame.
   * 
   * Also called if a play is being chosen again after reporting itself done in @c getCompletionState.
   */
  virtual void enter() {}

  /**
   * @brief Called when a new play is being run and this play was running in the previous frame.
   * 
   * Also called if a play is being chosen again after reporting itself done in @c getCompletionState.
   */
  virtual void exit() {}

  virtual std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(
    const World & world) = 0;

  bool isEnabled() const
  {
    return enabled_;
  }

  void setEnabled(bool value)
  {
    enabled_ = value;
  }

private:
  bool enabled_ = true;
};

}  // namespace ateam_kenobi::stp

#endif  // STP__PLAY_HPP_
