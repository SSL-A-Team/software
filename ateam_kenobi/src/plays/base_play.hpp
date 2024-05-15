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


#ifndef PLAYS__BASE_PLAY_HPP_
#define PLAYS__BASE_PLAY_HPP_

#include <array>
#include <optional>
#include <string>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "visualization/overlays.hpp"
#include <nlohmann/json.hpp>
#include "types/world.hpp"

namespace ateam_kenobi::plays
{

class BasePlay
{
public:
  explicit BasePlay(std::string play_name)
  : play_name_(play_name), overlays_(play_name) {}

  virtual ~BasePlay() = default;

  /**
   * @brief Get the play's validity / confidence score
   * 
   * Plays should override this with logic that checks the game state and returns a number representing if the play should be run or not.
   * 
   * The play selector will prefer plays with a higher score.
   * 
   * If getScore() returns NaN, the play will never be executed unless specified via play override
   * 
   * @return double 
   */
  virtual double getScore(const World &) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  virtual void reset() = 0;

  virtual std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(
    const World & world) = 0;

  const std::string & getName() const
  {
    return play_name_;
  }

  visualization::Overlays & getOverlays()
  {
    return overlays_;
  }

  nlohmann::json & getPlayInfo()
  {
    return play_info_;
  }

  bool isEnabled() const {
    return enabled_;
  }

  void setEnabled(bool value) {
    enabled_ = value;
  }

protected:
  std::string play_name_;
  visualization::Overlays overlays_;
  nlohmann::json play_info_;
  bool enabled_ = true;
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__BASE_PLAY_HPP_
