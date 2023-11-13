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
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "visualization/overlay_publisher.hpp"
#include "visualization/play_info_publisher.hpp"
#include "types/world.hpp"

namespace ateam_kenobi::plays
{

class BasePlay
{
public:
  explicit BasePlay(
    visualization::OverlayPublisher & overlay_publisher,
    visualization::PlayInfoPublisher & play_info_publisher)
  : overlay_publisher_(overlay_publisher), play_info_publisher_(play_info_publisher) {}

  virtual ~BasePlay() = default;

  virtual void reset() = 0;

  virtual std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(
    const World & world) = 0;

protected:
  visualization::OverlayPublisher & overlay_publisher_;
  visualization::PlayInfoPublisher & play_info_publisher_;
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__BASE_PLAY_HPP_
