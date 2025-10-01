// Copyright 2025 A Team
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


#ifndef CORE__BALLSENSE_FILTER_HPP_
#define CORE__BALLSENSE_FILTER_HPP_

#include <ranges>
#include <array>
#include "core/types.hpp"

namespace ateam_kenobi
{

class BallSenseFilter
{
public:
  BallSenseFilter()
  {
    for (auto & buffer : filter_buffers_) {
      std::ranges::fill(buffer, 0);
    }
    std::ranges::fill(filter_sums_, 0);
  }

  void Update(World & world)
  {
    std::ranges::for_each(
      world.our_robots, [this](Robot & robot) {
        UpdateFilter(robot);
      });
    buffer_index_ = (buffer_index_ + 1) % kFilterWidth;
  }

private:
  static constexpr size_t kFilterWidth = 30;
  static constexpr size_t kFilterThreshold = 20;

  std::array<std::array<uint8_t, kFilterWidth>, 16> filter_buffers_;
  std::array<size_t, 16> filter_sums_;
  size_t buffer_index_ = 0;

  void UpdateFilter(Robot & robot)
  {
    auto & buffer = filter_buffers_[robot.id];
    auto & sum = filter_sums_[robot.id];
    const auto raw_detect = robot.breakbeam_ball_detected;
    const auto outgoing = buffer[buffer_index_];
    sum -= outgoing;
    if (robot.radio_connected) {
      if (raw_detect) {
        buffer[buffer_index_] = 1;
        sum++;
      } else {
        buffer[buffer_index_] = 0;
      }
      robot.breakbeam_ball_detected_filtered = (sum >= kFilterThreshold);
    } else {
      robot.breakbeam_ball_detected_filtered = false;
      buffer[buffer_index_] = 0;
    }
  }
};

}  // namespace ateam_kenobi

#endif  // CORE__BALLSENSE_FILTER_HPP_
