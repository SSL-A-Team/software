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


#ifndef CORE__FPS_TRACKER_HPP_
#define CORE__FPS_TRACKER_HPP_

#include <ateam_common/time.hpp>
#include "core/types.hpp"

namespace ateam_kenobi
{

class FpsTracker
{
public:
  FpsTracker()
  {
    std::fill(buffer_.begin(), buffer_.end(), average_fps_ / kWindowWidth);
  }

  double Update(const World & world)
  {
    const auto dt = ateam_common::TimeDiffSeconds(world.current_time, prev_time_);
    prev_time_ = world.current_time;
    const auto current_fps = 1.0 / dt;
    average_fps_ -= buffer_[buffer_index_];
    buffer_[buffer_index_] = current_fps / kWindowWidth;
    average_fps_ += buffer_[buffer_index_];
    buffer_index_ = (buffer_index_ + 1) % kWindowWidth;
    return average_fps_;
  }

private:
  static constexpr size_t kWindowWidth = 100;
  std::chrono::steady_clock::time_point prev_time_;
  std::array<double, kWindowWidth> buffer_;
  size_t buffer_index_ = 0;
  double average_fps_ = 100.0;  // Assume everything's fine at the start
};

}  // namespace ateam_kenobi

#endif  // CORE__FPS_TRACKER_HPP_
