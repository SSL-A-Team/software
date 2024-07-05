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


#ifndef STP__PLAY_SCORE_HPP_
#define STP__PLAY_SCORE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace ateam_kenobi::stp
{

/**
 * @brief Holds bounded score values for picking plays
 *
 * Allows values in the range 0-100, infinity, negative infinity, and NaN
 */
class PlayScore
{
public:
  PlayScore() = default;

  PlayScore(double value)  // NOLINT(runtime/explicit)
  {
    SetValue(value);
  }

  operator double() const {return value_;}

  PlayScore & operator=(double new_value)
  {
    SetValue(new_value);
    return *this;
  }

  void SetValue(double new_value)
  {
    if (std::isnan(new_value) || std::isinf(new_value)) {
      value_ = new_value;
      return;
    }
    value_ = std::clamp(new_value, kMinAllowed, kMaxAllowed);
  }

  static PlayScore Infinity()
  {
    return kInfinity;
  }

  static PlayScore NegativeInfinity()
  {
    return -kInfinity;
  }

  static PlayScore NaN()
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  static PlayScore Max()
  {
    return kMaxAllowed;
  }

  static PlayScore Min()
  {
    return kMinAllowed;
  }

  auto operator<=>(const PlayScore &) const = default;

private:
  static constexpr double kMaxAllowed = 100.0;
  static constexpr double kMinAllowed = 0.0;
  static constexpr double kInfinity = std::numeric_limits<double>::infinity();

  double value_ = kInfinity;
};

}  // namespace ateam_kenobi::stp

#endif  // STP__PLAY_SCORE_HPP_
