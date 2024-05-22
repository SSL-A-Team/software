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


#ifndef TYPES__ROBOT_HPP_
#define TYPES__ROBOT_HPP_

#include <ateam_geometry/types.hpp>

namespace ateam_kenobi
{
struct Robot
{
  int id;
  bool visible = false;
  bool radio_connected = false;

  ateam_geometry::Point pos;
  double theta;
  ateam_geometry::Vector vel;
  double omega;

  bool breakbeam_ball_detected = false;

  // I don't know if these are actually in the radio packet
  // but might as well handle them now
  bool kicker_available = true;
  bool chipper_available = false;

  bool IsAvailable() const
  {
    return visible && radio_connected;
  }
};
}  // namespace ateam_kenobi

#endif  // TYPES__ROBOT_HPP_
