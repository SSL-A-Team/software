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

#ifndef CORE__PLAY_HELPERS__LANES_HPP_
#define CORE__PLAY_HELPERS__LANES_HPP_

#include <ateam_geometry/types.hpp>
#include "core/types/state_types.hpp"

namespace ateam_kenobi::play_helpers::lanes
{

enum class Lane
{
  Left,
  Center,
  Right,
  LeftOffense,
  CenterOffense,
  RightOffense,
  LeftDefense,
  CenterDefense,
  RightDefense
};

ateam_geometry::Segment GetLaneLongitudinalMidSegment(const World & world, const Lane & lane);

ateam_geometry::Rectangle GetLaneBounds(const World & world, const Lane & lane);

bool IsPointInLane(const World & world, const ateam_geometry::Point & point, const Lane & lane);

bool IsBallInLane(const World & world, const Lane & lane);

}  // namespace ateam_kenobi::play_helpers::lanes

#endif  // CORE__PLAY_HELPERS__LANES_HPP_
