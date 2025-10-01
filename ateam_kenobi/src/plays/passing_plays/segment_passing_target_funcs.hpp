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


#ifndef PLAYS__PASSING_PLAYS__SEGMENT_PASSING_TARGET_FUNCS_HPP_
#define PLAYS__PASSING_PLAYS__SEGMENT_PASSING_TARGET_FUNCS_HPP_

#include <ateam_geometry/types.hpp>
#include "core/types.hpp"
#include "core/play_helpers/lanes.hpp"
#include "core/stp/base.hpp"

namespace ateam_kenobi::segment_passing_target_funcs
{

inline ateam_geometry::Segment GetForwardSegment(
  const World & world,
  const play_helpers::lanes::Lane lane)
{
  const auto lane_segment = play_helpers::lanes::GetLaneLongitudinalMidSegment(world, lane);
  const auto ball_x = world.ball.pos.x();
  return ateam_geometry::Segment{
    ateam_geometry::Point{lane_segment.source().x() < ball_x ? ball_x : lane_segment.source().x(),
      lane_segment.source().y()},
    ateam_geometry::Point{lane_segment.target().x() < ball_x ? ball_x : lane_segment.target().x(),
      lane_segment.target().y()}
  };
}

inline ateam_geometry::Segment GetBackwardSegment(
  const World & world,
  const play_helpers::lanes::Lane lane)
{
  const auto lane_segment = play_helpers::lanes::GetLaneLongitudinalMidSegment(world, lane);
  const auto ball_x = world.ball.pos.x();
  return ateam_geometry::Segment{
    ateam_geometry::Point{lane_segment.source().x() > ball_x ? ball_x : lane_segment.source().x(),
      lane_segment.source().y()},
    ateam_geometry::Point{lane_segment.target().x() > ball_x ? ball_x : lane_segment.target().x(),
      lane_segment.target().y()}
  };
}

inline ateam_geometry::Segment LeftForwardPassTarget(const World & world)
{
  return GetForwardSegment(world, play_helpers::lanes::Lane::Left);
}

inline ateam_geometry::Segment CenterForwardPassTarget(const World & world)
{
  return GetForwardSegment(world, play_helpers::lanes::Lane::Center);
}

inline ateam_geometry::Segment RightForwardPassTarget(const World & world)
{
  return GetForwardSegment(world, play_helpers::lanes::Lane::Right);
}

inline ateam_geometry::Segment LeftBackwardPassTarget(const World & world)
{
  return GetBackwardSegment(world, play_helpers::lanes::Lane::Left);
}

inline ateam_geometry::Segment CenterBackwardPassTarget(const World & world)
{
  return GetBackwardSegment(world, play_helpers::lanes::Lane::Center);
}

inline ateam_geometry::Segment RightBackwardPassTarget(const World & world)
{
  return GetBackwardSegment(world, play_helpers::lanes::Lane::Right);
}

}  // namespace ateam_kenobi::segment_passing_target_funcs

#endif  // PLAYS__PASSING_PLAYS__SEGMENT_PASSING_TARGET_FUNCS_HPP_
