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

#include "lanes.hpp"
#include <ateam_geometry/do_intersect.hpp>

namespace ateam_kenobi::play_helpers::lanes
{

ateam_geometry::Segment GetLaneLongitudinalMidSegment(const World & world, const Lane & lane)
{
  const auto half_field_length = world.field.field_length / 2.0;
  const auto one_third_field_width = (1.0 / 3.0) * world.field.field_width;
  switch (lane) {
    case Lane::Left:
      return ateam_geometry::Segment{
        ateam_geometry::Point{-half_field_length, one_third_field_width},
        ateam_geometry::Point{half_field_length, one_third_field_width}
      };
    case Lane::Center:
      return ateam_geometry::Segment{
        ateam_geometry::Point{-half_field_length, 0.0},
        ateam_geometry::Point{half_field_length, 0.0}
      };
    case Lane::Right:
      return ateam_geometry::Segment{
        ateam_geometry::Point{-half_field_length, -one_third_field_width},
        ateam_geometry::Point{half_field_length, -one_third_field_width}
      };
    case Lane::LeftOffense:
      return ateam_geometry::Segment{
        ateam_geometry::Point{0.0, one_third_field_width},
        ateam_geometry::Point{half_field_length, one_third_field_width}
      };
    case Lane::CenterOffense:
      return ateam_geometry::Segment{
        ateam_geometry::Point{0.0, 0.0},
        ateam_geometry::Point{half_field_length, 0.0}
      };
    case Lane::RightOffense:
      return ateam_geometry::Segment{
        ateam_geometry::Point{0.0, -one_third_field_width},
        ateam_geometry::Point{half_field_length, -one_third_field_width}
      };
    case Lane::LeftDefense:
      return ateam_geometry::Segment{
        ateam_geometry::Point{-half_field_length, one_third_field_width},
        ateam_geometry::Point{0.0, one_third_field_width}
      };
    case Lane::CenterDefense:
      return ateam_geometry::Segment{
        ateam_geometry::Point{-half_field_length, 0.0},
        ateam_geometry::Point{0.0, 0.0}
      };
    case Lane::RightDefense:
      return ateam_geometry::Segment{
        ateam_geometry::Point{-half_field_length, -one_third_field_width},
        ateam_geometry::Point{0.0, -one_third_field_width}
      };
    default:
      throw std::runtime_error("Unrecognize lane value.");
  }
}

ateam_geometry::Rectangle GetLaneBounds(const World & world, const Lane & lane)
{
  const auto half_field_length = world.field.field_length / 2.0;
  const auto sixth_field_width = world.field.field_width / 6.0;
  const auto half_field_width = world.field.field_width / 2.0;
  switch (lane) {
    case Lane::Left:
      return ateam_geometry::Rectangle{
        ateam_geometry::Point{-half_field_length, half_field_width},
        ateam_geometry::Point{half_field_length, sixth_field_width}
      };
    case Lane::Center:
      return ateam_geometry::Rectangle{
        ateam_geometry::Point{-half_field_length, sixth_field_width},
        ateam_geometry::Point{half_field_length, -sixth_field_width}
      };
    case Lane::Right:
      return ateam_geometry::Rectangle{
        ateam_geometry::Point{-half_field_length, -sixth_field_width},
        ateam_geometry::Point{half_field_length, -half_field_width}
      };
    case Lane::LeftOffense:
      return ateam_geometry::Rectangle{
        ateam_geometry::Point{0.0, half_field_width},
        ateam_geometry::Point{half_field_length, sixth_field_width}
      };
    case Lane::CenterOffense:
      return ateam_geometry::Rectangle{
        ateam_geometry::Point{0.0, sixth_field_width},
        ateam_geometry::Point{half_field_length, -sixth_field_width}
      };
    case Lane::RightOffense:
      return ateam_geometry::Rectangle{
        ateam_geometry::Point{0.0, -sixth_field_width},
        ateam_geometry::Point{half_field_length, -half_field_width}
      };
    case Lane::LeftDefense:
      return ateam_geometry::Rectangle{
        ateam_geometry::Point{-half_field_length, half_field_width},
        ateam_geometry::Point{0.0, sixth_field_width}
      };
    case Lane::CenterDefense:
      return ateam_geometry::Rectangle{
        ateam_geometry::Point{-half_field_length, sixth_field_width},
        ateam_geometry::Point{0.0, -sixth_field_width}
      };
    case Lane::RightDefense:
      return ateam_geometry::Rectangle{
        ateam_geometry::Point{-half_field_length, -sixth_field_width},
        ateam_geometry::Point{0.0, -half_field_width}
      };
    default:
      throw std::runtime_error("Unrecognize lane value.");
  }
}

bool IsPointInLane(const World & world, const ateam_geometry::Point & point, const Lane & lane)
{
  const auto rectangle = GetLaneBounds(world, lane);
  return ateam_geometry::doIntersect(point, rectangle);
}

bool IsBallInLane(const World & world, const Lane & lane)
{
  return IsPointInLane(world, world.ball.pos, lane);
}

}  // namespace ateam_kenobi::play_helpers::lanes
