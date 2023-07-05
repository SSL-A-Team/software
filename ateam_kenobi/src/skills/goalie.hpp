// Copyright 2023 A Team
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

#ifndef SKILLS__GOALIE_HPP_
#define SKILLS__GOALIE_HPP_

#include "ateam_geometry/types.hpp"
#include "ateam_geometry/nearest_points.hpp"
#include "types/world.hpp"

namespace ateam_kenobi::skills
{
inline ateam_geometry::Point get_goalie_defense_point(const World & world){
    ateam_geometry::Segment goalie_line = ateam_geometry::Segment(
        ateam_geometry::Point(-4, 0.5),
        ateam_geometry::Point(-4, -0.5)
    );

    ateam_geometry::Point ball_location = world.ball.pos;

    return ateam_geometry::NearestPointOnSegment(goalie_line, ball_location);
}
}
#endif // SKILLS__GOALIE_HPP_

