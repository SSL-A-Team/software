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


#ifndef CORE__PLAY_HELPERS__INTERCEPT_CALCULATION_HPP_
#define CORE__PLAY_HELPERS__INTERCEPT_CALCULATION_HPP_

#include <ateam_geometry/types.hpp>
#include "core/types/world.hpp"

namespace ateam_kenobi::play_helpers
{

struct InterceptResults {
    ateam_geometry::Vector robot_proj_ball;
    ateam_geometry::Vector robot_perp_ball;

    double d; // robot distance to ball projected onto the balls trajectory
    double h; // robot distance tangent to the balls trajectory

    double t; // time for robot to intercept ball
    double equation_sign; // -1 or +1 to represent which version of the equation was used
    std::optional<ateam_geometry::Point> intercept_pos = std::nullopt;
};

InterceptResults calculateIntercept(const World & world, const Robot & robot);

}  // namespace ateam_kenobi::play_helpers

#endif  // CORE__PLAY_HELPERS__INTERCEPT_CALCULATION_HPP_
