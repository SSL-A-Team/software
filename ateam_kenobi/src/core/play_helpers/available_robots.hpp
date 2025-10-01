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


#ifndef CORE__PLAY_HELPERS__AVAILABLE_ROBOTS_HPP_
#define CORE__PLAY_HELPERS__AVAILABLE_ROBOTS_HPP_

#include <vector>
#include <ateam_geometry/types.hpp>
#include "core/types.hpp"
#include "core/types.hpp"

namespace ateam_kenobi::play_helpers
{

std::vector<Robot> getAvailableRobots(const World & world);

std::vector<Robot> getVisibleRobots(const std::array<Robot, 16> & robots);

void removeGoalie(std::vector<Robot> & robots, const World & world);

void removeRobotWithId(std::vector<Robot> & robots, int id);

/**
 * @brief Get the closest robot to the target point.
 * @note Assumes @c robots is not empty.
 */
Robot getClosestRobot(const std::vector<Robot> & robots, const ateam_geometry::Point & target);

}  // namespace ateam_kenobi::play_helpers

#endif  // CORE__PLAY_HELPERS__AVAILABLE_ROBOTS_HPP_
