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


#include "available_robots.hpp"

namespace ateam_kenobi::play_helpers
{

std::vector<Robot> getAvailableRobots(const World & world)
{
  return getVisibleRobots(world.our_robots);
}

std::vector<Robot> getVisibleRobots(const std::array<std::optional<Robot>, 16> & robots)
{
  std::vector<Robot> available_robots;
  for (const auto & maybe_robot : robots) {
    if (maybe_robot) {
      available_robots.push_back(maybe_robot.value());
    }
  }
  return available_robots;
}

void removeGoalie(std::vector<Robot> & robots, const World & world)
{
  return removeRobotWithId(robots, world.referee_info.our_goalie_id);
}

void removeGoalie(std::array<std::optional<Robot>, 16> & robots, const World & world)
{
  return removeRobotWithId(robots, world.referee_info.our_goalie_id);
}

void removeRobotWithId(std::vector<Robot> & robots, int id)
{
  robots.erase(
    std::remove_if(
      robots.begin(), robots.end(), [&id](const Robot & robot) {
        return robot.id == id;
      }), robots.end());
}

void removeRobotWithId(std::array<std::optional<Robot>, 16> & robots, int id)
{
  robots.at(id).reset();
}
}  // namespace ateam_kenobi::play_helpers
