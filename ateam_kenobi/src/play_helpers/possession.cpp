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

#include "possession.hpp"
#include <limits>
#include <ranges>
#include <ateam_common/robot_constants.hpp>
#include "types/robot.hpp"

namespace ateam_kenobi::play_helpers
{

PossessionResult WhoHasPossession(const World & world)
{
  const auto possession_threshold = 0.03;
  const auto possession_threhold_sq = possession_threshold * possession_threshold;
  const auto & ball_pos = world.ball.pos;

  double closest_our_bot_sq_distance = std::numeric_limits<double>::infinity();
  double closest_their_bot_sq_distance = std::numeric_limits<double>::infinity();

  for (const auto & robot : world.our_robots) {
    if (!robot.visible) {
      continue;
    }
    const auto distance = CGAL::squared_distance(ball_pos, robot.pos);
    closest_our_bot_sq_distance = std::min(closest_our_bot_sq_distance, distance);
  }

  for (const auto & robot : world.their_robots) {
    if (!robot.visible) {
      continue;
    }
    const auto distance = CGAL::squared_distance(ball_pos, robot.pos);
    closest_their_bot_sq_distance = std::min(closest_their_bot_sq_distance, distance);
  }

  if (closest_our_bot_sq_distance < closest_their_bot_sq_distance &&
    closest_our_bot_sq_distance < possession_threhold_sq)
  {
    return PossessionResult::Ours;
  } else if (closest_our_bot_sq_distance > closest_their_bot_sq_distance &&
    closest_their_bot_sq_distance < possession_threhold_sq)
  {
    return PossessionResult::Theirs;
  } else {
    return PossessionResult::Neither;
  }
}

}  // namespace ateam_kenobi::play_helpers
