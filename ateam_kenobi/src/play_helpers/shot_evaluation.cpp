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

#include "shot_evaluation.hpp"
#include <angles/angles.h>
#include <ateam_geometry/angles.hpp>
#include "play_helpers/available_robots.hpp"
#include "play_helpers/window_evaluation.hpp"

namespace ateam_kenobi::play_helpers
{

double GetShotSuccessChance(const World & world, const ateam_geometry::Point & shot_point)
{
  const ateam_geometry::Segment their_goal_segment{
    ateam_geometry::Point{world.field.field_length / 2.0, -world.field.goal_width / 2.0},
    ateam_geometry::Point{world.field.field_length / 2.0, world.field.goal_width / 2.0, }
  };

  const ateam_geometry::Point goal_midpoint{world.field.field_length / 2.0, 0.0};

  const ateam_geometry::Vector ball_goal_vector(shot_point, goal_midpoint);

  const auto shot_angle = std::abs(
    ateam_geometry::ShortestAngleBetween(
      their_goal_segment.to_vector(), ball_goal_vector));

  if (shot_angle < angles::from_degrees(15) || shot_angle > angles::from_degrees(165)) {
    return 0.0;
  }

  const auto windows = play_helpers::window_evaluation::getWindows(
    their_goal_segment,
    shot_point, play_helpers::getVisibleRobots(world.their_robots));
  const auto largest_window = play_helpers::window_evaluation::getLargestWindow(windows);
  if (!largest_window) {
    return 0.0;
  }

  const auto shot_width_ratio = largest_window->squared_length() /
    their_goal_segment.squared_length();

  const auto distance_to_goal =
    CGAL::approximate_sqrt(CGAL::squared_distance(shot_point, goal_midpoint));

  const auto distance_threshold = 2.0;
  const auto shot_distance_factor = distance_to_goal <
    distance_threshold ? 1.0 : distance_threshold / distance_to_goal;
  return 100.0 * shot_width_ratio * shot_distance_factor;
}

}  // namespace ateam_kenobi::play_helpers
