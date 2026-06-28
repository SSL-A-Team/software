// Copyright 2026 A Team
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


#ifndef CORE__PATH_PLANNING__PLANNER_OPTIONS_HPP_
#define CORE__PATH_PLANNING__PLANNER_OPTIONS_HPP_

#include <ateam_common/robot_constants.hpp>
#include <ateam_path_planning/planner_options.hpp>

namespace ateam_kenobi::motion::path_planning
{

struct PlannerOptions
{
  /**
   * If true, the planner treats the ball as an obstacle.
   */
  bool avoid_ball = true;

  /**
   * If true, the planner avoids default obstacles (currently just the two defense areas)
   */
  bool use_default_obstacles = true;

  /**
   * The size by which the radius of the robot will be augmented during collision checking
   *
   */
  double footprint_inflation = 0.06;

  /**
   * Time step, in seconds, used for collision checking
   */
  double collision_check_resolution = 0.1;

  /**
   * Time, in seconds, after which colliisons are ignored
   */
  double collision_check_horizon = 3.0;

  /**
   * @brief If true, any obstacles touching the start point will be ignored for all planning.
   *
   * Useful if you want to plan a path to escape a virtual obstacle like a keep out zone.
   */
  bool ignore_start_obstacle = true;

  bool draw_obstacles = false;

  bool force_replan = false;

  ateam_path_planning::ReplanThresholds replan_thresholds;
};

}  // namespace ateam_kenobi::path_planning


#endif  // CORE__PATH_PLANNING__PLANNER_OPTIONS_HPP_
