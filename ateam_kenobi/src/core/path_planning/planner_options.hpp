// Copyright 2025 A Team
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
namespace ateam_kenobi::path_planning
{

struct ReplanThresholds
{
  double goal_distance_ = 0.05;
  double obstacle_distance_ = kRobotRadius;
  double start_distance_ = 0.20;
};

struct PlannerOptions
{
  /**
   * @brief Max time before planner will give up searching for a path
   */
  double search_time_limit = 2e-3;  // seconds

  /**
   * @brief If true, the planner treats the ball as an obstacle.
   */
  bool avoid_ball = true;

  /**
   * @brief The size by which the radius of the robot will be augmented during collision checking
   *
   */
  double footprint_inflation = 0.06;

  double collision_check_resolution = 0.05;

  bool use_default_obstacles = true;

  /**
   * @brief If true, any obstacles touching the start point will be ignored for all planning.
   *
   * Useful if you want to plan a path to escape a virtual obstacle like a keep out zone.
   */
  bool ignore_start_obstacle = true;

  bool draw_obstacles = false;

  bool force_replan = false;

  /**
   * Any corners sharper than this angle will attempt to be smoothed, time permitting
   */
  double corner_smoothing_angle_threshold = 2.36;

  double corner_smoothing_step_size = 0.005;

  ReplanThresholds replan_thresholds;
};
  
} // namespace ateam_kenobi::path_planning


#endif  // CORE__PATH_PLANNING__PLANNER_OPTIONS_HPP_
