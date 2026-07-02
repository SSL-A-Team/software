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

#ifndef ATEAM_PATH_PLANNING__PLANNER_OPTIONS_HPP_
#define ATEAM_PATH_PLANNING__PLANNER_OPTIONS_HPP_

#include <ateam_common/robot_constants.hpp>

namespace ateam_path_planning
{

struct Limits
{
  double linear_velocity = 0.0;
  double linear_acceleration = 0.0;
  double angular_velocity = 0.0;
  double angular_acceleration = 0.0;

  auto operator<=>(const Limits &) const = default;
};

struct ReplanThresholds
{
  /**
   * Distance, in meters, that the goal must move to trigger a replan
   */
  double goal_distance = 0.01;

  /**
   * Distance, in radians, that the goal heading must change to trigger a replan
   */
  double goal_heading_distance = 8e-3;

  /**
   * Distance, in meters, that the robot must deviate from the expected state, based on time, to
   * trigger a replan.
   */
  double deviation_distance = kRobotDiameter;

  /**
   * Time, in seconds, to look behind now for the robot's expected state along the path
   */
  double lag_estimate = 0.05;

  auto operator<=>(const ReplanThresholds &) const = default;
};

struct PlannerOptions
{
  double collision_check_resolution = 0.1;
  double collision_check_horizon = 3.0;
  double footprint_inflation = 0.06;
  double boundary_footprint_inflation = 0.02;
  double inter_target_dist_min = 1.0;
  double inter_target_dist_max = 5.0;
  double inter_target_dist_step = 1.0;
  double inter_target_angle_step = 0.5;
  bool ignore_start_obstacles = false;
  Limits limits;
  ReplanThresholds replan_thresholds;

  auto operator<=>(const PlannerOptions &) const = default;
};

}  // namespace ateam_path_planning

#endif  // ATEAM_PATH_PLANNING__PLANNER_OPTIONS_HPP_
