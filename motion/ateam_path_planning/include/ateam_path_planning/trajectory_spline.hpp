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

#ifndef ATEAM_PATH_PLANNING__TRAJECTORY_SPLINE_HPP_
#define ATEAM_PATH_PLANNING__TRAJECTORY_SPLINE_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <vector>
#include <ateam_game_state/world.hpp>
#include <ateam_geometry/types.hpp>
#include "pose.hpp"

namespace ateam_path_planning
{
struct CollisionStats;
struct Obstacle;
class TrajectorySpline;
struct TrajectorySplineImpl;

namespace collisions
{
CollisionStats GetCollisionStats(
  const TrajectorySpline & spline,
  const std::vector<Obstacle> & obstacles,
  const ateam_game_state::World & world,
  const double collision_check_resolution,
  const double collision_check_horizon,
  const double footprint_inflation,
  const double boundary_footprint_inflation,
  const double search_start_t);
}  // namespace collisions

class TrajectorySpline
{
public:
  TrajectorySpline(const TrajectorySpline & other);
  TrajectorySpline(TrajectorySpline && other);

  ~TrajectorySpline();

  TrajectorySpline & operator=(const TrajectorySpline & other);
  TrajectorySpline & operator=(TrajectorySpline && other);

  std::optional<Pose> GetStateAtT(double t) const;

  std::optional<Pose> GetStateAt(const std::chrono::steady_clock::time_point & time_point) const;

  std::optional<Pose> GetTargetAtT(double t) const;

  std::optional<Pose> GetTargetAtNow() const;

  std::vector<ateam_geometry::Point> ToPoints(double delta_t = 0.1) const;

  std::vector<std::vector<ateam_geometry::Point>> ToPointsBySegment(double delta_t = 0.1) const;

  Pose GetStartPose() const;

  Pose GetEndPose() const;

  size_t GetSegmentCount() const;

  double GetTotalDuration() const;

  std::chrono::steady_clock::time_point GetStartTime() const;

private:
  explicit TrajectorySpline(TrajectorySplineImpl & impl);

  std::unique_ptr<TrajectorySplineImpl> impl_;

  friend TrajectorySpline MakeTrajectorySpline(TrajectorySplineImpl &);
  friend TrajectorySpline MakeTrajectorySpline(TrajectorySplineImpl);

  friend CollisionStats collisions::GetCollisionStats(
    const TrajectorySpline & spline,
    const std::vector<Obstacle> & obstacles,
    const ateam_game_state::World & world,
    const double collision_check_resolution,
    const double collision_check_horizon,
    const double footprint_inflation,
    const double boundary_footprint_inflation,
    const double search_start_t);
};
}  // namespace ateam_path_planning

#endif  // ATEAM_PATH_PLANNING__TRAJECTORY_SPLINE_HPP_
