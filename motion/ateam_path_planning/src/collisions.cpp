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

#include "ateam_path_planning/colliisions.hpp"
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/do_intersect.hpp>
#include <ateam_geometry/creation_helpers.hpp>
#include <ateam_geometry/printing.hpp>
#include "ateam_path_planning/controls_lib_adapters.hpp"

namespace ateam_path_planning::collisions
{

// TODO(barulicm): check state validity (in bounds)

std::optional<double> TimeToCollision(
  const BangBangTraj3D & trajectory,
  const Vector6C_t & start_state,
  const double & start_t,
  const std::vector<Obstacle> & obstacles,
  const double collision_check_resolution,
  const double footprint_inflation)
{
  const auto duration = GetBangBangTrajectoryDuration(trajectory);
  for (double t = 0.0; t < duration; t += collision_check_resolution) {
    Vector6C_t state_at_t;
    if(const auto err =
      ateam_controls_traj_state_at(trajectory, start_state, 0.0, t, &state_at_t);
      err != ATEAM_CONTROLS_OK)
    {
      std::cerr << "collision due to error: " << err << '\n';
      return t;
    }
    const ateam_geometry::Point robot_pos(state_at_t.data[0], state_at_t.data[1]);
    const auto robot_footprint = ateam_geometry::makeDisk(robot_pos,
        kRobotRadius + footprint_inflation);
    for (const auto & obstacle : obstacles) {
      if(ateam_geometry::doIntersect(robot_footprint,
          obstacle.ShapeAtT(t + start_t)))
      {
        std::cerr << "collision at time " << t << " with " << obstacle.ShapeAtT(t + start_t) <<
          '\n';
        return t;
      }
    }
  }

  return std::nullopt;
}

}  // namespace ateam_path_planning::collisions
