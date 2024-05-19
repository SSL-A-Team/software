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


#ifndef PLAYS__WAYPOINTS_PLAY_HPP_
#define PLAYS__WAYPOINTS_PLAY_HPP_

#include <array>
#include <chrono>
#include <tuple>
#include <vector>
#include <string>
#include <ateam_geometry/types.hpp>
#include "stp/play.hpp"
#include "play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::plays
{

class WaypointsPlay : public stp::Play
{
public:
  explicit WaypointsPlay(stp::Options stp_options);

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

private:
  struct Pose
  {
    ateam_geometry::Point position;
    double heading;
  };

  struct Waypoint
  {
    std::vector<Pose> poses;
    int64_t duration_ms;
  };

  std::array<play_helpers::EasyMoveTo, 16> easy_move_tos_;
  std::vector<Waypoint> waypoints_;
  std::chrono::steady_clock::time_point next_transition_time_ =
    std::chrono::steady_clock::time_point::max();
  std::size_t waypoint_index_ = 0;

  void addWaypoint(
    const int64_t duration_ms, const std::vector<std::tuple<double, double,
    double>> & poses);
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__WAYPOINTS_PLAY_HPP_
