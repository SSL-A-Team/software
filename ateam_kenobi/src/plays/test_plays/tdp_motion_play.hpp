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


#ifndef PLAYS__TEST_PLAYS__TDP_MOTION_PLAY_HPP_
#define PLAYS__TEST_PLAYS__TDP_MOTION_PLAY_HPP_

#include <array>
#include <chrono>
#include <optional>
#include <string>
#include <vector>
#include <ateam_geometry/types.hpp>
#include "core/stp/play.hpp"

namespace ateam_kenobi::plays
{

/**
 * @brief Field-scale motion test for the TDP 2027 motion experiments
 * (firmware repo: TDP_2027_EXPERIMENTS.md, "Field-scale tests").
 *
 * Drives one robot through a scripted pattern of straight GlobalPosition moves
 * (path planner skipped, so the firmware's own trajectory and limits are what is
 * measured). Select it with the play override; configure it with ROS parameters on
 * kenobi_node under stp_parameters.TdpMotionPlay.* (read when the play is selected).
 *
 * Patterns (parameter "pattern"):
 *   star         center -> point at star_radius_m -> center, for star_directions
 *                directions at a fixed heading: samples acceleration vs. direction
 *                in the robot frame
 *   sprint_x     end to end along field x at a fixed heading
 *   sprint_y     side to side along field y
 *   sprint_diag  corner to corner
 *   tour         the usable rectangle's corners, turning 90 deg per side
 *                (translation and rotation together)
 * Every pattern repeats "cycles" times. A thermal soak is sprint_x with dwell_s 0 and
 * many cycles.
 *
 * Area (parameter "area"): "field" uses the field lines from vision (fallback_* without
 * vision geometry); "custom" uses area_center_x_m / area_center_y_m / area_length_m /
 * area_width_m / area_yaw_deg, e.g. a test patch while vision reports the whole field.
 * Targets stay margin_m inside the area. Patterns and heading_deg are in the area's frame.
 *
 * Defense areas: kenobi stops any non-goalie robot moving deeper into one (outside our ball
 * placement). With avoid_defense_areas (default) targets stay clear of them in field x.
 * To use them, set the test robot as our goalie in the GC and avoid_defense_areas false;
 * the play refuses to start without the goalie, so moves are never silently stopped.
 *
 * Safety: the robot is sent a Stop (and the play latches aborted) if it is seen faster
 * than abort_speed_mps, outside the area, or lost for more than 0.3 s. Re-select the
 * play to start over. A game-controller HALT always overrides it.
 */
class TdpMotionPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "TdpMotionPlay";

  explicit TdpMotionPlay(stp::Options stp_options);

  void enter() override;

  std::array<std::optional<RobotCommand>,
    16> runFrame(const World & world) override;

private:
  enum class Phase
  {
    WaitingForRobot,
    Running,
    Done,
    Aborted
  };

  struct Target
  {
    ateam_geometry::Point position;
    double heading;
  };

  struct Settings
  {
    std::string pattern;
    int robot_id;
    double accel;
    double vel;
    double ang_accel;
    double ang_vel;
    double heading;
    int star_directions;
    double star_radius;
    int cycles;
    double dwell_s;
    double margin;
    double arrive_tol;
    double arrive_timeout_s;
    double abort_speed;
    double fallback_length;
    double fallback_width;
    double fallback_defense_depth;
    std::string area;
    double area_center_x;
    double area_center_y;
    double area_length;
    double area_width;
    double area_yaw;
    bool avoid_defense_areas;
  };

  Settings settings_;
  Phase phase_ = Phase::WaitingForRobot;
  std::vector<Target> targets_;
  std::size_t target_index_ = 0;
  std::optional<int> robot_id_;
  std::chrono::steady_clock::time_point target_start_time_;
  std::optional<std::chrono::steady_clock::time_point> arrived_time_;
  std::optional<std::chrono::steady_clock::time_point> lost_since_;
  int timeouts_ = 0;
  std::string abort_reason_;
  // test area (field or custom) in field coordinates: the out-of-area abort uses it
  ateam_geometry::Point area_center_{0.0, 0.0};
  double area_yaw_ = 0.0;
  double area_half_length_ = 0.0;
  double area_half_width_ = 0.0;
  // usable rectangle for targets, in the area's frame (offset from the area center)
  ateam_geometry::Point usable_offset_{0.0, 0.0};
  double half_x_ = 0.0;
  double half_y_ = 0.0;

  void readSettings();
  bool buildTargets(const World & world);
  ateam_geometry::Point areaToField(double x, double y) const;
  bool insideArea(const ateam_geometry::Point & p, double tolerance) const;
  std::optional<int> pickRobot(const World & world) const;
  void abortRun(const std::string & reason);
  RobotCommand stopCommand() const;
  RobotCommand moveCommand(const Target & target, bool positioning) const;
  void fillPlayInfo();
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__TEST_PLAYS__TDP_MOTION_PLAY_HPP_
