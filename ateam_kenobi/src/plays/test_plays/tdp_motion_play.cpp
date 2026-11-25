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


#include "tdp_motion_play.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <ateam_geometry/creation_helpers.hpp>
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

namespace
{
constexpr double kArriveHeadingTol = 0.035;   // rad
constexpr double kPositioningAccel = 1.0;     // m/s^2, move to the start pose (not measured)
constexpr double kPositioningVel = 1.0;       // m/s
constexpr double kPositioningAngAccel = 5.0;  // rad/s^2
constexpr double kPositioningAngVel = 2.0;    // rad/s
constexpr double kStopAccel = 3.0;            // m/s^2
constexpr double kStopAngAccel = 10.0;        // rad/s^2
constexpr double kLostTimeoutS = 0.3;
constexpr double kFieldLineTolerance = 0.05;  // m outside the lines before aborting
constexpr double kMinUsableHalfExtent = 0.2;  // m

std::chrono::steady_clock::duration toDuration(double s)
{
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(s));
}
}  // namespace

TdpMotionPlay::TdpMotionPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options)
{
  auto & p = getParamInterface();
  p.declareParameter<std::string>("pattern", "star");
  p.declareParameter<int64_t>("robot_id", -1);            // -1: first available robot
  p.declareParameter<double>("accel", 3.0);               // m/s^2 requested (firmware may cap)
  p.declareParameter<double>("vel", 2.0);                 // m/s
  p.declareParameter<double>("ang_accel", 10.0);          // rad/s^2
  p.declareParameter<double>("ang_vel", 3.0);             // rad/s
  p.declareParameter<double>("heading_deg", 0.0);         // fixed heading, in the area's frame
  p.declareParameter<int64_t>("star_directions", 12);
  p.declareParameter<double>("star_radius_m", 1.0);       // clamped to the usable rectangle
  p.declareParameter<int64_t>("cycles", 1);
  p.declareParameter<double>("dwell_s", 1.0);             // hold at each target once arrived
  p.declareParameter<double>("margin_m", 0.3);            // inside the area (and defense areas)
  p.declareParameter<double>("arrive_tol_m", 0.02);
  p.declareParameter<double>("arrive_timeout_s", 6.0);    // per target, then move on
  p.declareParameter<double>("abort_speed_mps", 3.0);     // vision speed -> stop and abort
  // Used only when kenobi has no field geometry from vision (lab field lines 4470 x 2615 mm).
  p.declareParameter<double>("fallback_field_length_m", 4.47);
  p.declareParameter<double>("fallback_field_width_m", 2.615);
  p.declareParameter<double>("fallback_defense_depth_m", 0.5);
  // Test area: "field" (lines from vision) or "custom" (e.g. a test patch; see
  // controls/analysis/tdp/areas.json and `config.py area NAME`).
  p.declareParameter<std::string>("area", "field");
  p.declareParameter<double>("area_center_x_m", 0.0);
  p.declareParameter<double>("area_center_y_m", 0.0);
  p.declareParameter<double>("area_length_m", 0.0);       // along the area's x
  p.declareParameter<double>("area_width_m", 0.0);
  p.declareParameter<double>("area_yaw_deg", 0.0);
  // false needs the test robot to be our goalie (kenobi's defense-area keepout)
  p.declareParameter<bool>("avoid_defense_areas", true);
}

void TdpMotionPlay::enter()
{
  readSettings();
  phase_ = Phase::WaitingForRobot;
  targets_.clear();
  target_index_ = 0;
  robot_id_.reset();
  arrived_time_.reset();
  lost_since_.reset();
  timeouts_ = 0;
  abort_reason_.clear();
  RCLCPP_INFO(
    getLogger(), "TdpMotionPlay: pattern=%s accel=%.2f vel=%.2f heading=%.1f deg cycles=%d",
    settings_.pattern.c_str(), settings_.accel, settings_.vel,
    settings_.heading * 180.0 / M_PI, settings_.cycles);
}

void TdpMotionPlay::readSettings()
{
  auto & p = getParamInterface();
  settings_.pattern = p.getParameter<std::string>("pattern");
  settings_.robot_id = static_cast<int>(p.getParameter<int64_t>("robot_id"));
  settings_.accel = p.getParameter<double>("accel");
  settings_.vel = p.getParameter<double>("vel");
  settings_.ang_accel = p.getParameter<double>("ang_accel");
  settings_.ang_vel = p.getParameter<double>("ang_vel");
  settings_.heading = angles::normalize_angle(p.getParameter<double>("heading_deg") * M_PI / 180.0);
  settings_.star_directions = static_cast<int>(p.getParameter<int64_t>("star_directions"));
  settings_.star_radius = p.getParameter<double>("star_radius_m");
  settings_.cycles = std::max(1, static_cast<int>(p.getParameter<int64_t>("cycles")));
  settings_.dwell_s = std::max(0.0, p.getParameter<double>("dwell_s"));
  settings_.margin = p.getParameter<double>("margin_m");
  settings_.arrive_tol = p.getParameter<double>("arrive_tol_m");
  settings_.arrive_timeout_s = p.getParameter<double>("arrive_timeout_s");
  settings_.abort_speed = p.getParameter<double>("abort_speed_mps");
  settings_.fallback_length = p.getParameter<double>("fallback_field_length_m");
  settings_.fallback_width = p.getParameter<double>("fallback_field_width_m");
  settings_.fallback_defense_depth = p.getParameter<double>("fallback_defense_depth_m");
  settings_.area = p.getParameter<std::string>("area");
  settings_.area_center_x = p.getParameter<double>("area_center_x_m");
  settings_.area_center_y = p.getParameter<double>("area_center_y_m");
  settings_.area_length = p.getParameter<double>("area_length_m");
  settings_.area_width = p.getParameter<double>("area_width_m");
  settings_.area_yaw = angles::normalize_angle(p.getParameter<double>("area_yaw_deg") * M_PI / 180.0);
  settings_.avoid_defense_areas = p.getParameter<bool>("avoid_defense_areas");
}

ateam_geometry::Point TdpMotionPlay::areaToField(double x, double y) const
{
  // usable-rectangle coordinates -> area frame -> field
  const double ax = usable_offset_.x() + x;
  const double ay = usable_offset_.y() + y;
  const double c = std::cos(area_yaw_);
  const double s = std::sin(area_yaw_);
  return ateam_geometry::Point(
    area_center_.x() + c * ax - s * ay, area_center_.y() + s * ax + c * ay);
}

bool TdpMotionPlay::insideArea(const ateam_geometry::Point & p, double tolerance) const
{
  const double dx = p.x() - area_center_.x();
  const double dy = p.y() - area_center_.y();
  const double c = std::cos(area_yaw_);
  const double s = std::sin(area_yaw_);
  return std::abs(c * dx + s * dy) <= area_half_length_ + tolerance &&
         std::abs(-s * dx + c * dy) <= area_half_width_ + tolerance;
}

bool TdpMotionPlay::buildTargets(const World & world)
{
  const bool have_field = world.field.field_length > 0.f && world.field.field_width > 0.f;
  const double length = have_field ? world.field.field_length : settings_.fallback_length;
  const double width = have_field ? world.field.field_width : settings_.fallback_width;
  const double defense_depth = world.field.defense_area_depth > 0.f ?
    world.field.defense_area_depth : settings_.fallback_defense_depth;
  const double m = settings_.margin;

  if (settings_.area == "field") {
    area_center_ = ateam_geometry::Point(0.0, 0.0);
    area_yaw_ = 0.0;
    area_half_length_ = length / 2.0;
    area_half_width_ = width / 2.0;
  } else if (settings_.area == "custom") {
    if (settings_.area_length <= 0.0 || settings_.area_width <= 0.0) {
      abortRun("area custom needs area_length_m and area_width_m (config.py area NAME sets them)");
      return false;
    }
    area_center_ = ateam_geometry::Point(settings_.area_center_x, settings_.area_center_y);
    area_yaw_ = settings_.area_yaw;
    area_half_length_ = settings_.area_length / 2.0;
    area_half_width_ = settings_.area_width / 2.0;
  } else {
    abortRun("unknown area '" + settings_.area + "' (field, custom)");
    return false;
  }

  // usable rectangle in the area's frame
  double xmin = -area_half_length_ + m;
  double xmax = area_half_length_ - m;
  const double ymin = -area_half_width_ + m;
  const double ymax = area_half_width_ - m;
  if (settings_.avoid_defense_areas) {
    if (std::abs(area_yaw_) > 1e-3) {
      abortRun("avoid_defense_areas needs area_yaw_deg 0; or make the robot goalie and set it false");
      return false;
    }
    // keep targets out of both defense areas over their full width: field |x| <= limit
    const double limit = length / 2.0 - defense_depth - m;
    xmin = std::max(xmin, -limit - area_center_.x());
    xmax = std::min(xmax, limit - area_center_.x());
  }
  half_x_ = (xmax - xmin) / 2.0;
  half_y_ = (ymax - ymin) / 2.0;
  usable_offset_ = ateam_geometry::Point((xmin + xmax) / 2.0, (ymin + ymax) / 2.0);
  if (half_x_ < kMinUsableHalfExtent || half_y_ < kMinUsableHalfExtent) {
    abortRun("usable area too small; check margin_m, the area, and avoid_defense_areas");
    return false;
  }

  // Patterns are written in the usable rectangle's frame (centered, +/-half_x_ by
  // +/-half_y_); Point() here means that frame, converted to field coordinates below.
  using ateam_geometry::Point;
  const double h = settings_.heading;
  const double hx = half_x_;
  const double hy = half_y_;
  const auto & pattern = settings_.pattern;
  auto out_and_back = [&](Point a, Point b) {
      targets_.push_back({a, h});
      for (int c = 0; c < settings_.cycles; ++c) {
        targets_.push_back({b, h});
        targets_.push_back({a, h});
      }
    };

  if (pattern == "star") {
    const Point center(0.0, 0.0);
    const double r = std::min({settings_.star_radius, hx, hy});
    const int n = std::max(1, settings_.star_directions);
    targets_.push_back({center, h});
    for (int c = 0; c < settings_.cycles; ++c) {
      for (int k = 0; k < n; ++k) {
        const double a = 2.0 * M_PI * k / n;
        targets_.push_back({Point(r * std::cos(a), r * std::sin(a)), h});
        targets_.push_back({center, h});
      }
    }
  } else if (pattern == "sprint_x") {
    out_and_back(Point(-hx, 0.0), Point(hx, 0.0));
  } else if (pattern == "sprint_y") {
    out_and_back(Point(0.0, -hy), Point(0.0, hy));
  } else if (pattern == "sprint_diag") {
    out_and_back(Point(-hx, -hy), Point(hx, hy));
  } else if (pattern == "tour") {
    const std::array<Point, 4> corners{Point(-hx, -hy), Point(hx, -hy), Point(hx, hy),
      Point(-hx, hy)};
    targets_.push_back({corners[0], h});
    for (int c = 0; c < settings_.cycles; ++c) {
      for (int i = 1; i <= 4; ++i) {
        const int quarter_turns = c * 4 + i;
        targets_.push_back({corners[i % 4],
            angles::normalize_angle(h + quarter_turns * M_PI_2)});
      }
    }
  } else {
    abortRun("unknown pattern '" + pattern + "' (star, sprint_x, sprint_y, sprint_diag, tour)");
    return false;
  }

  for (auto & t : targets_) {
    t.position = areaToField(t.position.x(), t.position.y());
    t.heading = angles::normalize_angle(t.heading + area_yaw_);
  }

  RCLCPP_INFO(
    getLogger(),
    "TdpMotionPlay: %zu targets in +/-%.2f x +/-%.2f m; area %s %.2f x %.2f m at (%.2f, %.2f) "
    "yaw %.1f deg; field %s %.2f x %.2f m; defense areas %s",
    targets_.size(), hx, hy, settings_.area.c_str(), 2 * area_half_length_, 2 * area_half_width_,
    area_center_.x(), area_center_.y(), area_yaw_ * 180.0 / M_PI,
    have_field ? "from vision" : "fallback", length, width,
    settings_.avoid_defense_areas ? "avoided" : "used (robot must be goalie)");
  return true;
}

std::optional<int> TdpMotionPlay::pickRobot(const World & world) const
{
  if (settings_.robot_id >= 0) {
    if (settings_.robot_id < static_cast<int>(world.our_robots.size()) &&
      world.our_robots[settings_.robot_id].IsAvailable())
    {
      return settings_.robot_id;
    }
    return std::nullopt;
  }
  const auto available = play_helpers::getAvailableRobots(world);
  if (available.empty()) {
    return std::nullopt;
  }
  return available.front().id;
}

void TdpMotionPlay::abortRun(const std::string & reason)
{
  if (phase_ != Phase::Aborted) {
    RCLCPP_ERROR(getLogger(), "TdpMotionPlay aborted: %s", reason.c_str());
  }
  phase_ = Phase::Aborted;
  abort_reason_ = reason;
}

RobotCommand TdpMotionPlay::stopCommand() const
{
  RobotCommand command;
  motion::intents::Stop stop;
  stop.limits.linear_acceleration = kStopAccel;
  stop.limits.angular_acceleration = kStopAngAccel;
  command.motion_intent = stop;
  command.kick = KickState::Disable;
  command.dribbler_setpoint = 0.0;
  return command;
}

RobotCommand TdpMotionPlay::moveCommand(const Target & target, bool positioning) const
{
  motion::intents::Position intent;
  intent.position = target.position;
  intent.heading = target.heading;
  intent.planner_options.skip_planning = true;  // straight GlobalPosition: measure the firmware
  intent.limits.linear_velocity = positioning ?
    std::min(settings_.vel, kPositioningVel) : settings_.vel;
  intent.limits.linear_acceleration = positioning ?
    std::min(settings_.accel, kPositioningAccel) : settings_.accel;
  intent.limits.angular_velocity = positioning ?
    std::min(settings_.ang_vel, kPositioningAngVel) : settings_.ang_vel;
  intent.limits.angular_acceleration = positioning ?
    std::min(settings_.ang_accel, kPositioningAngAccel) : settings_.ang_accel;
  RobotCommand command;
  command.motion_intent = intent;
  command.kick = KickState::Disable;
  command.dribbler_setpoint = 0.0;
  return command;
}

std::array<std::optional<RobotCommand>, 16> TdpMotionPlay::runFrame(const World & world)
{
  std::array<std::optional<RobotCommand>, 16> commands;
  const auto now = world.current_time;

  if (phase_ == Phase::WaitingForRobot) {
    if (targets_.empty() && !buildTargets(world)) {
      fillPlayInfo();
      return commands;
    }
    robot_id_ = pickRobot(world);
    if (!robot_id_) {
      fillPlayInfo();
      return commands;
    }
    if (!settings_.avoid_defense_areas && world.referee_info.our_goalie_id != *robot_id_) {
      abortRun(
        "avoid_defense_areas is false but robot " + std::to_string(*robot_id_) +
        " is not our goalie: set it as goalie in the GC (kenobi stops non-goalies entering "
        "defense areas), or set avoid_defense_areas true");
      fillPlayInfo();
      return commands;
    }
    phase_ = Phase::Running;
    target_index_ = 0;
    target_start_time_ = now;
    RCLCPP_INFO(getLogger(), "TdpMotionPlay: running robot %d", *robot_id_);
  }

  if (!robot_id_) {
    fillPlayInfo();
    return commands;
  }
  const auto id = *robot_id_;
  const auto & robot = world.our_robots[id];

  if (phase_ != Phase::Aborted) {
    if (!robot.IsAvailable()) {
      if (!lost_since_) {
        lost_since_ = now;
      }
      if (now - *lost_since_ > toDuration(kLostTimeoutS)) {
        abortRun("robot lost (not visible or radio disconnected)");
      }
    } else {
      lost_since_.reset();
      const double speed = std::hypot(robot.vel.x(), robot.vel.y());
      if (speed > settings_.abort_speed) {
        abortRun("speed " + std::to_string(speed) + " m/s > abort_speed_mps");
      } else if (!insideArea(robot.pos, kFieldLineTolerance)) {
        abortRun("robot outside the test area");
      }
    }
  }

  if (phase_ == Phase::Aborted || lost_since_) {
    commands[id] = stopCommand();
    fillPlayInfo();
    return commands;
  }

  if (phase_ == Phase::Running) {
    const auto & target = targets_[target_index_];
    const double dist = std::hypot(
      robot.pos.x() - target.position.x(), robot.pos.y() - target.position.y());
    const double heading_err =
      std::abs(angles::shortest_angular_distance(robot.theta, target.heading));
    bool advance = false;
    if (!arrived_time_ && dist < settings_.arrive_tol && heading_err < kArriveHeadingTol) {
      arrived_time_ = now;  // the dwell runs from first arrival, even if it wobbles out after
    }
    if (arrived_time_) {
      advance = now - *arrived_time_ >= toDuration(settings_.dwell_s);
    } else if (now - target_start_time_ > toDuration(settings_.arrive_timeout_s)) {
      ++timeouts_;
      RCLCPP_WARN(
        getLogger(), "TdpMotionPlay: target %zu not reached in %.1f s (%.3f m, %.3f rad off)",
        target_index_, settings_.arrive_timeout_s, dist, heading_err);
      advance = true;
    }
    if (advance) {
      arrived_time_.reset();
      target_start_time_ = now;
      if (target_index_ + 1 >= targets_.size()) {
        phase_ = Phase::Done;
        RCLCPP_INFO(getLogger(), "TdpMotionPlay: done (%d timeouts)", timeouts_);
      } else {
        ++target_index_;
      }
    }
  }

  const auto & target = targets_[target_index_];
  commands[id] = moveCommand(target, target_index_ == 0);

  getOverlays().drawLine(
    "usable_area", {areaToField(-half_x_, -half_y_), areaToField(half_x_, -half_y_),
      areaToField(half_x_, half_y_), areaToField(-half_x_, half_y_),
      areaToField(-half_x_, -half_y_)}, "yellow", 2);
  getOverlays().drawCircle(
    "target", ateam_geometry::makeCircle(target.position, 0.05), "orange", "transparent");

  fillPlayInfo();
  return commands;
}

void TdpMotionPlay::fillPlayInfo()
{
  auto & info = getPlayInfo();
  switch (phase_) {
    case Phase::WaitingForRobot: info["phase"] = "waiting for robot"; break;
    case Phase::Running: info["phase"] = "running"; break;
    case Phase::Done: info["phase"] = "done"; break;
    case Phase::Aborted: info["phase"] = "ABORTED"; break;
  }
  info["pattern"] = settings_.pattern;
  info["robot"] = robot_id_ ? *robot_id_ : -1;
  info["target"] = target_index_;
  info["targets"] = targets_.size();
  info["timeouts"] = timeouts_;
  info["abort_reason"] = abort_reason_;
  info["usable_half_extents_m"] = {half_x_, half_y_};
  info["area"] = settings_.area;
  info["area_center_m"] = {area_center_.x(), area_center_.y()};
  info["area_size_m"] = {2 * area_half_length_, 2 * area_half_width_};
  info["avoid_defense_areas"] = settings_.avoid_defense_areas;
}

}  // namespace ateam_kenobi::plays
