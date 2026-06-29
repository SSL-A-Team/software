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

#ifndef PLAYS__DEFENSE_CLEAR_PLAY_HPP_
#define PLAYS__DEFENSE_CLEAR_PLAY_HPP_

#include "core/stp/play.hpp"
#include "skills/line_kick.hpp"
#include "tactics/standard_defense.hpp"
#include "tactics/blockers.hpp"

namespace ateam_kenobi::plays
{

class DefenseClearPlay : public stp::Play
{
public:

  enum class ShotType {
    NoShot, // No clear shot, keep defending
    StraightLine, // Kick straight away from goal for safety
    DownfieldLine // Turn to shoot downfield on the sides
  };

  static constexpr const char * kPlayName = "DefenseClearPlay";

  explicit DefenseClearPlay(stp::Options stp_options);

  stp::PlayScore getScore(const World & world) override;

  void reset() override;

  std::array<std::optional<RobotCommand>, 16> runFrame(
    const World & world) override;

  RobotCommand runReceivingRobot(const World & world, const Robot & robot,
    const ateam_geometry::Point target_pos);
  RobotCommand runPositionBasedRobot(const World & world, const Robot & robot,
    const ateam_geometry::Point target_pos);
  RobotCommand runClearingRobot(const World & world, const Robot & robot,
    const ateam_geometry::Point defense_point);

  bool shouldDefenseClearBall(const World & world);
  ateam_geometry::Point getClearTargetPoint(const World & world);
  ateam_geometry::Point getGuardPoint(const World & world);

  // Returns true if the incoming command tries to pass across the keepout zone
  bool enforceKeepoutZone(const World & world,
    const Robot & robot, ateam_geometry::Point & target_point);

private:
  static constexpr double defense_zone_clear_threshold_ = 0.4;
  static constexpr double ball_velocity_clear_threshold_ = 0.1;

  static constexpr double clear_window_angular_width_ = M_PI / 6.0;
  // Evaluated at the far end of the window
  static constexpr double clear_window_min_kick_width = 3 * kRobotDiameter;
  static constexpr double clear_receiver_distance_ = 3.0;

  // How far away oponents must be to clear downfield on the sides
  static constexpr double clear_downfield_distance_threshold_ = 3.0;

  ShotType shot_type_ = ShotType::NoShot;
  // Latch receive point so receiver robot doesn't move with ball
  std::optional<ateam_geometry::Point> kicked_to_target_;
  // Keep all robots out of viable shot zone
  std::optional<ateam_geometry::Segment> clearing_keepout_zone_;
  // Current target segment in consideration
  std::optional<ateam_geometry::Segment> clearing_target_segment_;

  skills::LineKick kick_;
  tactics::StandardDefense defense_tactic_;
  tactics::Blockers blockers_;
};

}  // namespace ateam_kenobi::plays


#endif  // PLAYS__DEFENSE_CLEAR_PLAY_HPP_
