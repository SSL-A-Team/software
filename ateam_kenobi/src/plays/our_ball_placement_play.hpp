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

#ifndef PLAYS__OUR_BALL_PLACEMENT_PLAY_HPP_
#define PLAYS__OUR_BALL_PLACEMENT_PLAY_HPP_

#include <vector>
#include "core/stp/play.hpp"
#include "tactics/pass.hpp"
#include "skills/dribble.hpp"
#include "tactics/multi_move_to.hpp"

namespace ateam_kenobi::plays
{

class OurBallPlacementPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "OurBallPlacementPlay";

  explicit OurBallPlacementPlay(stp::Options stp_options);

  stp::PlayScore getScore(const World & world) override;


  void reset() override;

  std::array<std::optional<RobotCommand>,
    16> runFrame(const World & world) override;

private:
  tactics::Pass pass_tactic_;
  skills::Dribble dribble_;
  tactics::MultiMoveTo multi_move_to_;
  ateam_geometry::Point placement_point_;

  double approach_radius_ = kRobotRadius + kBallRadius + 0.3;  // m
  ateam_geometry::Point approach_point_;

  // Ball should have enough room for the robot to fit between it and the obstacle with a bit of
  // extra space
  double ball_distance_from_obstacle_ = kRobotDiameter + kBallRadius + 0.03;

  std::optional<ateam_geometry::Vector> calculateObstacleOffset(const World & world);


  enum class State
  {
    Extracting,
    Passing,
    Placing,
    Done
  } state_ = State::Passing;

  void runExtracting(
    const std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<RobotCommand>,
    16> & motion_commands);

  void runPassing(
    const std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<RobotCommand>,
    16> & motion_commands);

  void runPlacing(
    const std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<RobotCommand>,
    16> & motion_commands);

  void runDone(
    const std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<RobotCommand>,
    16> & motion_commands);

  void DrawKeepoutArea(
    const ateam_geometry::Point & ball_pos,
    const ateam_geometry::Point & placement_point);

  bool shouldRobotMove(const World & world, const ateam_geometry::Point & placement_point, const Robot & robot);

  ateam_geometry::Point getTargetPoint(const World & world, const ateam_geometry::Point & placement_point, const Robot & robot);
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__OUR_BALL_PLACEMENT_PLAY_HPP_
