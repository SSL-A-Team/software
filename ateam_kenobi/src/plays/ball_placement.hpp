#ifndef PLAYS__TRIANGLE_PASS_PLAY_HPP_
#define PLAYS__TRIANGLE_PASS_PLAY_HPP_

#include "base_play.hpp"
#include "skills/line_kick.hpp"
#include "play_helpers/easy_move_to.hpp"
#include <ateam_geometry/ateam_geometry.hpp>

namespace ateam_kenobi::plays
{

class BallPlacementPlay : public BasePlay
{
public:
  BallPlacementPlay();

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

private:
  skills::LineKick line_kick_;
  std::array<play_helpers::EasyMoveTo, 16> easy_move_tos_;
  ateam_geometry::Point> placement_point;

  enum class State
  {
    Kicking,
    Receiving,
    Placing
  } state_ = State::Kicking;

  void runKicking(
    const std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);

  void runReceiving(
    const std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);

  void runPlacing(
    const std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);

};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__BALL_PLACEMENT__PLAY_HPP_
