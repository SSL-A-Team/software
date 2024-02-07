#ifndef PLAYS__TRIANGLE_PASS_PLAY_HPP_
#define PLAYS__TRIANGLE_PASS_PLAY_HPP_

#include "base_play.hpp"
#include "skills/line_kick.hpp"
#include "play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::plays
{

class TrianglePassPlay : public BasePlay
{
public:
  TrianglePassPlay();

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

private:
  skills::LineKick line_kick_;
  std::array<play_helpers::EasyMoveTo, 16> easy_move_tos_;
  std::vector<ateam_geometry::Point> positions;
  double ball_vel_avg_ = 0.0;
  bool latch_receive_ = false;
  int last_kicked_id_ = 0;

  enum class State
  {
    Kicking,
    Receiving,
    BackOff
  } state_ = State::Kicking;

  void runKicking(
    const std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);
  void runReceiving(
    std::vector<Robot> available_robots, const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);
  void runBackOff(
    const std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);

};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__TRIANGLE_PASS_PLAY_HPP_
