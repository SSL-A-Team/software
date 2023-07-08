#ifndef PLAYS__THEIR_PENALTY_PLAY_HPP_
#define PLAYS__THEIR_PENALTY_PLAY_HPP_

#include "base_play.hpp"
#include "play_helpers/easy_move_to.hpp"
#include "skills/goalie.hpp"

namespace ateam_kenobi::plays
{

class TheirPenaltyPlay : public BasePlay
{
public:
  TheirPenaltyPlay(visualization::OverlayPublisher & op, visualization::PlayInfoPublisher & pip);

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(const World & world) override;

private:
  std::array<play_helpers::EasyMoveTo, 16> move_tos_;
  skills::Goalie goalie_skill_;

};

}

#endif  // PLAYS__THEIR_PENALTY_PLAY_HPP_