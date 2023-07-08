#ifndef PLAYS__OUR_PENALTY_PLAY_HPP_
#define PLAYS__OUR_PENALTY_PLAY_HPP_

#include "base_play.hpp"
#include "skills/line_kick.hpp"
#include "play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::plays
{

class OurPenaltyPlay : public BasePlay
{
public:
  OurPenaltyPlay(visualization::OverlayPublisher & op, visualization::PlayInfoPublisher & pip);

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(const World & world) override;

private:
  skills::LineKick line_kick_skill_;
  std::array<play_helpers::EasyMoveTo, 16> move_tos_;

};

}

#endif  // PLAYS__OUR_PENALTY_PLAY_HPP_