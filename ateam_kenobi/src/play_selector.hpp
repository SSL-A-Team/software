#ifndef ATEAM_KENOBI__PLAY_SELECTOR_HPP_
#define ATEAM_KENOBI__PLAY_SELECTOR_HPP_

#include "plays/all_plays.hpp"
#include "types/world.hpp"
#include "visualization/overlay_publisher.hpp"
#include "visualization/play_info_publisher.hpp"

namespace ateam_kenobi
{

class PlaySelector
{
public:
  explicit PlaySelector(visualization::OverlayPublisher & overlay_publisher, 
    visualization::PlayInfoPublisher & play_info_publisher);

  plays::BasePlay * getPlay(const World & world);

private:
  plays::TestPlay test_play_;
  plays::HaltPlay halt_play_;
  plays::StopPlay stop_play_;
  plays::WallPlay wall_play_;
  plays::OurKickoffPlay our_kickoff_play_;
  plays::TestKickPlay test_kick_play_;
  plays::Basic122 basic_122_play_;
  plays::OurPenaltyPlay our_penalty_play_;
  plays::TheirPenaltyPlay their_penalty_play_;

  ateam_common::GameCommand previous_game_command_ = ateam_common::GameCommand::Halt;
  void * prev_play_address_ = nullptr;
  std::size_t prev_play_type_index_ = -1;

  void resetPlayIfNeeded(plays::BasePlay * play);

  plays::BasePlay * finalizeSelection(plays::BasePlay * play, ateam_common::GameCommand current_game_command);

  plays::BasePlay * pickNormalStartPlay();

};

}  // namespace ateam_kenobi

#endif  // ATEAM_KENOBI__PLAY_SELECTOR_HPP_