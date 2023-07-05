#ifndef ATEAM_KENOBI__PLAY_SELECTOR_HPP_
#define ATEAM_KENOBI__PLAY_SELECTOR_HPP_

#include "plays/all_plays.hpp"
#include "types/world.hpp"
#include "visualization/overlay_publisher.hpp"
#include "in_play_eval.hpp"

namespace ateam_kenobi
{

class PlaySelector
{
public:
  explicit PlaySelector(visualization::OverlayPublisher & overlay_publisher);

  plays::BasePlay * getPlay(const World & world);

private:
  plays::TestPlay test_play_;
  void * prev_play_address_ = 0;
  std::size_t prev_play_type_index_ = -1;
  InPlayEval in_play_eval_;

  void resetPlayIfNeeded(plays::BasePlay * play);

};

}  // namespace ateam_kenobi

#endif  // ATEAM_KENOBI__PLAY_SELECTOR_HPP_