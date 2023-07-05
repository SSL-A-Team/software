#include "play_selector.hpp"

namespace ateam_kenobi
{

PlaySelector::PlaySelector(visualization::OverlayPublisher & overlay_publisher)
  : test_play_(overlay_publisher)
{

}

plays::BasePlay * PlaySelector::getPlay(const World & world)
{
  plays::BasePlay * selected_play;

  //
  // Play selection logic goes here
  //
  selected_play = &test_play_;

  resetPlayIfNeeded(selected_play);

  return selected_play;
}

void PlaySelector::resetPlayIfNeeded(plays::BasePlay * play)
{
  void * play_address = static_cast<void*>(play);
  if(play_address != prev_play_address_) {
    if(play != nullptr) {
      play->reset();
    }
    prev_play_address_ = play_address;
  }
}

} // namespace ateam_kenobi
