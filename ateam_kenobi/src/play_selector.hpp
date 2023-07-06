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
  void * prev_play_address_ = nullptr;
  std::size_t prev_play_type_index_ = -1;

  void resetPlayIfNeeded(plays::BasePlay * play);

};

}  // namespace ateam_kenobi

#endif  // ATEAM_KENOBI__PLAY_SELECTOR_HPP_