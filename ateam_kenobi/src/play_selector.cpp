#include "play_selector.hpp"

namespace ateam_kenobi
{

plays::AnyPlay PlaySelector::getPlay(const World & world)
{
  plays::AnyPlay selected_play;

  //
  // Play selection logic goes here
  //
  selected_play = test_play_;

  resetPlayIfNeeded(selected_play);

  return selected_play;
}

void PlaySelector::resetPlayIfNeeded(plays::AnyPlay & play)
{
  if (play.index() != prev_play_type_index_) {
    std::visit(
      [](auto & p) {
        using PlayType = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<PlayType, std::monostate>) {
          return;
        } else {
           p.get().reset();
        }
      }, play);
    prev_play_type_index_ = play.index();
  }
}

} // namespace ateam_kenobi
