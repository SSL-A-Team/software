#ifndef ATEAM_KENOBI__PLAY_SELECTOR_HPP_
#define ATEAM_KENOBI__PLAY_SELECTOR_HPP_

#include "plays/any_play.hpp"
#include "types/world.hpp"

namespace ateam_kenobi
{

class PlaySelector
{
public:

  plays::AnyPlay getPlay(const World & world);

private:
  plays::TestPlay test_play_;

};

}  // namespace ateam_kenobi

#endif  // ATEAM_KENOBI__PLAY_SELECTOR_HPP_