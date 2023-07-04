#include "play_selector.hpp"

namespace ateam_kenobi
{

  plays::AnyPlay PlaySelector::getPlay(const World &world)
  {
    return test_play_;
  }

} // namespace ateam_kenobi
