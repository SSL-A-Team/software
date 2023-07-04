#ifndef PLAYS__ANY_PLAY_HPP_
#define PLAYS__ANY_PLAY_HPP_

#include <functional>
#include <variant>
#include "test_play.hpp"

namespace ateam_kenobi::plays
{

using AnyPlay = std::variant<std::monostate, std::reference_wrapper<TestPlay>>;

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__ANY_PLAY_HPP_
