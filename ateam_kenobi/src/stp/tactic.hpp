#ifndef STP__TACTIC_HPP_
#define STP__TACTIC_HPP_

#include "base.hpp"

namespace ateam_kenobi::stp
{

class Tactic : public Base
{
public:
  explicit Tactic(Options options)
  : Base(options) {}

  virtual ~Tactic() = default;

};

}  // namespace ateam_kenobi::stp


#endif  // STP__TACTIC_HPP_
