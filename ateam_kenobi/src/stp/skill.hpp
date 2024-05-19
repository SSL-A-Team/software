#ifndef STP__SKILL_HPP_
#define STP__SKILL_HPP_

#include "base.hpp"

namespace ateam_kenobi::stp
{

class Skill : public Base
{
public:
  explicit Skill(Options options)
  : Base(options) {}

  virtual ~Skill() = default;

};

}  // namespace ateam_kenobi::stp

#endif  // STP__SKILL_HPP_
