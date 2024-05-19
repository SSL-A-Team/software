#ifndef STP__PLAY_HPP_
#define STP__PLAY_HPP_

#include <array>
#include <limits>
#include <optional>
#include <string>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "base.hpp"
#include "types/world.hpp"

namespace ateam_kenobi::stp
{

class Play : public Base
{
public:
  explicit Play(std::string name)
  : Base(name) {}

  virtual ~Play() = default;

  /**
   * @brief Get the play's validity / confidence score
   *
   * Plays should override this with logic that checks the game state and returns a number representing if the play should be run or not.
   *
   * The play selector will prefer plays with a higher score.
   *
   * If getScore() returns NaN, the play will never be executed unless specified via play override
   *
   * @return double
   */
  virtual double getScore(const World &)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  virtual void reset() = 0;

  virtual std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(
    const World & world) = 0;

  bool isEnabled() const
  {
    return enabled_;
  }

  void setEnabled(bool value)
  {
    enabled_ = value;
  }

private:
  bool enabled_ = true;

};

}  // namespace ateam_kenobi::stp

#endif  // STP__PLAY_HPP_
