#ifndef PLAYS__SPINNING_A_PLAY_HPP_
#define PLAYS__SPINNING_A_PLAY_HPP_

#include "base_play.hpp"
#include <vector>
#include "play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::plays
{

class SpinningAPlay : public BasePlay
{
public:
  SpinningAPlay();

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

private:
  const double kAngleSpeed = 0.01;
  const double kNumRotations = 5;
  std::array<play_helpers::EasyMoveTo,16> easy_move_tos_;
  std::vector<ateam_geometry::Point> base_shape_;
  double angle_;

};

} // namespace ateam_kenobi::plays

#endif  // PLAYS__SPINNING_A_PLAY_HPP_
