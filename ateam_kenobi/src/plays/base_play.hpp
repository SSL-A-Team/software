#ifndef PLAYS__BASE_PLAY_HPP_
#define PLAYS__BASE_PLAY_HPP_

#include <array>
#include <optional>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "visualization/overlay_publisher.hpp"
#include "types/world.hpp"

namespace ateam_kenobi::plays
{

class BasePlay
{
public:
  explicit BasePlay(visualization::OverlayPublisher & overlay_publisher)
  : overlay_publisher_(overlay_publisher) {}

  virtual ~BasePlay() = default;

  virtual void reset() = 0;

  virtual std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(const World & world) = 0;

protected:
  visualization::OverlayPublisher & overlay_publisher_;

};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__BASE_PLAY_HPP_
