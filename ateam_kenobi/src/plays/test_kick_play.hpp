#ifndef PLAYS__TEST_KICK_PLAY_HPP_
#define PLAYS__TEST_KICK_PLAY_HPP_

#include "base_play.hpp"
#include "skills/line_kick.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

class TestKickPlay : public BasePlay
{
public:
  explicit TestKickPlay(
    visualization::OverlayPublisher & overlay_publisher,
    visualization::PlayInfoPublisher & play_info_publisher)
  : BasePlay(overlay_publisher, play_info_publisher),
    line_kick_skill_(overlay_publisher)
  {}

  void reset() override {}

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override
  {
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;
    const auto & robots = play_helpers::getAvailableRobots(world);
    if(robots.empty()) {
      return {};
    }
    const auto robot = robots.front();
    // aim for center of opponent goal
    line_kick_skill_.setTargetPoint(ateam_geometry::Point(world.field.field_length/2.0, 0.0));
    motion_commands[robot.id] = line_kick_skill_.runFrame(world, robot);
    play_info_publisher_.send_play_message("TestKickPlay");
    return motion_commands;
  }

private:
  skills::LineKick line_kick_skill_;

};

}

#endif  // PLAYS__TEST_KICK_PLAY_HPP_
