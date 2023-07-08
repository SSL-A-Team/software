#ifndef PLAYS__BASIC_122_HPP_
#define PLAYS__BASIC_122_HPP_

#include "base_play.hpp"
#include "skills/line_kick.hpp"
#include "skills/blockers.hpp"
#include "skills/goalie.hpp"

namespace ateam_kenobi::plays
{

class Basic122 : public BasePlay
{
public:
  Basic122(visualization::OverlayPublisher & op, visualization::PlayInfoPublisher & pip);

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(const World & world);

private:
  skills::LineKick striker_skill_;
  skills::Blockers blockers_skill_;
  skills::Goalie goalie_skill_;

  void assignAndRunStriker(
    std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motino_commands);

  void assignAndRunBlockers(
    std::vector<Robot> & available_robots, const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motino_commands);

};

}

#endif  // PLAYS__BASIC_122_HPP_
