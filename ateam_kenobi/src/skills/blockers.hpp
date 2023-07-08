#ifndef SKILLS__BLOCKERS_HPP_
#define SKILLS__BLOCKERS_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "visualization/overlay_publisher.hpp"
#include "types/world.hpp"
#include "play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::skills
{

class Blockers
{
public:
  Blockers(visualization::OverlayPublisher & overlay_publisher);

  void reset();

  std::vector<ateam_geometry::Point> getAssignmentPoints(const World & world);

  std::vector<ateam_msgs::msg::RobotMotionCommand> runFrame(
    const World & world,
    const std::vector<Robot> & robots);

private:
  visualization::OverlayPublisher & overlay_publisher_;
  std::array<play_helpers::EasyMoveTo, 16> easy_move_tos_;

  std::vector<Robot> getRankedBlockableRobots(const World & world);

  ateam_geometry::Point getBlockingPosition(const World & world, const Robot & blockee);

};

}

#endif  // SKILLS__BLOCKERS_HPP_
