#ifndef SKILLS__LINE_KICK_HPP_
#define SKILLS__LINE_KICK_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_common/robot_constants.hpp>
#include "visualization/overlay_publisher.hpp"
#include "types/world.hpp"
#include "play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::skills
{

class LineKick
{
public:
  LineKick(visualization::OverlayPublisher & overlay_publisher);

  void setTargetPoint(ateam_geometry::Point point) {
    target_point_ = point;
  }

  ateam_geometry::Point getAssignmentPoint(const World & world);

  ateam_msgs::msg::RobotMotionCommand runFrame(const World & world, const Robot & robot);

private:
  const double kPreKickOffset = kRobotRadius + 0.1;
  visualization::OverlayPublisher & overlay_publisher_;
  ateam_geometry::Point target_point_;
  play_helpers::EasyMoveTo easy_move_to_;

  ateam_geometry::Point getPreKickPosition(const World & world);

  ateam_msgs::msg::RobotMotionCommand moveToPreKick(const World & world, const Robot & robot);

  ateam_msgs::msg::RobotMotionCommand faceBall(const World & world, const Robot & robot);

  ateam_msgs::msg::RobotMotionCommand kickBall(const World & world, const Robot & robot);
};

}

#endif  // SKILLS__LINE_KICK_HPP_
