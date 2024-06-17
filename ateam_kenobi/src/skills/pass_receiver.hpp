#ifndef SKILLS__PASS_RECEIVER_HPP_
#define SKILLS__PASS_RECEIVER_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "play_helpers/easy_move_to.hpp"
#include "stp/skill.hpp"
#include "types/world.hpp"

namespace ateam_kenobi::skills
{

class PassReceiver : public stp::Skill
{
public:
  explicit PassReceiver(stp::Options stp_options);

  void reset();

  ateam_msgs::msg::RobotMotionCommand runFrame(const World & world, const Robot & robot);

  ateam_geometry::Point getAssignmentPoint() {
    return target_;
  }

  void setTarget(ateam_geometry::Point target) {
    target_ = target;
  }

  bool isDone() {
    return done_;
  }

private:
  ateam_geometry::Point target_;
  play_helpers::EasyMoveTo easy_move_to_;
  bool done_ = false;

  bool isBallFast(const World & world);
  bool isBallClose(const World & world, const Robot & robot);

  ateam_msgs::msg::RobotMotionCommand runPrePass(const World & world, const Robot & robot);
  ateam_msgs::msg::RobotMotionCommand runPass(const World & world, const Robot & robot);
  ateam_msgs::msg::RobotMotionCommand runPostPass();
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__PASS_RECEIVER_HPP_
