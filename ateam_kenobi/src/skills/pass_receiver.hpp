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

  ateam_msgs::msg::RobotMotionCommand runFrame(const World & world);

  void setTarget(ateam_geometry::Point target) {
    target_ = target;
  }

  void setRobotID(int id) {
    assigned_bot_id_ = id;
  }

  const std::optional<int> & getRobotID() const {
    return assigned_bot_id_;
  }

private:
  // If left unassigned, the closest non-goalie robot to target will be used.
  std::optional<int> assigned_bot_id_;
  ateam_geometry::Point target_;
  play_helpers::EasyMoveTo easy_move_to_;

  bool isBallFast(const World & world);
  bool isBallClose(const World & world);

  ateam_msgs::msg::RobotMotionCommand runPrePass(const World & world);
  ateam_msgs::msg::RobotMotionCommand runPass(const World & world);
  ateam_msgs::msg::RobotMotionCommand runPostPass();
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__PASS_RECEIVER_HPP_
