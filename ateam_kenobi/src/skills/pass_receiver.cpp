#include "pass_receiver.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::skills
{

PassReceiver::PassReceiver(stp::Options stp_options)
: stp::Skill(stp_options),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo"))
{}

void PassReceiver::reset()
{
  assigned_bot_id_.reset();
  easy_move_to_.reset();
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runFrame(const World & world)
{
  if (!assigned_bot_id_) {
    auto available_bots = play_helpers::getAvailableRobots(world);
    play_helpers::removeGoalie(available_bots, world);
    if (available_bots.empty()) {
      return ateam_msgs::msg::RobotMotionCommand{};
    }
    assigned_bot_id_ = play_helpers::getClosestRobot(available_bots, target_).id;
  }

  const ateam_geometry::Vector text_pos_offset(0.1, 0);
  const auto & assigned_bot = world.our_robots[*assigned_bot_id_];
  if (isBallFast(world)) {
    getOverlays().drawText("state", "Passing", assigned_bot.pos + text_pos_offset, "black");
    // RCLCPP_INFO(getLogger(), "State = Passing");
    return runPass(world);
  } else if (isBallClose(world)) {
    getOverlays().drawText("state", "Post Pass", assigned_bot.pos + text_pos_offset, "black");
    // RCLCPP_INFO(getLogger(), "State = Post Pass");
    return runPostPass();
  } else {
    getOverlays().drawText("state", "Pre Pass", assigned_bot.pos + text_pos_offset, "black");
    // RCLCPP_INFO(getLogger(), "State = Pre Pass");
    return runPrePass(world);
  }
}

bool PassReceiver::isBallFast(const World & world)
{
  return ateam_geometry::norm(world.ball.vel) > 0.2;
}

bool PassReceiver::isBallClose(const World & world)
{
  assert(assigned_bot_id_);
  return ateam_geometry::norm(world.ball.pos - world.our_robots[*assigned_bot_id_].pos) < 0.11;
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runPrePass(const World & world)
{
  assert(assigned_bot_id_);
  ateam_geometry::Point destination = target_;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);
  play_helpers::removeRobotWithId(available_robots, *assigned_bot_id_);
  if (!available_robots.empty()) {
    const auto likely_kicker = play_helpers::getClosestRobot(available_robots, world.ball.pos);
    const auto kicker_ball_ray = ateam_geometry::Ray(world.ball.pos, likely_kicker.pos);
    const auto projected_target_pos = kicker_ball_ray.supporting_line().projection(target_);
    const auto target_proj_vector = projected_target_pos - target_;
    // Roughly, 90% pull towards target & 10% pull towards kick line
    destination += target_proj_vector * 0.1;
  }

  easy_move_to_.setTargetPosition(destination);
  easy_move_to_.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.use_default_obstacles = false;  //TODO(barulicm) Marietta field only?
  easy_move_to_.setPlannerOptions(planner_options);
  return easy_move_to_.runFrame(world.our_robots[*assigned_bot_id_], world);
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runPass(const World & world)
{
  assert(assigned_bot_id_);
  const auto & assigned_bot = world.our_robots[*assigned_bot_id_];
  const ateam_geometry::Ray ball_ray(world.ball.pos, world.ball.vel);
  const auto destination = ball_ray.supporting_line().projection(assigned_bot.pos);
  easy_move_to_.setTargetPosition(destination);
  easy_move_to_.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.use_default_obstacles = false;  //TODO(barulicm) Marietta field only?
  easy_move_to_.setPlannerOptions(planner_options);
  ateam_msgs::msg::RobotMotionCommand motion_command;
  motion_command = easy_move_to_.runFrame(assigned_bot, world);
  motion_command.dribbler_speed = 200;
  const auto dist_to_ball = ateam_geometry::norm(assigned_bot.pos - world.ball.pos);
  if (dist_to_ball < 0.5) {
    ateam_geometry::Vector robot_vel(motion_command.twist.linear.x, motion_command.twist.linear.y);
    robot_vel += ateam_geometry::normalize(world.ball.vel) * 0.35;
    motion_command.twist.linear.x = robot_vel.x();
    motion_command.twist.linear.y = robot_vel.y();
  }
  return motion_command;
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runPostPass()
{
  ateam_msgs::msg::RobotMotionCommand command;
  command.dribbler_speed = 200;
  command.twist.linear.x = 0;
  command.twist.linear.y = 0;
  command.twist.angular.z = 0;
  return command;
}

}  // namespace ateam_kenobi::skills
