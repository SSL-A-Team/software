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
  done_ = false;
  easy_move_to_.reset();
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runFrame(const World & world, const Robot & robot)
{
  if(done_) {
    return runPostPass();
  }
  const ateam_geometry::Vector text_pos_offset(0.5, 0);
  if (isBallFast(world)) {
    return runPass(world, robot);
  } else if (isBallClose(world, robot)) {
    done_ = true;
    return runPostPass();
  } else {
    return runPrePass(world, robot);
  }
}

bool PassReceiver::isBallFast(const World & world)
{
  return ateam_geometry::norm(world.ball.vel) > 0.2;
}

bool PassReceiver::isBallClose(const World & world, const Robot & robot)
{
  return ateam_geometry::norm(world.ball.pos - robot.pos) < 0.11;
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runPrePass(const World & world, const Robot & robot)
{
  ateam_geometry::Point destination = target_;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);
  play_helpers::removeRobotWithId(available_robots, robot.id);
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
  return easy_move_to_.runFrame(robot, world);
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runPass(const World & world, const Robot & robot)
{
  assert(assigned_bot_id_);
  const ateam_geometry::Ray ball_ray(world.ball.pos, world.ball.vel);
  const auto destination = ball_ray.supporting_line().projection(robot.pos);
  easy_move_to_.setTargetPosition(destination);
  easy_move_to_.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.use_default_obstacles = false;  //TODO(barulicm) Marietta field only?
  easy_move_to_.setPlannerOptions(planner_options);
  ateam_msgs::msg::RobotMotionCommand motion_command;
  motion_command = easy_move_to_.runFrame(robot, world);
  motion_command.dribbler_speed = 200;
  const auto dist_to_ball = ateam_geometry::norm(robot.pos - world.ball.pos);
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
