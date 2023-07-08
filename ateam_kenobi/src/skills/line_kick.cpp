#include "line_kick.hpp"
#include <angles/angles.h>
#include <ateam_geometry/normalize.hpp>

namespace ateam_kenobi::skills
{

LineKick::LineKick(visualization::OverlayPublisher & overlay_publisher)
: overlay_publisher_(overlay_publisher),
  easy_move_to_(overlay_publisher_)
{
}

ateam_geometry::Point LineKick::getAssignmentPoint(const World & world)
{
  return getPreKickPosition(world);
}

ateam_msgs::msg::RobotMotionCommand LineKick::runFrame(const World & world, const Robot & robot)
{
  const auto pre_kick_position = getPreKickPosition(world);

  overlay_publisher_.drawLine("LineKick_line", {pre_kick_position, target_point_}, "#FFFF007F");

  const auto distance_to_pre_kick = ateam_geometry::norm(robot.pos, pre_kick_position);
  if (distance_to_pre_kick > 0.05) {
    if (prev_state_ != State::MoveToPreKick) {
      easy_move_to_.reset();
      prev_state_ = State::MoveToPreKick;
    }
    return moveToPreKick(world, robot);
  }

  const auto robot_to_ball = world.ball.pos - robot.pos;
  const auto robot_to_ball_angle = std::atan2(robot_to_ball.y(), robot_to_ball.x());
  if (angles::shortest_angular_distance(robot.theta, robot_to_ball_angle) > 0.02) {
    if (prev_state_ != State::FaceBall) {
      easy_move_to_.reset();
      prev_state_ = State::FaceBall;
    }
    return faceBall(world, robot);
  }

  if (prev_state_ != State::KickBall) {
    easy_move_to_.reset();
    prev_state_ = State::KickBall;
  }
  return kickBall(world, robot);
}

ateam_geometry::Point LineKick::getPreKickPosition(const World & world)
{
  return world.ball.pos + (kPreKickOffset * ateam_geometry::normalize(
           world.ball.pos - target_point_));
}

ateam_msgs::msg::RobotMotionCommand LineKick::moveToPreKick(
  const World & world,
  const Robot & robot)
{
  easy_move_to_.setPlannerOptions({});
  easy_move_to_.setTargetPosition(getPreKickPosition(world));
  easy_move_to_.face_travel();
  std::vector<ateam_geometry::AnyShape> obstacles = {
    ateam_geometry::makeCircle(world.ball.pos, 0.02)
  };
  return easy_move_to_.runFrame(robot, world, obstacles);
}

ateam_msgs::msg::RobotMotionCommand LineKick::faceBall(const World & world, const Robot & robot)
{
  easy_move_to_.setPlannerOptions({});
  easy_move_to_.setTargetPosition(robot.pos);
  easy_move_to_.face_point(world.ball.pos);
  easy_move_to_.setMaxAngularVelocity(0.5);
  return easy_move_to_.runFrame(robot, world);
}

ateam_msgs::msg::RobotMotionCommand LineKick::kickBall(const World & world, const Robot & robot)
{
  const auto robot_to_ball = (world.ball.pos - robot.pos);
  easy_move_to_.setTargetPosition(
    world.ball.pos +
    (0.1 * ateam_geometry::normalize(robot_to_ball)));
  easy_move_to_.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.footprint_inflation = 0.0;
  planner_options.use_default_obstacles = false;
  easy_move_to_.setPlannerOptions(planner_options);
  auto command = easy_move_to_.runFrame(robot, world);
  command.kick = ateam_msgs::msg::RobotMotionCommand::KICK_ON_TOUCH;
  command.kick_speed = 5.0;
  return command;
}
}
