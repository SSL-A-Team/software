#include "line_kick.hpp"
#include <angles/angles.h>
#include <ateam_geometry/normalize.hpp>

namespace ateam_kenobi::skills
{

  LineKick::LineKick(visualization::OverlayPublisher &overlay_publisher)
  : overlay_publisher_(overlay_publisher),
    easy_move_to_(overlay_publisher_)
  {
  }

  ateam_geometry::Point LineKick::getAssignmentPoint(const World &world)
  {
    return getPreKickPosition(world);
  }

  ateam_msgs::msg::RobotMotionCommand LineKick::runFrame(const World &world, const Robot &robot)
  {
    const auto pre_kick_position = getPreKickPosition(world);
    const ateam_geometry::Segment kick_line(pre_kick_position, target_point_);
    overlay_publisher_.drawLine("line_kick_line", {kick_line.point(0), kick_line.point(1)}, "#FFFF007F");
    const auto distance_to_kick_line = ateam_geometry::norm(robot.pos, kick_line);
    if(distance_to_kick_line > 0.05) {
      return moveToPreKick(world, robot);
    }

    const auto robot_to_ball = world.ball.pos - robot.pos;
    const auto robot_to_ball_angle = std::atan2(robot_to_ball.y(), robot_to_ball.x());
    if(angles::shortest_angular_distance(robot.theta, robot_to_ball_angle) > 0.02) {
      return faceBall(world, robot);
    }

    return kickBall(world, robot);
  }

  ateam_geometry::Point LineKick::getPreKickPosition(const World &world)
  {
    return world.ball.pos + (kPreKickOffset * ateam_geometry::normalize(world.ball.pos - target_point_));
  }

  ateam_msgs::msg::RobotMotionCommand LineKick::moveToPreKick(const World &world, const Robot &robot)
  {
    easy_move_to_.setPlannerOptions({});
    easy_move_to_.setTargetPosition(getPreKickPosition(world));
    easy_move_to_.face_travel();
    return easy_move_to_.runFrame(robot, world);
  }
  
  ateam_msgs::msg::RobotMotionCommand LineKick::faceBall(const World &world, const Robot &robot)
  {
    easy_move_to_.setPlannerOptions({});
    easy_move_to_.setTargetPosition(robot.pos);
    easy_move_to_.face_point(world.ball.pos);
    easy_move_to_.setMaxAngularVelocity(0.5);
    return easy_move_to_.runFrame(robot, world);
  }

  ateam_msgs::msg::RobotMotionCommand LineKick::kickBall(const World &world, const Robot &robot)
  {
    const auto robot_to_ball = (world.ball.pos - robot.pos);
    easy_move_to_.setTargetPosition(world.ball.pos + (0.1 * ateam_geometry::normalize(robot_to_ball)));
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
