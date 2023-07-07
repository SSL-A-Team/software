#include "goalie.hpp"

namespace ateam_kenobi::skills
{

Goalie::Goalie(visualization::OverlayPublisher & overlay_publisher)
: overlay_publisher_(overlay_publisher)
{
  reset();
}

void Goalie::reset()
{
  easy_move_to_.reset();
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  easy_move_to_.setPlannerOptions(planner_options);
}

void Goalie::runFrame(
  const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  const auto robot_id = world.referee_info.our_goalie_id;
  const auto & maybe_robot = world.our_robots.at(robot_id);
  if(!maybe_robot) {
    // Assigned robot is not visible
    return;
  }

  ateam_geometry::Segment goalie_line = ateam_geometry::Segment(
    ateam_geometry::Point(-(world.field.field_length/2.0) + 0.25, 0.5),
    ateam_geometry::Point(-(world.field.field_length/2.0) + 0.25, -0.5)
  );
  overlay_publisher_.drawLine("goalie_line", {goalie_line.point(0), goalie_line.point(1)}, "blue");

  easy_move_to_.setTargetPosition(ateam_geometry::NearestPointOnSegment(goalie_line, world.ball.pos));
  easy_move_to_.setFacingTowards(world.ball.pos);
  motion_commands.at(robot_id) = easy_move_to_.runFrame(maybe_robot.value(), world);
}

} // namespace ateam_kenobi::skills
