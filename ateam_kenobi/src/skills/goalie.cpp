#include "goalie.hpp"

namespace ateam_kenobi::skills
{

Goalie::Goalie(visualization::OverlayPublisher & overlay_publisher, visualization::PlayInfoPublisher & play_info_publisher)
: overlay_publisher_(overlay_publisher), 
  play_info_publisher_(play_info_publisher), 
  easy_move_to_(overlay_publisher)
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

  ateam_geometry::Segment goal_line {
    world.field.ours.goal_posts.at(0),
    world.field.ours.goal_posts.at(1)
  };

  // also backing up as the ball comes in would be nice so we have max distance to respond
  const double scale_in_factor = 3;

  // TODO(CAVIDANO) make a general version of this for param(t) 0 <= t <= 1 by changing the division of the two
  // half of the norm of the line in the direction of target from source + the source point gives midpoint
  ateam_geometry::Point midpoint = goal_line.source() + ateam_geometry::normalize(goal_line.target() - goal_line.source()) * (ateam_geometry::norm(goal_line) / 2);
  ateam_geometry::Point target_point = midpoint + (world.field.goal_depth / scale_in_factor) * (goal_line.target() - goal_line.source()).perpendicular(CGAL::Orientation::NEGATIVE);
  // this part needs to be smarter at some point and account for states of the oppenent and vel of the ball as well as possible angles of shots
  // Probably needs to be a voronoi point vs a fix 90 deg angle

  ateam_geometry::Ray shot_ray(world.ball.pos, target_point - world.ball.pos);

  // restrict consideration of goalie to just the goalbox
  ateam_geometry::Rectangle goalbox(world.field.ours.goalie_corners.at(0), world.field.ours.goalie_corners.at(2));

  boost::optional<boost::variant<ateam_geometry::Point, ateam_geometry::Segment>> maybe_restricted_defense_segment = 
    CGAL::intersection(shot_ray, goalbox);

  if (!maybe_restricted_defense_segment.has_value()) {
    // TODO(CAVIDANO) LOG SOMETHING MESSED IF THE DEFENDER HAS NO POINT ON THE GOAL JUST STAY WHERE YOU ARE
    return;
  }
  const auto & restricted_defense_segment = maybe_restricted_defense_segment.value();
  
  // Pick point
  ateam_geometry::Point defense_point = boost::apply_visitor(IntersectVisitor(maybe_robot.value(), overlay_publisher_), restricted_defense_segment);

  // Move
  easy_move_to_.setTargetPosition(defense_point);
  easy_move_to_.setFacingTowards(world.ball.pos);
  motion_commands.at(robot_id) = easy_move_to_.runFrame(maybe_robot.value(), world);

  // STATUS
  // overlay_publisher_.drawLine("shot_ray", {world.ball.pos, target_point}, "red");
  overlay_publisher_.drawCircle(
      "defense point",
      ateam_geometry::makeCircle(defense_point, 0.2), "blue", "transparent");
  play_info_publisher_.message["Goalie"]["defense_point"] = {defense_point.x(), defense_point.y()};

  // ateam_geometry::Segment goalie_line = ateam_geometry::Segment(
  //   ateam_geometry::Point(-(world.field.field_length/2.0) + 0.25, world.field.goal_width/2.0),
  //   ateam_geometry::Point(-(world.field.field_length/2.0) + 0.25, -world.field.goal_width/2.0)
  // );
  // overlay_publisher_.drawLine("goalie_line", {goalie_line.point(0), goalie_line.point(1)}, "blue");

  // easy_move_to_.setTargetPosition(ateam_geometry::NearestPointOnSegment(goalie_line, world.ball.pos));

  // easy_move_to_.setFacingTowards(world.ball.pos);
  // motion_commands.at(robot_id) = easy_move_to_.runFrame(maybe_robot.value(), world);
}

} // namespace ateam_kenobi::skills
