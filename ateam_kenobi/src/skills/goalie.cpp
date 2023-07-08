#include "goalie.hpp"

#include <vector>
#include <optional>

#include <ranges>

namespace ateam_kenobi::skills
{

inline Robot closest_to_ray(ateam_geometry::Ray ray, std::vector<Robot> robots) {
  auto dist_from_ray = [&](const Robot & robot)->std::pair<Robot, double> {
    auto p = robot.pos;
    ateam_geometry::Point orthogonal_projection = ray.supporting_line().projection(p);
    double dist = ray.has_on(orthogonal_projection) ? ateam_geometry::norm(orthogonal_projection - p) : std::numeric_limits<double>::infinity();
    return {robot, dist};
  };
  auto min_func = [&](auto a, auto b) {return a.second < b.second;}; // Is this what mem funcs are for?

  std::vector<std::pair<Robot, double>> dist_pairs;
  std::transform(robots.begin(), robots.end(), std::back_inserter(dist_pairs), dist_from_ray);
  return std::min_element(dist_pairs.begin(), dist_pairs.end(), min_func)->first;
  
  // auto closest = robots | std::views::transform(dist_from_ray) | std::views::min(min_func);
  // return closest.first;
}

inline std::optional<Robot> closest_opponent_to_ball(const World & world) {
  auto visible_their_robots = play_helpers::getVisibleRobots(world.their_robots);
  const auto & robot_assignments = robot_assignment::assign(visible_their_robots, {world.ball.pos});
  if (!robot_assignments.empty()) {
    return world.their_robots.at(robot_assignments.begin()->first);
  } else {
    return std::nullopt;
  } 
}

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
  planner_options.use_default_obstacles = false;
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

  // 3 modes
  // IDLE: sit in the center of the goal on idle (is what woodward said I like the sit in front of the vector from ball to Center goal as an idle)
  // MARK: When an opponent is near the ball or the ball is moving at an opponent move to mark that oponents shot to the goal
  // SHOTBLOCK: When the ball is in any general velocity if it would intersect with the goal move to block. If its not in line with the goal or defense box move to block the next
  // closest attacks (MARK on ray)

  const Robot & robot = maybe_robot.value();
  
  double goal_offset = 0.05;
  ateam_geometry::Segment goal_line {
    {goal_offset + -world.field.field_length/2 , -world.field.goal_width/2},
    {goal_offset + -world.field.field_length/2 , world.field.goal_width/2}
  };

  ateam_geometry::Point goal_midpoint = goal_line.source() + ateam_geometry::normalize(goal_line.target() - goal_line.source()) * (ateam_geometry::norm(goal_line) / 2);

  // Idle case basically has to be overriden
  ateam_geometry::Point target_point = goal_midpoint;
  ateam_geometry::Ray shot_ray = {world.ball.pos, target_point - world.ball.pos};

  std::string play_info_state = "IDLE";

  // Note we just assume if the ball is coming at our goal even if friction would slow it down that its coming for us
  // Dont want to get into the tuning and estimation for friction and such yet

  // TODO INVERT ALL THESE AND MAKE THIS AN EARLY RETURN WHERE YOU CALL INTO SET MOTION STUFF BEFORE RETURNING
  // IT WLL MAKE THE LOGIC FAR LESS CANCEROUS LIKE IT WASNT WRITTEN AT 3 am
  if (ateam_geometry::norm(world.ball.vel) > 0.05) {
    shot_ray = {world.ball.pos, world.ball.vel};

    boost::optional<boost::variant<ateam_geometry::Point, ateam_geometry::Segment>> maybe_target_variant = 
      CGAL::intersection(shot_ray, goal_line);
    
    // Ball is coming at the goal SIT ON TARGET POINT
    if (maybe_target_variant.has_value()) {
      // SHOTBLOCK
      play_info_state = "SHOTBLOCK";
      const auto & target = maybe_target_variant.value();

      if (target.which() == 0) {
        target_point = boost::get<ateam_geometry::Point>(target);
      } else {
        target_point = ateam_geometry::NearestPointOnSegment(boost::get<ateam_geometry::Segment>(target), robot.pos);
        // reduction for case its some how parallel with the goal line... Should be very unlikely
      } 

    } else {
      // MARK RAY
      play_info_state = "MARK_RAY";
      // Need to refactor all of this this is identical to other mark other than decision on what to mark so marking is really a func
      Robot closest_robot = closest_to_ray(shot_ray, play_helpers::getVisibleRobots(world.their_robots));
      
      double scale_in_factor = 1.5;
      ateam_geometry::Point setback_point = goal_midpoint + (world.field.goal_depth / scale_in_factor) * (goal_line.target() - goal_line.source()).perpendicular(CGAL::Orientation::NEGATIVE);
      ateam_geometry::Segment shot_segment (setback_point, closest_robot.pos);
      shot_ray = {closest_robot.pos, setback_point};
      overlay_publisher_.drawLine("shot_segment", shot_segment, "orange");

      // seperate to function for sure with a given else lambda
      boost::optional<boost::variant<ateam_geometry::Point, ateam_geometry::Segment>> maybe_target_variant = 
      CGAL::intersection(shot_segment, goal_line);
      if (maybe_target_variant.has_value()) {
        const auto & target = maybe_target_variant.value();
        if (target.which() == 0) {
          target_point = boost::get<ateam_geometry::Point>(target);
        } else {
          target_point = ateam_geometry::NearestPointOnSegment(boost::get<ateam_geometry::Segment>(target), robot.pos);
          // reduction for case its some how parallel with the goal line... Should be very unlikely
        } 
      }
    }
  } else {
    // YEAH THIS IS JUST CANCER NOW I CANT CODE ANYMORE DESPITE DOING C++ MANY YEARS
    // hack for nearest their robot to point
    auto maybe_opponent = closest_opponent_to_ball(world);
    if (maybe_opponent) {
      Robot opponent_robot= maybe_opponent.value();

      // std::cerr << ateam_geometry::norm(opponent_robot.pos, world.ball.pos) << std::endl;

      double mark_dist = 0.5;
      overlay_publisher_.drawCircle("mark_idle_distance", ateam_geometry::makeCircle(world.ball.pos, mark_dist), "red");
      if (ateam_geometry::norm(opponent_robot.pos, world.ball.pos) < mark_dist) {
        // MARK IDLE
        play_info_state = "MARK_IDLE";
        double scale_in_factor = 1.5;
        ateam_geometry::Point setback_point = goal_midpoint + (world.field.goal_depth / scale_in_factor) * (goal_line.target() - goal_line.source()).perpendicular(CGAL::Orientation::NEGATIVE);
        ateam_geometry::Segment shot_segment (setback_point, opponent_robot.pos);
        shot_ray = {opponent_robot.pos, setback_point};
        overlay_publisher_.drawLine("shot_segment", shot_segment, "orange");

        // seperate to function for sure with a given else lambda
        boost::optional<boost::variant<ateam_geometry::Point, ateam_geometry::Segment>> maybe_target_variant = 
        CGAL::intersection(shot_segment, goal_line);
        if (maybe_target_variant.has_value()) {
          const auto & target = maybe_target_variant.value();
          if (target.which() == 0) {
            target_point = boost::get<ateam_geometry::Point>(target);
          } else {
            target_point = ateam_geometry::NearestPointOnSegment(boost::get<ateam_geometry::Segment>(target), robot.pos);
            // reduction for case its some how parallel with the goal line... Should be very unlikely
          }
        }
      }
    }
  }
  
  this->play_info_publisher_.message["GOALIE"]["mode"] = play_info_state;
  this->play_info_publisher_.send_play_message("goalie");
  // STATUS
  overlay_publisher_.drawLine("shot_ray", {world.ball.pos, target_point}, "red");
  overlay_publisher_.drawCircle("target_point", ateam_geometry::makeCircle(target_point, 0.1), "red");

  // Move
  easy_move_to_.setTargetPosition(target_point);
  easy_move_to_.face_point(world.ball.pos);
  motion_commands.at(robot_id) = easy_move_to_.runFrame(robot, world);

  return;
}


// different than nearest points as once its past something cost is inf
// TODO should have written as a transform
// template<size_t N = 16>
// ateam_geometry::Point closest_to_ray(ateam_geometry::Ray ray, array<ateam_geometry::Point, N> points) {
//   double min = std::numeric_limits<double>::infinity;
//   for(const auto & p : points) {
//     ateam_geometry::Point orthogonal_projection = ray.supporting_line().projection(p);
//     double dist = ray.has_on(orthogonal_projection)) ? ateam_geometry::norm(orthogonal_projection - p) : std::numeric_limits<double>::infinity
//     std::min(dist, min);
//   }
// }

} // namespace ateam_kenobi::skills
