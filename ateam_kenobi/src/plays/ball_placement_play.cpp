#include "ball_placement_play.hpp"
#include <angles/angles.h>
#include <ateam_common/robot_constants.hpp>
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

BallPlacementPlay::BallPlacementPlay()
: BasePlay("BallPlacementPlay"),
  line_kick_(getOverlays().getChild("LineKick"))
{
  play_helpers::EasyMoveTo::CreateArray(easy_move_tos_, getOverlays().getChild("EasyMoveTo"));
}

void BallPlacementPlay::reset()
{
  state_ = State::Kicking;
  for (auto & e : easy_move_tos_) {
    e.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> BallPlacementPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);

  // TODO: add conversion in kenobi node to team coordinates
  // TODO: I cannot stress how important it is to not get "Thank you"ed over this again
  // TODO: No, really
  placement_point = world.referee_info.designated_position;

  const auto ball_dist = ateam_geometry::norm(placement_point - world.ball.pos);
  const auto ball_speed = ateam_geometry::norm(world.ball.vel);

  getOverlays().drawCircle("placement_pos", ateam_geometry::makeCircle(placement_point, 0.15), "red");

  switch (state_) {
    case State::Kicking:
      // TODO: Add shortcut into placing if the ball is close to the placement point
      if (ball_speed > 1.0) {
        state_ = State::Receiving;
      // Skip to placing the ball if we are close or have no robots to pass with
      } else if (ball_dist < 1.0 || available_robots.size() < 2) {
        state_ = State::Placing;
        break;
      }
      runKicking(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Kicking";
      break;
    case State::Receiving:
      if (ball_speed < 0.1) {
        state_ = State::Placing;
      }
      runReceiving(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Receiving";
      break;
    case State::Placing:
      // Ball must be placed in a 0.15m radius
      if ( ball_dist < 0.1) {
        state_ = State::Done;
      // Can try to pass if the ball is far away and we have enough robots
      } else if (ball_dist > 1 && available_robots.size() >= 2) {
        state_ = State::Kicking;
      }
      runPlacing(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Placing";
      break;
    case State::Done:
      if (ball_dist > 0.14) {
        state_ = State::Placing;
      }
      runDone(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Done";
  }

  // TODO: Get the other robots out of the way. Maybe just line up on the opposite side of the field
  for (auto ind = 0ul; ind < available_robots.size(); ++ind) {
    const auto & robot = available_robots[ind];
    auto & motion_command = maybe_motion_commands[robot.id];

    if (!motion_command) {
      auto & emt = easy_move_tos_[robot.id];
      emt.setTargetPosition(robot.pos);
      emt.face_point(world.ball.pos);
      maybe_motion_commands[robot.id] = emt.runFrame(robot, world);
      getPlayInfo()["Robots"][std::to_string(robot.id)] = "-";
    }
  }

  return maybe_motion_commands;
}

void BallPlacementPlay::runKicking(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  auto byDistToBall = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.ball.pos, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  auto byDistToPlacement = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.referee_info.designated_position, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto kick_robot_iter = std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);

  auto receiver_robot_iter = std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToPlacement);

  // I believe this can't overflow since we should never enter this state with less than 2 available robots
  if (receiver_robot_iter == kick_robot_iter) {
    receiver_robot_iter++;
  }

  const auto & kick_robot = *kick_robot_iter;
  const auto & receiver_robot = *receiver_robot_iter;

  getPlayInfo()["Assignments"]["Kicker"] = kick_robot.id;
  getPlayInfo()["Assignments"]["Receiver"] = receiver_robot.id;


  auto & emt = easy_move_tos_[receiver_robot.id];
  emt.setTargetPosition(placement_point);
  emt.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  emt.setPlannerOptions(planner_options);
  motion_commands[receiver_robot.id] = emt.runFrame(receiver_robot, world);

  // TODO: handle if the kicker is already close to the placement point
  if (ateam_geometry::norm(receiver_robot.pos - placement_point) < 0.5) {
    line_kick_.setTargetPoint(receiver_robot.pos);
    line_kick_.setKickSpeed(3.0);
    motion_commands[kick_robot.id] = line_kick_.runFrame(world, kick_robot);
  } else {
    // TODO: line up for kick
  }

}

void BallPlacementPlay::runReceiving(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  const ateam_geometry::Ray ball_ray(world.ball.pos, world.ball.vel);

  auto byDistToBallRay = [&ball_ray](const Robot & lhs, const Robot & rhs) {
      return CGAL::squared_distance(lhs.pos, ball_ray) <
             CGAL::squared_distance(rhs.pos, ball_ray);
    };

  const auto & receiver_robot = *std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBallRay);

  getPlayInfo()["Assignments"]["Receiver"] = receiver_robot.id;

  const auto target_point = ball_ray.supporting_line().projection(receiver_robot.pos);

  getOverlays().drawCircle("receiving_pos", ateam_geometry::makeCircle(target_point, kRobotRadius));

  auto & emt = easy_move_tos_[receiver_robot.id];
  emt.setTargetPosition(target_point);
  emt.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  emt.setPlannerOptions(planner_options);
  motion_commands[receiver_robot.id] = emt.runFrame(receiver_robot, world);
}

void BallPlacementPlay::runPlacing(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  // TODO: Make dribble skill since this really doesn't work

  auto byDistToBall = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.ball.pos, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto & place_robot = *std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);

  getPlayInfo()["Assignments"]["Placer"] = place_robot.id;

  auto & emt = easy_move_tos_[place_robot.id];

  const auto robot_to_placement = placement_point - place_robot.pos;

  emt.setTargetPosition(placement_point + (kRobotRadius * ateam_geometry::normalize(robot_to_placement)));
  emt.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  emt.setPlannerOptions(planner_options);
  auto command = emt.runFrame(place_robot, world);
  command.dribbler_speed = 200; // I have no clue what to use for this
  motion_commands[place_robot.id] = command;
}

void BallPlacementPlay::runDone(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  // TODO: step away from the placement point but stay nearby in case the ball rolls out

  auto byDistToBall = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.ball.pos, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto place_robot = *std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);


  getPlayInfo()["Assignments"]["Placer"] = place_robot.id;

  const auto ball_to_center = ateam_geometry::Point(0, 0) - world.ball.pos;

  auto & emt = easy_move_tos_[place_robot.id];

  // have to be at least 0.5 away for a force start
  emt.setTargetPosition(CGAL::ORIGIN + (0.6 * ateam_geometry::normalize(ball_to_center)));
  emt.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  emt.setPlannerOptions(planner_options);
  auto command = emt.runFrame(place_robot, world);
  motion_commands[place_robot.id] = command;
}

}  // namespace ateam_kenobi::plays
