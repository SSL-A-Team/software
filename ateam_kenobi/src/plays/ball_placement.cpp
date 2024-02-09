#include "triangle_pass_play.hpp"
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
  for (auto & e : easy_move_tos_) {
    e.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> BallPlacementPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);

  if (available_robots.size() < 2) {
    // TODO: handle single robot ball placement
    return maybe_motion_commands;
  }

  placement_point = world_.referee_info.designated_point;
  const auto ball_speed = ateam_geometry::norm(world.ball.vel);

  // TODO: might need to add final state to keep all robots away from placed ball
  switch (state_) {
    case State::Kicking:
      // TODO: Add shortcut into placing if the ball is close to the placement point
      if (ball_speed > 1.0) {
        state_ = State::Receiving;
      }
      runKicking(available_robots, world, maybe_motion_commands);
      break;
    case State::Receiving:
      if (ball_speed < 0.1) {
        state_ = State::Placing;
      }
      runReceiving(available_robots, world, maybe_motion_commands);
      break;
    case State::Placing:
      runPlacing(available_robots, world, maybe_motion_commands);
  }

  // TODO: Get the other robots out of the way. Maybe just line up on the opposite side of the field
  for (auto ind = 0ul; ind < available_robots.size(); ++ind) {
    const auto & robot = available_robots[ind];
    const auto & position = positions[ind];
    auto & motion_command = maybe_motion_commands[robot.id];

    if (!motion_command) {
      auto & emt = easy_move_tos_[robot.id];
      emt.setTargetPosition(position);
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

  const auto closest_robot_iter = std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);
  auto receiver_robot_iter = closest_robot_iter + 1;
  if (receiver_robot_iter >= available_robots.end()) {
    receiver_robot_iter = available_robots.begin();
  }

  const auto & closest_robot = *closest_robot_iter;
  const auto & receiver_robot = *receiver_robot_iter;

  getPlayInfo()["Robots"][std::to_string(closest_robot.id)] = "Kicker";
  getPlayInfo()["Robots"][std::to_string(receiver_robot.id)] = "Receiver";


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
    motion_commands[closest_robot.id] = line_kick_.runFrame(world, closest_robot);
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

  getPlayInfo()["Robots"][std::to_string(receiver_robot.id)] = "Receiver";

  const auto target_point = ball_ray.supporting_line().projection(receiver_robot.pos);

  getOverlays().drawCircle("receiving_pos", ateam_geometry::makeCircle(target_point, kRobotRadius));

  auto & emt = easy_move_tos_[receiver_robot.id];
  emt.setTargetPosition(target_point);
  emt.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  emt.setPlannerOptions(planner_options);
  motion_commands[receiver_robot.id] = emt.runFrame(receiver_robot, world);
  std::cerr <<
      std::hypot(
    motion_commands[receiver_robot.id]->twist.linear.x,
    motion_commands[receiver_robot.id]->twist.linear.y) << '\n';
}

void BallPlacementPlay::runPlacing(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  // TODO: slowly drive the ball to the placement point
}

}  // namespace ateam_kenobi::plays
