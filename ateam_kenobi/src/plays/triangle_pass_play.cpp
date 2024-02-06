#include "triangle_pass_play.hpp"
#include <angles/angles.h>
#include <ateam_common/robot_constants.hpp>
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

TrianglePassPlay::TrianglePassPlay()
: BasePlay("TrianglePassPlay"),
  line_kick_(getOverlays().getChild("LineKick"))
{
  play_helpers::EasyMoveTo::CreateArray(easy_move_tos_, getOverlays().getChild("EasyMoveTo"));

  positions.emplace_back(1, 0);
  const auto angle = angles::from_degrees(120);
  CGAL::Aff_transformation_2<ateam_geometry::Kernel> rotate_transform(CGAL::ROTATION, std::sin(
      angle), std::cos(angle));
  positions.push_back(positions.back().transform(rotate_transform));
  positions.push_back(positions.back().transform(rotate_transform));
}

void TrianglePassPlay::reset()
{
  for (auto & e : easy_move_tos_) {
    e.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> TrianglePassPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);

  if (available_robots.size() < 2) {
    return maybe_motion_commands;
  }

  if (available_robots.size() > 3) {
    available_robots.erase(available_robots.begin() + 3, available_robots.end());
  }

  const auto ball_speed = ateam_geometry::norm(world.ball.vel);

  switch (state_) {
    case State::Kicking:
      if (ball_speed > 1.0) {
        state_ = State::Receiving;
      }
      runKicking(available_robots, world, maybe_motion_commands);
      break;
    case State::Receiving:
      if (ball_speed < 0.1) {
        state_ = State::Kicking;
      }
      runReceiving(available_robots, world, maybe_motion_commands);
      break;
  }

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

void TrianglePassPlay::runKicking(
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

  line_kick_.setTargetPoint(receiver_robot.pos);
  line_kick_.setKickSpeed(3.0);
  motion_commands[closest_robot.id] = line_kick_.runFrame(world, closest_robot);
}

void TrianglePassPlay::runReceiving(
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

}  // namespace ateam_kenobi::plays
