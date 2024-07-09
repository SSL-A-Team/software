#include "our_ball_placement_play.hpp"
#include <angles/angles.h>
#include <ateam_common/robot_constants.hpp>
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

OurBallPlacementPlay::OurBallPlacementPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  pass_tactic_(createChild<tactics::Pass>("pass")),
  dribble_(createChild<skills::Dribble>("dribble")),
  easy_move_tos_(createIndexedChildren<play_helpers::EasyMoveTo>("EasyMoveTo"))
{}

stp::PlayScore OurBallPlacementPlay::getScore(const World & world)
{
  const auto & cmd = world.referee_info.running_command;
  if (cmd == ateam_common::GameCommand::BallPlacementOurs)
  {
    return stp::PlayScore::Max();
  }
  return stp::PlayScore::NaN();
}

void OurBallPlacementPlay::reset()
{
  state_ = State::Passing;
  pass_tactic_.reset();
  dribble_.reset();
  for (auto & emt : easy_move_tos_) {
    emt.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> OurBallPlacementPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);

  placement_point = world.referee_info.designated_position;

  const auto ball_dist = ateam_geometry::norm(placement_point - world.ball.pos);
  const auto ball_speed = ateam_geometry::norm(world.ball.vel);

  getOverlays().drawCircle("placement_pos", ateam_geometry::makeCircle(placement_point, 0.15), "red");

  switch (state_) {
    case State::Passing:
      if (pass_tactic_.isDone()
            || ball_dist < 1.0 
            || available_robots.size() < 2) {
        state_ = State::Placing;
        break;
      }
      runPassing(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Passing";
      break;
    case State::Placing:
      // Ball must be placed in a 0.15m radius
      if (ball_dist < 0.05 && ball_speed < 0.1) {
        state_ = State::Done;

      // Can try to pass if the ball is far away and we have enough robots
      } else if (ball_dist > 1.0 && available_robots.size() >= 2) {
        state_ = State::Passing;
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

  // TODO: Get the other robots actually out of the way. Maybe just line up on the opposite side of the field
  for (auto ind = 0ul; ind < available_robots.size(); ++ind) {
    const auto & robot = available_robots[ind];
    auto & motion_command = maybe_motion_commands[robot.id];

    if (!motion_command) {
      auto & emt = easy_move_tos_[robot.id];

      const auto ball_to_robot = robot.pos - world.ball.pos;
      const auto placement_to_robot = robot.pos - placement_point;
      if (ateam_geometry::norm(ball_to_robot) < 0.6) {
        emt.setTargetPosition(world.ball.pos + 0.7*ateam_geometry::normalize(ball_to_robot));
      } else if ((ateam_geometry::norm(placement_to_robot) < 0.6)){
         emt.setTargetPosition(placement_point + 0.7*ateam_geometry::normalize(placement_to_robot));
      } else {
        emt.setTargetPosition(robot.pos);
      }

      emt.face_point(world.ball.pos);
      maybe_motion_commands[robot.id] = emt.runFrame(robot, world);
      getPlayInfo()["Robots"][std::to_string(robot.id)] = "-";
    }
  }

  return maybe_motion_commands;
}

void OurBallPlacementPlay::runPassing(
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

  const auto receiver_robot_iter = std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToPlacement);

  auto kicker_robot_iter = std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);

  // Might be better to prioritize kicker being closer to ball instead
  // that way if we have a smart passing tactic it can kick the ball earlier
  if (receiver_robot_iter == kicker_robot_iter) {
    kicker_robot_iter++;
    if (kicker_robot_iter >= available_robots.end()) {
      kicker_robot_iter = available_robots.begin();
    }
  }

  const auto & receiver_robot = *receiver_robot_iter;
  const auto & kicker_robot = *kicker_robot_iter;

  getPlayInfo()["Assignments"]["Receiver"] = receiver_robot.id;
  getPlayInfo()["Assignments"]["Kicker"] = kicker_robot.id;

  // Offset the robot receiving the pass so the ball is on the placement point
  const auto ball_to_placement = placement_point - world.ball.pos;
  pass_tactic_.setTarget(placement_point + (kRobotRadius * ateam_geometry::normalize(ball_to_placement)));

  auto & kicker_command = *(motion_commands[kicker_robot.id] = ateam_msgs::msg::RobotMotionCommand{});
  auto & receiver_command =
    *(motion_commands[receiver_robot.id] = ateam_msgs::msg::RobotMotionCommand{});

  pass_tactic_.runFrame(world, kicker_robot, receiver_robot, kicker_command, receiver_command);
}

void OurBallPlacementPlay::runPlacing(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{

  auto byDistToBall = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.ball.pos, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto & place_robot = *std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);

  getPlayInfo()["Assignments"]["Placer"] = place_robot.id;

  dribble_.setTarget(placement_point);
  motion_commands[place_robot.id] = dribble_.runFrame(world, place_robot); 
}

void OurBallPlacementPlay::runDone(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  // TODO: Might need to add a delay for this so the dribbler slows down

  auto byDistToBall = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.ball.pos, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto place_robot = *std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);

  getPlayInfo()["Assignments"]["Placer"] = place_robot.id;

  const auto ball_to_robot = place_robot.pos - world.ball.pos;

  auto & emt = easy_move_tos_[place_robot.id];

  // TODO: check next ref command to know if we need 0.5 for force start or
  // 0.05 for our free kick
  emt.setTargetPosition(world.ball.pos + (0.5 * ateam_geometry::normalize(ball_to_robot)));
  emt.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  emt.setPlannerOptions(planner_options);
  auto command = emt.runFrame(place_robot, world);
  motion_commands[place_robot.id] = command;
}

}  // namespace ateam_kenobi::plays