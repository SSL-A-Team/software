#include "ball_placement_play.hpp"
#include <angles/angles.h>
#include <ateam_common/robot_constants.hpp>
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

BallPlacementPlay::BallPlacementPlay()
: BasePlay("BallPlacementPlay"),
  line_kick_(getOverlays().getChild("LineKick")),
  dribble_(getOverlays().getChild("Dribble"))
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

  placement_point = world.referee_info.designated_position;

  const auto ball_dist = ateam_geometry::norm(placement_point - world.ball.pos);
  const auto ball_speed = ateam_geometry::norm(world.ball.vel);

  getOverlays().drawCircle("placement_pos", ateam_geometry::makeCircle(placement_point, 0.15), "red");

  switch (state_) {
    case State::Kicking:
      if (ball_speed > 0.05) {
        state_ = State::Receiving;
        latch_receive_ = false;

      // Skip to placing the ball if we are close or have no robots to pass with
      } else if (ball_dist < 1.0 || available_robots.size() < 2) {
        state_ = State::Placing;
        break;
      }
      runKicking(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Kicking";
      break;
    case State::Receiving:
      if (ball_speed < 0.05) {
        if (ball_dist < 0.14) {
          state_ = State::Done;
        } else {
          state_ = State::Placing;
        }
      }
      runReceiving(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Receiving";
      break;
    case State::Placing:
      // Ball must be placed in a 0.15m radius
      if (ball_dist < 0.05 && ball_speed < 0.1) {
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


  const auto receiver_robot_iter = std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToPlacement);

  auto kick_robot_iter = std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);

  if (receiver_robot_iter == kick_robot_iter) {
    kick_robot_iter++;
    if (kick_robot_iter >= available_robots.end()) {
      kick_robot_iter = available_robots.begin();
    }
  }

  const auto & receiver_robot = *receiver_robot_iter;
  const auto & kick_robot = *kick_robot_iter;

  getPlayInfo()["Assignments"]["Receiver"] = receiver_robot.id;
  getPlayInfo()["Assignments"]["Kicker"] = kick_robot.id;


  auto & emt = easy_move_tos_[receiver_robot.id];
  emt.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  emt.setPlannerOptions(planner_options);

  // Offset the robot receiving the pass so the ball is on the placement point
  const auto ball_to_placement = placement_point - world.ball.pos;
  emt.setTargetPosition(placement_point + (kRobotRadius * ateam_geometry::normalize(ball_to_placement)));
  motion_commands[receiver_robot.id] = emt.runFrame(receiver_robot, world);

  if (ateam_geometry::norm(receiver_robot.pos - placement_point) < 0.1) {
    line_kick_.setTargetPoint(placement_point);
    line_kick_.setKickSpeed(0.55);
    motion_commands[kick_robot.id] = line_kick_.runFrame(world, kick_robot);
  } else {
    // Line up for the kick while we wait for the receiver
    auto & emt = easy_move_tos_[kick_robot.id];
    emt.face_point(placement_point);
    path_planning::PlannerOptions planner_options;
    emt.setPlannerOptions(planner_options);
    const auto placement_to_ball = world.ball.pos - placement_point;
    emt.setTargetPosition(world.ball.pos + (kRobotRadius + 0.7) * ateam_geometry::normalize(placement_to_ball));
    motion_commands[kick_robot.id] = line_kick_.runFrame(world, kick_robot);
  }

}

void BallPlacementPlay::runReceiving(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{

  const ateam_geometry::Ray ball_ray(world.ball.pos, world.ball.vel);

  auto byDistToPlacement = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.referee_info.designated_position, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto receiver_robot_iter = std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToPlacement);

  const auto & receiver_robot = *receiver_robot_iter;
  getPlayInfo()["Robots"][std::to_string(receiver_robot.id)] = "Receiver";

  const auto target_point = ball_ray.supporting_line().projection(receiver_robot.pos);

  getOverlays().drawCircle("receiving_pos", ateam_geometry::makeCircle(target_point, kRobotRadius));

  auto & emt = easy_move_tos_[receiver_robot.id];
  emt.setTargetPosition(target_point);
  emt.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.use_default_obstacles = false;
  emt.setPlannerOptions(planner_options);
  auto & motion_command = motion_commands[receiver_robot.id];
  motion_command = emt.runFrame(receiver_robot, world);
  motion_command->dribbler_speed = 200;

  // This doesn't work well in sim, might be useful for real
  /*
  if (CGAL::approximate_sqrt(CGAL::squared_distance(receiver_robot.pos, world.ball.pos)) < 0.5) {
    ateam_geometry::Vector robot_vel(motion_command->twist.linear.x,
      motion_command->twist.linear.y);
    robot_vel += ateam_geometry::normalize(world.ball.vel) * 0.35;
    motion_command->twist.linear.x = robot_vel.x();
    motion_command->twist.linear.y = robot_vel.y();
  }
  const auto ball_close = CGAL::approximate_sqrt(CGAL::squared_distance(receiver_robot.pos, world.ball.pos)) < 0.11;
  if (latch_receive_ || ball_close) {
    motion_command->twist.linear.x = 0;
    motion_command->twist.linear.y = 0;
    latch_receive_ = true;
  }
  */
}

void BallPlacementPlay::runPlacing(
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

  dribble_.setTargetPoint(placement_point);
  motion_commands[place_robot.id] = dribble_.runFrame(world, place_robot); 
}

void BallPlacementPlay::runDone(
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

  // have to be at least 0.5 away for a force start
  emt.setTargetPosition(world.ball.pos + (0.5 * ateam_geometry::normalize(ball_to_robot)));
  emt.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  emt.setPlannerOptions(planner_options);
  auto command = emt.runFrame(place_robot, world);
  motion_commands[place_robot.id] = command;
}

}  // namespace ateam_kenobi::plays
