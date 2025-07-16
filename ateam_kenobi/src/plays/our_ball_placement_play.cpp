// Copyright 2024 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include "our_ball_placement_play.hpp"
#include <angles/angles.h>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

OurBallPlacementPlay::OurBallPlacementPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  pass_tactic_(createChild<tactics::Pass>("pass")),
  dribble_(createChild<skills::Dribble>("dribble"))
{
  createIndexedChildren<play_helpers::EasyMoveTo>(easy_move_tos_, "EasyMoveTo");
}

stp::PlayScore OurBallPlacementPlay::getScore(const World & world)
{
  const auto & cmd = world.referee_info.running_command;
  if (cmd == ateam_common::GameCommand::BallPlacementOurs) {
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

  placement_point_ = [this, &world]() {
      if (world.referee_info.designated_position.has_value()) {
        return world.referee_info.designated_position.value();
      } else {
        RCLCPP_WARN(
        getLogger(),
        "No designated position set in referee info, using ball position.");
        return world.ball.pos;
      }
    }();

  DrawKeepoutArea(world.ball.pos, placement_point_);

  const auto point_to_ball = world.ball.pos - placement_point_;
  const auto angle = std::atan2(point_to_ball.y(), point_to_ball.x());

  const auto ball_dist = ateam_geometry::norm(point_to_ball);
  const auto ball_speed = ateam_geometry::norm(world.ball.vel);

  getOverlays().drawCircle(
    "placement_pos", ateam_geometry::makeCircle(
      placement_point_,
      0.15), "green");

  if (state_ != State::Placing &&
    world.ball.visible &&
    ball_dist < 0.12 &&
    ball_speed < 0.04) {

    state_ = State::Done;
  }

  switch (state_) {
    case State::Passing:
      if (pass_tactic_.isDone() ||
        (ball_dist < 1.0 && ball_speed < 0.05) ||
        available_robots.size() < 2)
      {
        state_ = State::Placing;
        break;
      }
      runPassing(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Passing";
      break;
    case State::Placing:
      // Ball must be placed in a 0.15m radius
      // if (ball_dist < 0.13 && ball_speed < 0.08) {
      if (dribble_.isDone()
        || (false && ball_dist < 0.08 && ball_speed < 0.04)) {
        state_ = State::Done;

        // Can try to pass if the ball is far away and we have enough robots
      } else if (ball_dist > 1.1 && available_robots.size() >= 2) {
        pass_tactic_.reset();
        state_ = State::Passing;
      }
      runPlacing(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Placing";
      break;
    case State::Done:
      if (ball_dist > 0.14) {
        dribble_.reset();
        state_ = State::Placing;
      }
      runDone(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Done";
  }

  for (auto ind = 0ul; ind < available_robots.size(); ++ind) {
    const auto & robot = available_robots[ind];
    auto & motion_command = maybe_motion_commands[robot.id];

    if (!motion_command) {
      auto & emt = easy_move_tos_[robot.id];

      const auto placement_segment = ateam_geometry::Segment(placement_point_, world.ball.pos);
      const auto nearest_point =
        ateam_geometry::nearestPointOnSegment(placement_segment, robot.pos);

      ateam_geometry::Point target_position = robot.pos;
      if (ateam_geometry::norm(robot.pos - nearest_point) < 0.6 + kRobotRadius) {
        target_position = nearest_point +
          0.7 * ateam_geometry::Vector(std::cos(angle + M_PI / 2), std::sin(angle + M_PI / 2));

        const auto alternate_position = nearest_point +
          0.7 * ateam_geometry::Vector(std::cos(angle - M_PI / 2), std::sin(angle - M_PI / 2));

        if (ateam_geometry::norm(target_position - robot.pos) >
          ateam_geometry::norm(alternate_position - robot.pos))
        {
          target_position = alternate_position;
        }

        getPlayInfo()["Robots"][std::to_string(robot.id)] = "MOVING";
      } else {
        getPlayInfo()["Robots"][std::to_string(robot.id)] = "-";
      }

      emt.setTargetPosition(target_position);
      emt.face_point(world.ball.pos);
      maybe_motion_commands[robot.id] = emt.runFrame(robot, world);
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

  auto byDistToPlacement = [this](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(placement_point_, lhs.pos, rhs.pos) == CGAL::SMALLER;
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
  const auto ball_to_placement = placement_point_ - world.ball.pos;
  pass_tactic_.setTarget(
    placement_point_ +
    (kRobotRadius * ateam_geometry::normalize(ball_to_placement)));

  auto & kicker_command =
    *(motion_commands[kicker_robot.id] = ateam_msgs::msg::RobotMotionCommand{});
  auto & receiver_command =
    *(motion_commands[receiver_robot.id] = ateam_msgs::msg::RobotMotionCommand{});

  pass_tactic_.runFrame(world, kicker_robot, receiver_robot, kicker_command, receiver_command);

  getPlayInfo()["Pass Tactic"] = pass_tactic_.getPlayInfo();
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

  dribble_.setTarget(placement_point_);
  motion_commands[place_robot.id] = dribble_.runFrame(world, place_robot);
  getPlayInfo()["Dribbler"] = dribble_.getPlayInfo();
}

void OurBallPlacementPlay::runDone(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  // TODO(chachmu): Might need to add a delay for this so the dribbler slows down

  auto byDistToBall = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.ball.pos, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto place_robot = *std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);

  getPlayInfo()["Assignments"]["Placer"] = place_robot.id;

  const auto ball_to_robot = place_robot.pos - world.ball.pos;

  auto & emt = easy_move_tos_[place_robot.id];

  // TODO(chachmu): check next ref command to know if we need 0.5 for force start or
  // 0.05 for our free kick
  emt.setTargetPosition(world.ball.pos + (0.5 * ateam_geometry::normalize(ball_to_robot)));
  emt.face_point(world.ball.pos);
  auto command = emt.runFrame(place_robot, world);
  motion_commands[place_robot.id] = command;
}

void OurBallPlacementPlay::DrawKeepoutArea(
  const ateam_geometry::Point & ball_pos,
  const ateam_geometry::Point & placement_point)
{
  const auto point_to_ball = ball_pos - placement_point;
  const auto angle = std::atan2(point_to_ball.y(), point_to_ball.x());

  auto & overlays = getOverlays();

  const auto keepout_radius = 0.5;
  const ateam_geometry::Vector pos_offset{keepout_radius * std::cos(angle + M_PI_2),
    keepout_radius * std::sin(angle + M_PI_2)};
  const ateam_geometry::Vector neg_offset{keepout_radius * std::cos(angle - M_PI_2),
    keepout_radius * std::sin(angle - M_PI_2)};

  const ateam_geometry::Arc ball_side_arc{ball_pos, keepout_radius, neg_offset.direction(),
    pos_offset.direction()};
  const ateam_geometry::Arc point_side_arc{placement_point, keepout_radius, pos_offset.direction(),
    neg_offset.direction()};
  const ateam_geometry::Segment pos_segment{placement_point + pos_offset, ball_pos + pos_offset};
  const ateam_geometry::Segment neg_segment{placement_point + neg_offset, ball_pos + neg_offset};


  overlays.drawArc("placement_avoid_ball_arc", ball_side_arc, "Cyan");
  overlays.drawArc("placement_avoid_place_arc", point_side_arc, "Cyan");
  overlays.drawLine("placement_avoid_pos_line", {pos_segment.source(), pos_segment.target()},
      "Cyan");
  overlays.drawLine("placement_avoid_neg_line", {neg_segment.source(), neg_segment.target()},
      "Cyan");
}

}  // namespace ateam_kenobi::plays
