// Copyright 2026 A Team
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

#include "defense_clear_play.hpp"
#include <vector>
#include "core/play_helpers/possession.hpp"
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/window_evaluation.hpp"
#include "core/play_helpers/robot_assignment.hpp"

namespace ateam_kenobi::plays
{

DefenseClearPlay::DefenseClearPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  kick_(createChild<skills::LineKick>("clearer")),
  defense_tactic_(createChild<tactics::StandardDefense>("defense")),
  blockers_(createChild<tactics::Blockers>("blockers"))
{
  blockers_.setMaxBlockerCount(3);
}

stp::PlayScore DefenseClearPlay::getScore(const World & world)
{

  if (!world.in_play &&
    world.referee_info.running_command != ateam_common::GameCommand::ForceStart &&
    world.referee_info.running_command != ateam_common::GameCommand::NormalStart &&
    world.referee_info.running_command != ateam_common::GameCommand::DirectFreeOurs)
  {
    // Defense isn't running
    return stp::PlayScore::NaN();
  }

  if (!shouldDefenseClearBall(world)) {
    // defense tactic doesn't want to clear the ball
    return stp::PlayScore::NaN();
  }

  return stp::PlayScore::Max();
}

void DefenseClearPlay::reset()
{
  kick_.Reset();
  defense_tactic_.reset();

  shot_type_ = ShotType::NoShot;
  kicked_to_target_ = std::nullopt;
  clearing_keepout_zone_ = std::nullopt;
  clearing_target_segment_ = std::nullopt;
}

std::array<std::optional<RobotCommand>, 16> DefenseClearPlay::runFrame(
  const World & world)
{
  std::array<std::optional<RobotCommand>, 16> motion_commands;

  // GOALIE
  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);
  // Run only the goalie
  defense_tactic_.runFrame(world, std::vector<Robot>{}, motion_commands);

  auto num_robots = available_robots.size();
  if (num_robots <= 0) {
    return motion_commands;
  }

  play_helpers::GroupAssignmentSet groups;

  const auto defense_points = defense_tactic_.getAssignmentPoints(world);
  const ateam_geometry::Point clear_target_point = getClearTargetPoint(world);
  const ateam_geometry::Point guard_point = getGuardPoint(world);

  // CLEARER

  // These assignment points are bad
  auto clearer_assignment_point = defense_points[0];
  switch(shot_type_) {
    case ShotType::StraightLine:
      clearer_assignment_point = defense_points[0];
      break;
    case ShotType::DownfieldLine:
      clearer_assignment_point = defense_points[0];
      break;
    case ShotType::NoShot:
      clearer_assignment_point = defense_points[0];
      break;
  }

  const auto& clearer = play_helpers::getClosestRobot(available_robots, clearer_assignment_point);
  motion_commands[clearer.id] = runClearingRobot(world, clearer, defense_points[0]);

  play_helpers::removeRobotWithId(available_robots, clearer.id);
  num_robots -= 1;
  if (num_robots <= 0) {
    return motion_commands;
  }

  // DEFENDER

  auto defender_assignment_point = guard_point;
  auto defender = play_helpers::getClosestRobot(available_robots, defender_assignment_point);

  // Only use the defender to guard if it doesn't have to cross the keepout zone
  bool defender_is_guarding = !enforceKeepoutZone(world, defender, defender_assignment_point);
  getPlayInfo()["defender is guarding"] = defender_is_guarding;
  if (!defender_is_guarding) {
    defender_assignment_point = defense_points[1];
    defender = play_helpers::getClosestRobot(available_robots, defender_assignment_point);
  }
  motion_commands[defender.id] = runPositionBasedRobot(world, defender, defender_assignment_point);

  play_helpers::removeRobotWithId(available_robots, defender.id);
  num_robots -= 1;
  if (num_robots <= 0) {
    return motion_commands;
  }

  // RECEIVER

  // Try to position a robot to receive the cleared ball
  bool added_receiver = false;
  if (num_robots > 0) {
    groups.AddPosition("receiver", clear_target_point);
    added_receiver = true;
    num_robots--;
  }

  // GUARD

  // If the defender isn't guarding then a blocker should be assigned to guard the clear
  bool added_guard = false;
  if (num_robots > 0 && !defender_is_guarding) {
    getPlayInfo()["play_guarding"] = true;
    groups.AddPosition("guard", guard_point);
    added_guard = true;
    num_robots--;
  }

  // BLOCKERS

  // Block opposing robots from receiving passes
  blockers_.setMaxBlockerCount(num_robots);
  getPlayInfo()["num_blockers"] = num_robots;
  groups.AddGroup("blockers", blockers_.getAssignmentPoints(world));

  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  if (added_receiver) {
    const auto maybe_receiver = assignments.GetPositionAssignment("receiver");
    if (maybe_receiver.has_value()) {
      motion_commands[maybe_receiver.value().id] = runReceivingRobot(world, maybe_receiver.value(), clear_target_point);
    }
  }

  if (added_guard) {
    const auto maybe_guard = assignments.GetPositionAssignment("guard");
    if (maybe_guard.has_value()) {
      motion_commands[maybe_guard.value().id] = runPositionBasedRobot(world, maybe_guard.value(), guard_point);
    }
  }

  std::vector<Robot> blockers = assignments.GetGroupFilledAssignments("blockers");
  const auto blocker_commands =
    blockers_.runFrame(world, blockers);
  for (auto robot_ind = 0ul; robot_ind < blockers.size(); ++robot_ind) {
    auto& motion_command = motion_commands[blockers[robot_ind].id];
    motion_command = blocker_commands[robot_ind];

    auto* intent = std::get_if<motion::intents::PositionFacing>(&motion_command.value().motion_intent);
    enforceKeepoutZone(world, blockers[robot_ind], intent->position);
  }

  return motion_commands;
}

RobotCommand DefenseClearPlay::runClearingRobot(const World & world, const Robot & robot,
  const ateam_geometry::Point defense_point) {

  if (shot_type_ != ShotType::NoShot) {
    const auto kick_target = getClearTargetPoint(world);

    // TODO(chachmu): this doesn't really work very well
    if (kick_.IsDone()) {
      kicked_to_target_ = kick_target;
      kick_.Reset();
    }

    kick_.SetKickSpeed(2.3);
    kick_.SetTargetPoint(kick_target);

    auto command = kick_.RunFrame(world, robot);
    if(auto intent = std::get_if<motion::intents::Position>(&command.motion_intent);
      intent != nullptr)
    {
      intent->planner_options.use_default_obstacles = false;
    }
    if(auto intent = std::get_if<motion::intents::PositionFacing>(&command.motion_intent);
      intent != nullptr)
    {
      intent->planner_options.use_default_obstacles = false;
    }
    // ForwardPlayInfo(kick_);
    return command;
  }

  motion::intents::Position intent;
  intent.position = defense_point;
  intent.heading = 0;
  intent.planner_options.avoid_ball = false;

  RobotCommand command;
  command.motion_intent = intent;

  return command;
}

RobotCommand DefenseClearPlay::runPositionBasedRobot(const World & world, const Robot & robot,
  const ateam_geometry::Point target_pos)
{
  motion::intents::PositionFacing intent;
  intent.position = target_pos;
  intent.face_target = world.ball.pos;
  intent.planner_options.avoid_ball = false;

  enforceKeepoutZone(world, robot, intent.position);

  RobotCommand command;
  command.motion_intent = intent;

  return command;
}

RobotCommand DefenseClearPlay::runReceivingRobot(const World & world, const Robot & robot,
  const ateam_geometry::Point target_pos)
{
  // Could use the pass receiver skill here
  (void) robot;
  motion::intents::PositionFacing intent;
  intent.position = target_pos;
  intent.face_target = world.ball.pos;
  intent.planner_options.avoid_ball = false;

  RobotCommand command;
  command.motion_intent = intent;

  return command;
}

ateam_geometry::Point DefenseClearPlay::getClearTargetPoint(const World & world) {
  if (kicked_to_target_.has_value()) {
    shot_type_ = ShotType::NoShot; // already kicked, just maintain defense
    return kicked_to_target_.value();
  }

  double opponent_distance = 20.0; // This is fine I guess :/

  auto visible_opponents = play_helpers::getVisibleRobots(world.their_robots);
  std::ranges::sort(
    visible_opponents, [&world](const Robot & r1, const Robot & r2) {
      const auto r1_dist = ateam_geometry::norm(world.ball.pos, r1.pos);
      const auto r2_dist = ateam_geometry::norm(world.ball.pos, r2.pos);
      return r1_dist < r2_dist;
    });

  if (!visible_opponents.empty()) {
    opponent_distance = ateam_geometry::norm(world.ball.pos - visible_opponents[0].pos);
  }

  ateam_geometry::Point kick_target;
  ateam_geometry::Vector shot_center_line;

  if (opponent_distance < clear_downfield_distance_threshold_) {
    shot_type_ = ShotType::StraightLine;
    // Shot extends straight away from our goal
    shot_center_line = world.ball.pos - ateam_geometry::Point(-world.field.field_length / 2.0, 0.0);
  } else {
    shot_type_ = ShotType::DownfieldLine;
    // Shot goes straight downfield of the ball;
    shot_center_line = ateam_geometry::Point(world.field.field_length, world.ball.pos.y()) - world.ball.pos;
  }

  CGAL::Aff_transformation_2<ateam_geometry::Kernel> neg_rotate(CGAL::ROTATION,
    sin(-clear_window_angular_width_/2.0), cos(-clear_window_angular_width_/2.0));
  const auto negative_vector = shot_center_line.transform(neg_rotate);

  CGAL::Aff_transformation_2<ateam_geometry::Kernel> pos_rotate(CGAL::ROTATION,
    sin(clear_window_angular_width_/2.0), cos(clear_window_angular_width_/2.0));
  const auto positive_vector = shot_center_line.transform(pos_rotate);

  const ateam_geometry::Segment target_seg(
    world.ball.pos + clear_receiver_distance_ * ateam_geometry::normalize(negative_vector),
    world.ball.pos + clear_receiver_distance_ * ateam_geometry::normalize(positive_vector)
  );
  clearing_keepout_zone_ = target_seg;

  auto robots = play_helpers::getVisibleRobots(world.our_robots);
  for (Robot& our_robot : robots) {
    // ignore robots near the theoretical target distance when checking for open shot
    if (ateam_geometry::norm(our_robot.pos - world.ball.pos) > (clear_receiver_distance_ - 0.8)) {
      play_helpers::removeRobotWithId(robots, our_robot.id);
    }
  }

  std::ranges::copy(play_helpers::getVisibleRobots(world.their_robots), std::back_inserter(robots));

  const auto windows = play_helpers::window_evaluation::getWindows(
    target_seg, world.ball.pos,
    robots);
  play_helpers::window_evaluation::drawWindows(
    windows, world.ball.pos, getOverlays().getChild(
      "windows"));
  const auto largest_window = play_helpers::window_evaluation::getLargestWindow(windows);


  if (largest_window && largest_window.value().squared_length() > std::pow(clear_window_min_kick_width, 2)) {
    clearing_target_segment_ = *largest_window;
    const auto kick_vector = CGAL::midpoint(*largest_window) - world.ball.pos;
    return world.ball.pos + clear_receiver_distance_ * ateam_geometry::normalize(kick_vector);
  } else {
    shot_type_ = ShotType::NoShot;
    clearing_target_segment_ = std::nullopt;

    // Send the receiver to the less restrictive target
    const ateam_geometry::Point goal_center(-world.field.field_length / 2, 0);
    const auto kick_vector = world.ball.pos - goal_center;
    return world.ball.pos + clear_receiver_distance_ * ateam_geometry::normalize(kick_vector);
  }
}

ateam_geometry::Point DefenseClearPlay::getGuardPoint(const World & world) {

  auto visible_opponents = play_helpers::getVisibleRobots(world.their_robots);
  std::ranges::sort(
    visible_opponents, [&world](const Robot & r1, const Robot & r2) {
      const auto r1_dist = ateam_geometry::norm(world.ball.pos, r1.pos);
      const auto r2_dist = ateam_geometry::norm(world.ball.pos, r2.pos);
      return r1_dist < r2_dist;
    });
  if (visible_opponents.empty()) {
    // Handle this case

    return ateam_geometry::Point();
  }
  const auto opponent = visible_opponents[0];
  getPlayInfo()["guarding id"] = opponent.id;
  getPlayInfo()["opponent_dist"] = ateam_geometry::norm(opponent.pos - world.ball.pos);

  ateam_geometry::Point guard_pos = world.ball.pos +
    ((kRobotDiameter * 1.5) * ateam_geometry::normalize(opponent.pos - world.ball.pos));

  // TODO(chachmu): check this more carefully
  // Consider how far the opponent is from the ball
  return guard_pos;
}

bool DefenseClearPlay::shouldDefenseClearBall(const World & world)
{
  if (ateam_geometry::norm(world.ball.vel) > ball_velocity_clear_threshold_) {
    return false;
  }

  if (play_helpers::WhoHasPossession(world) == play_helpers::PossessionResult::Theirs) {
    return false;
  }

  const double defense_x_front = (-world.field.field_length / 2) + world.field.defense_area_depth;

  // Check if ball is just outside front of defense zone
  bool x_in_forward_zone = world.ball.pos.x() > defense_x_front &&
    world.ball.pos.x() < defense_x_front + defense_zone_clear_threshold_;
  bool y_in_forward_zone = abs(world.ball.pos.y()) <
    (world.field.defense_area_width / 2) + defense_zone_clear_threshold_;

  if (x_in_forward_zone && y_in_forward_zone) {
    return true;
  }

  // Check if ball is just to the sides of the defense zone
  bool x_in_side_zone = world.ball.pos.x() < defense_x_front + defense_zone_clear_threshold_;
  bool y_in_side_zone = abs(world.ball.pos.y()) > (world.field.defense_area_width / 2) &&
    abs(world.ball.pos.y()) < (world.field.defense_area_width / 2) + defense_zone_clear_threshold_;

  if (x_in_side_zone && y_in_side_zone) {
    return true;
  }

  // Ball not in front or side zone
  return false;
}

bool DefenseClearPlay::enforceKeepoutZone(const World & world, const Robot & robot,
  ateam_geometry::Point & target_point) {

    if (!clearing_keepout_zone_.has_value()) {
      return false;
    }

    bool crosses_over_keepout = false;

    const auto shot_line = ateam_geometry::Segment(CGAL::midpoint(clearing_keepout_zone_.value()), world.ball.pos);
    const auto nearest_point = ateam_geometry::nearestPointOnSegment(shot_line, target_point);

    auto along_track_dist = ateam_geometry::norm(nearest_point - world.ball.pos);
    if (along_track_dist > clear_receiver_distance_) {
      // Past the end of the keepout zone
      return false;
    }

    auto cross_track_vec = target_point - nearest_point;
    const auto cross_track_dist = ateam_geometry::norm(cross_track_vec);

    // Don't let the robot cross over the keepout zone
    // This check works because the robots also shouldn't be allowed to go behind
    // The clearing bot into the defense zone so it would have to path through the zone
    if (abs(ateam_geometry::ShortestAngleBetween(robot.pos - nearest_point, cross_track_vec)) > M_PI/2) {
      crosses_over_keepout = true;
      cross_track_vec *= -1; // Stay on the side of the robot
    }

    const double angle = std::atan2(cross_track_dist, along_track_dist);

    // Inside the keepout zone or want to cross over
    if (abs(angle) < clear_window_angular_width_ || crosses_over_keepout) {
      if (along_track_dist < 1.5 * kRobotDiameter) {
        // Keep the robot from getting so close it hits the ball
        along_track_dist = 1.5 * kRobotDiameter;
      }
      const double correct_dist = (0.5 * along_track_dist * std::tan(clear_window_angular_width_)) + kRobotRadius;
      target_point = nearest_point + correct_dist * ateam_geometry::normalize(cross_track_vec);
    }

    return crosses_over_keepout;
}

}  // namespace ateam_kenobi::plays
