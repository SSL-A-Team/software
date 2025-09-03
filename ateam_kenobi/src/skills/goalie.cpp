// Copyright 2021 A Team
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


#include "goalie.hpp"
#include <algorithm>
#include <limits>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include <ateam_common/time.hpp>
#include <ateam_geometry/nearest_point.hpp>
#include <ateam_geometry/angles.hpp>
#include "core/play_helpers/window_evaluation.hpp"
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/possession.hpp"

namespace ateam_kenobi::skills
{

Goalie::Goalie(stp::Options stp_options)
: stp::Skill(stp_options),
  kick_(createChild<skills::LineKick>("Kick"))
{
  reset();
  kick_.SetUseDefaultObstacles(false);
  default_planner_options_.avoid_ball = false;
  default_planner_options_.use_default_obstacles = false;
  // kick_.setPreKickOffset(kRobotRadius + kBallRadius + 0.04);
}

void Goalie::reset()
{
  prev_ball_in_def_area_ = false;
}

void Goalie::runFrame(
  const World & world,
  std::array<std::optional<RobotCommand>, 16> & motion_commands)
{
  Ball ball_state = world.ball;
  if(world.ball.visible) {
    const auto maybe_closest_enemy = getClosestEnemyRobotToBall(world);
    if(maybe_closest_enemy) {
      last_enemy_id_closest_to_ball_ = maybe_closest_enemy->id;
    }
  } else if (last_enemy_id_closest_to_ball_) {
    glueBallToLastClosestEnemy(ball_state, world);
  }

  if(!world.ball.visible) {
    getOverlays().drawCircle("ball", ateam_geometry::makeCircle(ball_state.pos, kRobotRadius),
        "orange", "#00000000");
  }

  const auto robot_id = world.referee_info.our_goalie_id;
  const auto & robot = world.our_robots.at(robot_id);
  if (!robot.IsAvailable()) {
    // Assigned robot is not available
    return;
  }

  RobotCommand motion_command;
  motion_command.motion_intent.planner_options = default_planner_options_;

  const auto ball_in_def_area = isBallInDefenseArea(world, ball_state);
  if(ball_in_def_area && !prev_ball_in_def_area_) {
    ball_entered_def_area_time_ = world.current_time;
  }
  prev_ball_in_def_area_ = ball_in_def_area;
  const auto time_in_def_area = ateam_common::TimeDiffSeconds(world.current_time,
      ball_entered_def_area_time_);

  if (isBallHeadedTowardsGoal(world, ball_state)) {
    getPlayInfo()["State"] = "Block Ball";
    motion_command = runBlockBall(world, robot, ball_state);
  } else if (doesOpponentHavePossesion(world)) {
    getPlayInfo()["State"] = "Block Shot";
    motion_command = runBlockShot(world, robot, ball_state);
  } else if (ball_in_def_area && time_in_def_area > 5.5) {
    getPlayInfo()["State"] = "Side Eject";
    motion_command = runSideEjectBall(world, robot);
  } else if (ball_in_def_area) {
    getPlayInfo()["State"] = "Clear Ball";
    motion_command = runClearBall(world, robot, ball_state);
  } else {
    getPlayInfo()["State"] = "Default";
    motion_command = runDefaultBehavior(world, robot, ball_state);
  }

  motion_commands[robot_id] = motion_command;
}

bool Goalie::doesOpponentHavePossesion(const World & world)
{
  const auto possession = play_helpers::WhoHasPossession(world);
  return possession == play_helpers::PossessionResult::Theirs ||
         possession == play_helpers::PossessionResult::TheirsWeak ||
         possession == play_helpers::PossessionResult::Tied ||
         possession == play_helpers::PossessionResult::TiedWeak;
}

bool Goalie::isBallHeadedTowardsGoal(const World & world, const Ball & ball_state)
{
  if (ball_state.vel.x() >= -0.1) {
    return false;
  }

  const ateam_geometry::Ray ball_vel_ray(ball_state.pos, ball_state.vel);

  const auto goal_inflation = 1.1;

  const ateam_geometry::Segment goal_segment(
    ateam_geometry::Point(
      -world.field.field_length / 2,
      -(world.field.goal_width * goal_inflation) / 2),
    ateam_geometry::Point(
      -world.field.field_length / 2,
      (world.field.goal_width * goal_inflation) / 2)
  );

  return CGAL::do_intersect(ball_vel_ray, goal_segment);
}

bool Goalie::isBallInDefenseArea(const World & world, const Ball & ball_state)
{
  ateam_geometry::Rectangle our_defense_area(
    *CGAL::top_vertex_2(
      world.field.ours.defense_area_corners.begin(),
      world.field.ours.defense_area_corners.end()),
    *CGAL::bottom_vertex_2(
      world.field.ours.defense_area_corners.begin(),
      world.field.ours.defense_area_corners.end()));

  return CGAL::do_intersect(ball_state.pos, our_defense_area);
}

RobotCommand Goalie::runDefaultBehavior(
  const World & world,
  const Robot &, const Ball & ball_state)
{
  auto goal_line_offset = 0.25;
  if (world.referee_info.running_command == ateam_common::GameCommand::PreparePenaltyTheirs ||
    (world.referee_info.running_command == ateam_common::GameCommand::NormalStart &&
    world.referee_info.prev_command == ateam_common::GameCommand::PreparePenaltyTheirs))
  {
    goal_line_offset = kRobotRadius - 0.03;
  }
  const ateam_geometry::Segment goalie_line(
    ateam_geometry::Point(
      -(world.field.field_length / 2) + goal_line_offset,
      world.field.goal_width / 2),
    ateam_geometry::Point(
      -(world.field.field_length / 2) + goal_line_offset,
      -world.field.goal_width / 2));

  RobotCommand command;
  command.motion_intent.planner_options = default_planner_options_;
  command.motion_intent.linear = motion::intents::linear::PositionIntent{ateam_geometry::nearestPointOnSegment(goalie_line, ball_state.pos)};
  command.motion_intent.angular = motion::intents::angular::HeadingIntent{M_PI_2};
  command.motion_intent.obstacles = getCustomObstacles(world);
  return command;
}

RobotCommand Goalie::runBlockShot(
  const World & world, const Robot & goalie,
  const Ball & ball_state)
{
  const auto & ball_pos = ball_state.pos;
  std::array<double, 16> distances;
  std::transform(
    world.their_robots.begin(), world.their_robots.end(), distances.begin(),
    [&ball_pos](const auto & robot) {
      if (!robot.visible) {
        return std::numeric_limits<double>::infinity();
      }
      return CGAL::approximate_sqrt(CGAL::squared_distance(ball_pos, robot.pos));
    });
  const auto min_distance_iter = std::min_element(distances.begin(), distances.end());
  if (std::isinf(*min_distance_iter)) {
    // Should only be true if there are no opponent robots
    getPlayInfo()["DEFAULT BEHAVIOR"] = "FALLTHROUGH";
    return runDefaultBehavior(world, goalie, ball_state);
  }
  const auto opponent_id = std::distance(distances.begin(), min_distance_iter);
  const auto opponent_bot = world.their_robots[opponent_id];
  const auto opponent_pos = world.their_robots[opponent_id].pos;
  // const auto opponent_ball_vector = ball_state.pos - opponent_pos;
  const auto opponent_ball_vector = ateam_geometry::directionFromAngle(opponent_bot.theta).vector();

  getOverlays().drawLine("Shot Line",
    {opponent_bot.pos, opponent_bot.pos + (10.0 * opponent_ball_vector)}, "DarkOrange");

  const ateam_geometry::Ray shot_ray(opponent_pos, opponent_ball_vector);

  const auto goalie_line_x = -(world.field.field_length / 2) + kRobotRadius;

  const ateam_geometry::Segment goalie_line_extended(
    ateam_geometry::Point(goalie_line_x, world.field.field_width / 2),
    ateam_geometry::Point(goalie_line_x, -world.field.field_width / 2));

  const ateam_geometry::Segment goalie_line(
    ateam_geometry::Point(goalie_line_x, world.field.goal_width / 2),
    ateam_geometry::Point(goalie_line_x, -world.field.goal_width / 2));

  const auto maybe_intersection = CGAL::intersection(shot_ray, goalie_line_extended);

  if (!maybe_intersection) {
    return runDefaultBehavior(world, goalie, ball_state);
  }

  ateam_geometry::Point shot_point_on_extended_goalie_line;

  if (const auto * point = boost::get<ateam_geometry::Point>(&*maybe_intersection)) {
    shot_point_on_extended_goalie_line = *point;
  } else if (const auto * segment = boost::get<ateam_geometry::Segment>(&*maybe_intersection)) {
    shot_point_on_extended_goalie_line = segment->source();
  }

  RobotCommand command;
  command.motion_intent.planner_options = default_planner_options_;
  command.motion_intent.linear = motion::intents::linear::PositionIntent{
    ateam_geometry::nearestPointOnSegment(
      goalie_line,
      shot_point_on_extended_goalie_line)};
  command.motion_intent.angular = motion::intents::angular::HeadingIntent{M_PI_2};
  command.motion_intent.obstacles = getCustomObstacles(world);
  return command;
}

RobotCommand Goalie::runBlockBall(
  const World & world, const Robot & goalie,
  const Ball & ball_state)
{
  const ateam_geometry::Ray ball_vel_ray(ball_state.pos, ball_state.vel);

  const auto goalie_line_x = -(world.field.field_length / 2) + kRobotRadius;

  const ateam_geometry::Segment goalie_line(
    ateam_geometry::Point(goalie_line_x, world.field.goal_width / 2),
    ateam_geometry::Point(goalie_line_x, -world.field.goal_width / 2));

  const auto maybe_intersection = CGAL::intersection(ball_vel_ray, goalie_line);

  if (!maybe_intersection) {
    return runDefaultBehavior(world, goalie, ball_state);
  }

  ateam_geometry::Point target_point{0.0, 0.0};

  if (const auto * point = boost::get<ateam_geometry::Point>(&*maybe_intersection)) {
    target_point = *point;
  } else if (const auto * segment = boost::get<ateam_geometry::Segment>(&*maybe_intersection)) {
    target_point = segment->source();
  }

  RobotCommand command;
  command.motion_intent.planner_options = default_planner_options_;
  command.motion_intent.planner_options.footprint_inflation = -0.05;
  command.motion_intent.linear = motion::intents::linear::PositionIntent{target_point};
  command.motion_intent.angular = motion::intents::angular::HeadingIntent{M_PI_2};
  command.motion_intent.obstacles = getCustomObstacles(world);
  return command;
}

RobotCommand Goalie::runClearBall(
  const World & world, const Robot & goalie,
  const Ball & ball_state)
{
  const ateam_geometry::Segment target_seg(ateam_geometry::Point(0, -world.field.field_width / 2),
    ateam_geometry::Point(0, world.field.field_width / 2));

  auto robots = play_helpers::getVisibleRobots(world.our_robots);
  play_helpers::removeRobotWithId(robots, goalie.id);
  std::ranges::copy(play_helpers::getVisibleRobots(world.their_robots), std::back_inserter(robots));

  const auto windows = play_helpers::window_evaluation::getWindows(
    target_seg, ball_state.pos,
    robots);
  play_helpers::window_evaluation::drawWindows(
    windows, ball_state.pos, getOverlays().getChild(
      "windows"));
  const auto largest_window = play_helpers::window_evaluation::getLargestWindow(windows);

  ateam_geometry::Point target_point;

  if (largest_window) {
    target_point = CGAL::midpoint(*largest_window);
  } else {
    // If no window exists, just kick away from the goal
    const ateam_geometry::Point goal_center(-world.field.field_length / 2, 0);
    const auto kick_vector = ball_state.pos - goal_center;
    target_point = ball_state.pos + (kick_vector * 3);
  }

  if (kick_.IsDone()) {
    kick_.Reset();
  }

  kick_.SetKickSpeed(1.3);
  kick_.SetTargetPoint(target_point);

  const auto command = kick_.RunFrame(world, goalie);
  ForwardPlayInfo(kick_);
  return command;
}


RobotCommand Goalie::runSideEjectBall(
  const World & world, const Robot & goalie)
{
  auto left_count = 0;
  auto right_count = 0;
  for(const auto & bot : world.their_robots) {
    if(!bot.visible) {continue;}
    if(bot.pos.y() < 0.0) {
      ++right_count;
    } else {
      ++left_count;
    }
  }

  const bool eject_left = left_count < right_count;
  ateam_geometry::Vector shoot_vec{0.0, eject_left ? 1 : -1};

  kick_.SetKickSpeed(0.2);
  kick_.SetTargetPoint(world.ball.pos + shoot_vec);

  const auto command = kick_.RunFrame(world, goalie);
  ForwardPlayInfo(kick_);
  return command;
}

std::vector<ateam_geometry::AnyShape> Goalie::getCustomObstacles(const World & world)
{
  std::vector<ateam_geometry::AnyShape> obstacles;

  const auto half_field_length = world.field.field_length / 2.0;
  const auto half_goal_width = world.field.goal_width / 2.0;
  const auto goal_thickness = 0.1;  // arbitrarily large

  const auto goal_backwall_x = half_field_length + world.field.goal_depth;
  const auto goal_sidewall_outer_y = half_goal_width + goal_thickness;

  obstacles.push_back(
    ateam_geometry::Rectangle{
      ateam_geometry::Point{-half_field_length, -half_goal_width},
      ateam_geometry::Point{-goal_backwall_x, -goal_sidewall_outer_y}
    });

  obstacles.push_back(
    ateam_geometry::Rectangle{
      ateam_geometry::Point{-half_field_length, half_goal_width},
      ateam_geometry::Point{-goal_backwall_x, goal_sidewall_outer_y}
    });

  return obstacles;
}

std::optional<Robot> Goalie::getClosestEnemyRobotToBall(const World & world)
{
  if(!world.ball.visible) {
    return std::nullopt;
  }
  const auto enemy_bots = play_helpers::getVisibleRobots(world.their_robots);
  if(enemy_bots.empty()) {
    return std::nullopt;
  }
  const auto closest_bot = play_helpers::getClosestRobot(enemy_bots, world.ball.pos);
  if(CGAL::squared_distance(closest_bot.pos, world.ball.pos) > 1.0) {
    return std::nullopt;
  }
  return closest_bot;
}

void Goalie::glueBallToLastClosestEnemy(Ball & ball, const World & world)
{
  if(!last_enemy_id_closest_to_ball_) {
    return;
  }
  const auto bot_id = *last_enemy_id_closest_to_ball_;
  const auto & bot = world.their_robots[bot_id];
  if(!bot.visible) {
    return;
  }
  const auto direction = ateam_geometry::directionFromAngle(bot.theta);
  const auto vec = direction.to_vector() * (kRobotRadius + kBallRadius);
  ball.pos = bot.pos + vec;
  ball.vel = bot.vel;
}

}  // namespace ateam_kenobi::skills
