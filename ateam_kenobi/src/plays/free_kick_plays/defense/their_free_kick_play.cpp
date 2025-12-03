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

#include "their_free_kick_play.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <vector>
#include "core/play_helpers/robot_assignment.hpp"
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

TheirFreeKickPlay::TheirFreeKickPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  defense_(createChild<tactics::StandardDefense>("defense"))
{
}

stp::PlayScore TheirFreeKickPlay::getScore(const World & world)
{
  if(world.referee_info.running_command == ateam_common::GameCommand::DirectFreeTheirs) {
    return world.in_play ? stp::PlayScore::Min() : stp::PlayScore::Max();
  } else {
    return stp::PlayScore::NaN();
  }
}

void TheirFreeKickPlay::reset()
{
  defense_.reset();
}

std::array<std::optional<RobotCommand>,
  16> TheirFreeKickPlay::runFrame(const World & world)
{
  std::array<std::optional<RobotCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  const auto blocker_points = getBlockerPoints(world);

  play_helpers::GroupAssignmentSet groups;
  groups.AddGroup("defense", defense_.getAssignmentPoints(world));
  groups.AddGroup("blockers", blocker_points);

  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  defense_.runFrame(world, assignments.GetGroupFilledAssignments("defense"), motion_commands);

  runBlockers(
    world, assignments.GetGroupFilledAssignments(
      "blockers"), blocker_points, motion_commands);

  return motion_commands;
}


std::vector<ateam_geometry::Point> TheirFreeKickPlay::getBlockerPoints(const World & world)
{
  const auto arc_radius = 1.0;
  getOverlays().drawCircle(
    "target", ateam_geometry::makeCircle(
      world.ball.pos,
      arc_radius), "blue", "transparent");
  const auto inter_robot_angle = angles::from_degrees(15);
  const auto our_goal_center = ateam_geometry::Point{-world.field.field_length / 2.0, 0.0};
  const auto ball_goal_vec = our_goal_center - world.ball.pos;
  const auto offset_vec = ateam_geometry::normalize(ball_goal_vec) * arc_radius;
  CGAL::Aff_transformation_2<ateam_geometry::Kernel> rotate_transform(CGAL::ROTATION, std::sin(
      inter_robot_angle), std::cos(inter_robot_angle));
  std::vector<ateam_geometry::Point> points;
  points.push_back(world.ball.pos + offset_vec);
  points.push_back(world.ball.pos + offset_vec.transform(rotate_transform));
  points.push_back(world.ball.pos + offset_vec.transform(rotate_transform.inverse()));
  return points;
}

void TheirFreeKickPlay::runBlockers(
  const World & world, const std::vector<Robot> & robots,
  const std::vector<ateam_geometry::Point> & points,
  std::array<std::optional<RobotCommand>, 16> & motion_commands)
{
  // Rules section 5.3.3 require 0.5m distance between defenders and ball
  const auto ball_obstacle = ateam_geometry::makeDisk(world.ball.pos, 0.7);
  getOverlays().drawCircle("ball_obstacle", ball_obstacle, "red", "transparent");

  // Rules section 8.4.1 require 0.2m distance between all robots and opponent defense area
  const auto def_area_margin = kRobotRadius + 0.2;
  const auto def_area_obst_width = world.field.defense_area_width + (2.0 * def_area_margin);
  const auto def_area_obst_depth = world.field.defense_area_depth + def_area_margin;
  const auto half_field_length = world.field.field_length / 2.0;
  const auto def_area_back_x = half_field_length + ( 2 * world.field.boundary_width ) +
    def_area_obst_depth;
  const auto def_area_front_x = half_field_length - def_area_obst_depth;
  const auto defense_area_obstacle = ateam_geometry::Rectangle{
    ateam_geometry::Point{def_area_front_x, -def_area_obst_width / 2.0},
    ateam_geometry::Point{def_area_back_x, def_area_obst_width / 2.0}
  };
  getOverlays().drawRectangle("defense_area_obstacle", defense_area_obstacle, "red", "transparent");

  std::vector<ateam_geometry::AnyShape> obstacles = {ball_obstacle, defense_area_obstacle};
  for (auto i = 0ul; i < std::min(robots.size(), points.size()); ++i) {
    const auto & robot = robots[i];
    const auto & point = points[i];
    RobotCommand command;
    command.motion_intent.linear = motion::intents::linear::PositionIntent{point};
    command.motion_intent.angular = motion::intents::angular::FacingIntent{world.ball.pos};
    command.motion_intent.obstacles = obstacles;
    motion_commands[robot.id] = command;
  }
}

}  // namespace ateam_kenobi::plays
