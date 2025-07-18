// Copyright 2025 A Team
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


#include "free_kick_on_goal_play.hpp"
#include <algorithm>
#include "core/play_helpers/window_evaluation.hpp"
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"
#include "core/play_helpers/lanes.hpp"

namespace ateam_kenobi::plays
{

FreeKickOnGoalPlay::FreeKickOnGoalPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  striker_(createChild<skills::UniversalKick>("striker")),
  idler_1_(createChild<skills::LaneIdler>("idler1")),
  idler_2_(createChild<skills::LaneIdler>("idler2")),
  defense_(createChild<tactics::StandardDefense>("defense"))
{
}

stp::PlayScore FreeKickOnGoalPlay::getScore(const World & world)
{
  if(world.referee_info.running_command != ateam_common::GameCommand::DirectFreeOurs) {
    return stp::PlayScore::NaN();
  }

  if((world.in_play || striker_.IsDone()) && world.ball.vel.x() < 0.01) {
    // Ball is stopped or moving up field
    return stp::PlayScore::NaN();
  }

  const auto largest_window = GetLargestWindowOnGoal(world);
  if (!largest_window) {
    return stp::PlayScore::Min();
  }
  const auto squared_goal_width = world.field.goal_width * world.field.goal_width;
  const auto fraction_of_goal_width = largest_window->squared_length() / squared_goal_width;
  return stp::PlayScore::Max() * fraction_of_goal_width;
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> FreeKickOnGoalPlay::runFrame(const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  const auto window = GetLargestWindowOnGoal(world);

  if(window) {
    striker_.SetTargetPoint(CGAL::midpoint(*window));
  } else {
    const auto goal_center = ateam_geometry::Point(world.field.field_length / 2.0, 0.0);
    striker_.SetTargetPoint(goal_center);
  }

  if(play_helpers::lanes::IsBallInLane(world, play_helpers::lanes::Lane::Left)) {
    idler_1_.SetLane(play_helpers::lanes::Lane::Center);
    idler_2_.SetLane(play_helpers::lanes::Lane::Right);
  } else if(play_helpers::lanes::IsBallInLane(world, play_helpers::lanes::Lane::Center)) {
    idler_1_.SetLane(play_helpers::lanes::Lane::Left);
    idler_2_.SetLane(play_helpers::lanes::Lane::Right);
  } else if(play_helpers::lanes::IsBallInLane(world, play_helpers::lanes::Lane::Right)) {
    idler_1_.SetLane(play_helpers::lanes::Lane::Left);
    idler_2_.SetLane(play_helpers::lanes::Lane::Center);
  }

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  play_helpers::GroupAssignmentSet groups;
  groups.AddPosition("striker", striker_.GetAssignmentPoint(world));
  groups.AddGroup("defense", defense_.getAssignmentPoints(world));
  if(available_robots.size() > 3) {
    groups.AddPosition("idler1", idler_1_.GetAssignmentPoint(world));
  }
  if(available_robots.size() > 4) {
    groups.AddPosition("idler2", idler_2_.GetAssignmentPoint(world));
  }

  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  SetDefenseAreaObstacles(world);

  assignments.RunPositionIfAssigned("striker", [this, &world, &motion_commands](const auto & robot){
      motion_commands[robot.id] = striker_.RunFrame(world, robot);
      getPlayInfo()["Striker"] = robot.id;
  });

  const auto defenders = assignments.GetGroupFilledAssignmentsOrEmpty("defense");
  defense_.runFrame(world, defenders, motion_commands);
  std::ranges::transform(defenders, std::back_inserter(getPlayInfo()["Defenders"]),
    [](const auto & r){return r.id;});

  if(available_robots.size() > 3) {
    assignments.RunPositionIfAssigned("idler1", [this, &world, &motion_commands](const auto & robot){
        motion_commands[robot.id] = idler_1_.RunFrame(world, robot);
        getPlayInfo()["Idlers"].push_back(robot.id);
    });
  }

  if(available_robots.size() > 4) {
    assignments.RunPositionIfAssigned("idler2", [this, &world, &motion_commands](const auto & robot){
        motion_commands[robot.id] = idler_2_.RunFrame(world, robot);
        getPlayInfo()["Idlers"].push_back(robot.id);
    });
  }

  return motion_commands;
}

std::optional<ateam_geometry::Segment> FreeKickOnGoalPlay::GetLargestWindowOnGoal(
  const World & world)
{
  const ateam_geometry::Segment their_goal_segment{
    ateam_geometry::Point{world.field.field_length / 2.0, -world.field.goal_width / 2.0},
    ateam_geometry::Point{world.field.field_length / 2.0, world.field.goal_width / 2.0}
  };
  const auto windows = play_helpers::window_evaluation::getWindows(
    their_goal_segment,
    world.ball.pos, play_helpers::getVisibleRobots(
      world.their_robots));
  return play_helpers::window_evaluation::getLargestWindow(windows);
}


void FreeKickOnGoalPlay::SetDefenseAreaObstacles(const World & world)
{
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
  const std::vector<ateam_geometry::AnyShape> obstacles = {defense_area_obstacle};
  idler_1_.SetExtraObstacles(obstacles);
  idler_2_.SetExtraObstacles(obstacles);
  getOverlays().drawRectangle("defense_area_obstacle", defense_area_obstacle, "red", "transparent");
}

}  // namespace ateam_kenobi::plays
