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

#include "stop_helpers.hpp"
#include <algorithm>
#include <ateam_common/robot_constants.hpp>
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/window_evaluation.hpp"
#include "core/path_planning/obstacles.hpp"
#include "core/path_planning/escape_velocity.hpp"
#include "core/play_helpers/robot_assignment.hpp"

namespace ateam_kenobi::plays::stop_plays::stop_helpers
{

std::vector<ateam_geometry::Point> getOpenSpots(
  const World & world,
  visualization::Overlays & overlays)
{
  const double consideration_radius = kKeepoutRadius + kRobotDiameter;

  auto bot_within_consideration_radius =
    [&consideration_radius, ball_pos = world.ball.pos](const Robot & bot) {
      return CGAL::squared_distance(
        bot.pos,
        ball_pos) <= consideration_radius * consideration_radius;
    };

  std::vector<Robot> close_opponent_bots;
  std::ranges::copy_if(
    play_helpers::getVisibleRobots(world.their_robots),
    std::back_inserter(close_opponent_bots), bot_within_consideration_radius);

  std::vector<ateam_geometry::Arc> openings = {
    ateam_geometry::Arc(
      world.ball.pos, kKeepoutRadius, ateam_geometry::directionFromAngle(
        0.0), ateam_geometry::directionFromAngle(1.99 * M_PI))
  };

  for (const auto & robot : world.their_robots) {
    if (!robot.IsAvailable()) {
      continue;
    }
    if (ateam_geometry::norm(robot.pos - world.ball.pos) > consideration_radius) {
      continue;
    }
    const auto [ray_1, ray_2] = play_helpers::window_evaluation::getRobotShadowRays(
      robot,
      world.ball.pos);
    removeArc(
      openings,
      ateam_geometry::Arc(world.ball.pos, kKeepoutRadius, ray_1.direction(), ray_2.direction()));
  }

  // 30% breathing room around robots
  const auto robot_sized_angle = 4 * std::asin((1.30 * kRobotRadius) / (2 * kKeepoutRadius));

  const CGAL::Aff_transformation_2<ateam_geometry::Kernel> rotate_one_robot_size(CGAL::ROTATION,
    std::sin(robot_sized_angle), std::cos(robot_sized_angle));

  std::vector<ateam_geometry::Point> spots;

  auto ind = 0;
  for (const auto & opening : openings) {
    overlays.drawArc(
      "opening" + std::to_string(
        ind), opening, ind % 2 == 0 ? "white" : "lightgray");
    ind++;

    const auto num_spots_in_opening = std::floor(opening.angle() / robot_sized_angle);
    if (num_spots_in_opening == 0) {
      // opening too small
      continue;
    }
    auto start = opening.start();
    for (auto i = 0; i < num_spots_in_opening; ++i) {
      const auto end = rotate_one_robot_size(start);
      const ateam_geometry::Arc spot_arc(world.ball.pos, kKeepoutRadius, start, end);
      spots.push_back(spot_arc.midpoint());
      start = end;
    }
  }

  spots.erase(
    std::remove_if(
      spots.begin(), spots.end(), [&world](const auto & p) {
        return (!path_planning::IsPointInBounds(p, world)) || isPointInOrBehindGoal(p, world);
      }), spots.end());

  return spots;
}

void removeArc(
  std::vector<ateam_geometry::Arc> & openings,
  const ateam_geometry::Arc & arc)
{
  for (size_t i = 0; i < openings.size(); ++i) {
    auto & opening = openings[i];
    const auto start_intersects = arc.start().counterclockwise_in_between(
      opening.start(), opening.end());
    const auto end_intersects =
      arc.end().counterclockwise_in_between(opening.start(), opening.end());
    const auto opening_entirely_covered = opening.start().counterclockwise_in_between(
      arc.start(), arc.end()) && opening.end().counterclockwise_in_between(arc.start(), arc.end());

    if (start_intersects) {
      if (end_intersects) {
        // split opening
        auto new_opening = opening;
        opening.end() = arc.start();
        new_opening.start() = arc.end();
        openings.insert(openings.begin() + i + 1, new_opening);
        i++;
      } else {
        // trim opening end
        opening.end() = arc.start();
      }
    } else if (end_intersects) {
      // trim opening start
      opening.start() = arc.end();
    } else if (opening_entirely_covered) {
      // opening is entirely within arc, remove it
      openings.erase(openings.begin() + i);
      i--;
    }
  }
}

bool isPointInOrBehindGoal(
  const ateam_geometry::Point & point,
  const World & world)
{
  const auto half_goal_width = world.field.goal_width / 2.0;
  const auto half_field_length = world.field.field_length / 2.0;
  const ateam_geometry::Rectangle bad_area{
    ateam_geometry::Point{-half_field_length, -half_goal_width},
    ateam_geometry::Point{-(half_field_length + world.field.boundary_width), half_goal_width}
  };
  return CGAL::do_intersect(bad_area, point);
}

std::vector<ateam_geometry::AnyShape> getAddedObstacles(const World & world)
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

  // Rules section 8.4.1 require 0.2m distance between all robtos and opponent defense area
  const auto def_area_margin = kRobotRadius + 0.2;
  const auto def_area_obst_width = world.field.defense_area_width + (2.0 * def_area_margin);
  const auto def_area_obst_depth = world.field.defense_area_depth + def_area_margin;
  const auto def_area_back_x = half_field_length + ( 2 * world.field.boundary_width ) +
    def_area_obst_depth;
  const auto def_area_front_x = half_field_length - def_area_obst_depth;
  const auto defense_area_obstacle = ateam_geometry::Rectangle{
    ateam_geometry::Point{def_area_front_x, -def_area_obst_width / 2.0},
    ateam_geometry::Point{def_area_back_x, def_area_obst_width / 2.0}
  };
  obstacles.push_back(defense_area_obstacle);

  return obstacles;
}

void drawObstacles(
  const World & world,
  const std::vector<ateam_geometry::AnyShape> & added_obstacles, visualization::Overlays & overlays,
  rclcpp::Logger & logger)
{
  int obstacle_index = 0;
  for(const auto & obst : added_obstacles) {
    std::visit([&obstacle_index, &overlays, &logger] (const auto & o) mutable {
        if constexpr (std::is_same_v<decltype(o), const ateam_geometry::Circle &>) {
          overlays.drawCircle("obstacle" + std::to_string(obstacle_index), o, "red",
            "transparent");
        } else if constexpr (std::is_same_v<decltype(o), const ateam_geometry::Rectangle &>) {
          overlays.drawRectangle("obstacle" + std::to_string(obstacle_index), o, "red",
            "transparent");
        } else {
          RCLCPP_WARN_STREAM(logger, "Unsupported obstacle type: " << typeid(o).name());
        }
    }, obst);
    obstacle_index++;
  }

  overlays.drawCircle(
    "keepout_circle",
    ateam_geometry::makeCircle(world.ball.pos, kKeepoutRadiusRules), "red", "transparent");
}

void moveBotsTooCloseToBall(
  const World & world,
  const std::vector<ateam_geometry::AnyShape> & added_obstacles,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & motion_commands,
  std::array<play_helpers::EasyMoveTo, 16> & easy_move_tos, visualization::Overlays & overlays,
  nlohmann::json & play_info)
{
  const auto spots = getOpenSpots(world, overlays);

  auto bot_too_close = [ball_pos = world.ball.pos, r = kKeepoutRadius](const Robot bot) {
      return ateam_geometry::norm(bot.pos - ball_pos) < r;
    };

  std::vector<Robot> bots_to_move;
  std::ranges::copy_if(
    play_helpers::getAvailableRobots(world), std::back_inserter(
      bots_to_move), bot_too_close);

  auto & play_info_array = play_info["bots near ball"];
  play_info_array = nlohmann::json::array();
  std::ranges::transform(bots_to_move, std::back_inserter(play_info_array),
    [](const Robot & r) {return r.id;});

  const auto assignments = play_helpers::assignRobots(bots_to_move, spots);

  for (size_t spot_ind = 0; spot_ind < spots.size(); ++spot_ind) {
    const auto & spot = spots[spot_ind];
    const auto & maybe_bot = assignments[spot_ind];
    if (!maybe_bot) {
      continue;
    }
    const auto & bot = *maybe_bot;
    if(motion_commands[bot.id]) {
      continue;
    }
    auto & emt = easy_move_tos.at(bot.id);
    emt.setTargetPosition(spot);
    emt.face_point(world.ball.pos);
    if (bot.id == world.referee_info.our_goalie_id) {
      auto planner_options = emt.getPlannerOptions();
      planner_options.use_default_obstacles = false;
      emt.setPlannerOptions(planner_options);
    } else {
      auto planner_options = emt.getPlannerOptions();
      planner_options.use_default_obstacles = true;
      emt.setPlannerOptions(planner_options);
    }
    motion_commands.at(bot.id) = emt.runFrame(bot, world, added_obstacles);
    overlays.drawCircle(
      "spot" + std::to_string(spot_ind),
      ateam_geometry::makeCircle(spot, kRobotRadius), "blue", "transparent");
  }
}

void moveBotsInObstacles(
  const World & world,
  const std::vector<ateam_geometry::AnyShape> & added_obstacles,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & motion_commands,
  nlohmann::json & play_info)
{
  auto & play_info_bots = play_info["bots in obstacles"];
  play_info_bots = nlohmann::json::array();
  for(auto i = 0ul; i < motion_commands.size(); ++i) {
    const auto & robot = world.our_robots[i];
    if(!robot.IsAvailable()) {
      continue;
    }
    if(motion_commands[i]) {
      continue;
    }
    const auto opt_escape_vel = path_planning::GenerateEscapeVelocity(robot, added_obstacles);
    if(!opt_escape_vel) {
      continue;
    }
    ateam_msgs::msg::RobotMotionCommand command;
    command.twist = *opt_escape_vel;
    motion_commands[i] = command;
    play_info_bots.push_back(robot.id);
  }
}

}  // namespace ateam_kenobi::plays::stop_plays::stop_helpers
