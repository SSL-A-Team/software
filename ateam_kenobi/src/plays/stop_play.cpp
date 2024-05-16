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

#include "stop_play.hpp"
#include <ranges>
#include <limits>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_common/robot_constants.hpp>
#include "play_helpers/window_evaluation.hpp"
#include "play_helpers/available_robots.hpp"
#include "play_helpers/robot_assignment.hpp"

namespace ateam_kenobi::plays
{
StopPlay::StopPlay()
: BasePlay("StopPlay")
{
  play_helpers::EasyMoveTo::CreateArray(easy_move_tos_, overlays_.getChild("EasyMoveTo"));
  for (auto & move_to : easy_move_tos_) {
    // Rules say <1.5m/s. We'll use 1m/s to give some room for error.
    move_to.setMaxVelocity(1.0);
  }
  StopPlay::reset();
}

double StopPlay::getScore(const World & world)
{
  switch (world.referee_info.running_command) {
    case ateam_common::GameCommand::Stop:
    case ateam_common::GameCommand::BallPlacementOurs:
    case ateam_common::GameCommand::BallPlacementTheirs:
      return std::numeric_limits<double>::max();
    default:
      return std::numeric_limits<double>::lowest();
  }
}

void StopPlay::reset()
{
  for (auto & move_to : easy_move_tos_) {
    move_to.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> StopPlay::runFrame(
  const World & world)
{
  const auto spots = getOpenSpots(world);

  auto bot_too_close = [ball_pos = world.ball.pos, r = kKeepoutRadius](const Robot bot) {
      return ateam_geometry::norm(bot.pos - ball_pos) < r;
    };

  std::vector<Robot> bots_to_move;
  std::ranges::copy_if(
    play_helpers::getAvailableRobots(world), std::back_inserter(
      bots_to_move), bot_too_close);

  std::ranges::transform(
    bots_to_move, std::back_inserter(getPlayInfo()["bots to move"]),
    [](const Robot & r) {return r.id;});

  const auto assignments = play_helpers::assignRobots(bots_to_move, spots);

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  for (size_t spot_ind = 0; spot_ind < spots.size(); ++spot_ind) {
    const auto & spot = spots[spot_ind];
    const auto & maybe_bot = assignments[spot_ind];
    if (!maybe_bot) {
      continue;
    }
    const auto & bot = *maybe_bot;
    auto & emt = easy_move_tos_.at(bot.id);
    emt.setTargetPosition(spot);
    emt.face_point(world.ball.pos);
    motion_commands.at(bot.id) = emt.runFrame(bot, world);
    getOverlays().drawCircle(
      "spot" + std::to_string(spot_ind),
      ateam_geometry::makeCircle(spot, kRobotRadius), "blue", "transparent");
  }

  // Halt all robots that weren't already assigned a motion command
  std::ranges::replace_if(
    motion_commands,
    [](const auto & o) {return !o;}, std::make_optional(ateam_msgs::msg::RobotMotionCommand{}));

  getOverlays().drawCircle(
    "keepout_circle",
    ateam_geometry::makeCircle(world.ball.pos, kKeepoutRadiusRules), "red", "transparent");

  return motion_commands;
}


std::vector<ateam_geometry::Point> StopPlay::getOpenSpots(const World & world)
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

  for (const auto & maybe_bot : world.their_robots) {
    if (!maybe_bot) {
      continue;
    }
    const auto & bot = *maybe_bot;
    if (ateam_geometry::norm(bot.pos - world.ball.pos) > consideration_radius) {
      continue;
    }
    const auto [ray_1, ray_2] = play_helpers::window_evaluation::getRobotShadowRays(
      bot,
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
    getOverlays().drawArc(
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
  return spots;
}

void StopPlay::removeArc(
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
}  // namespace ateam_kenobi::plays
