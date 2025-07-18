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


#include "path_planner.hpp"
#include <ranges>
#include <algorithm>
#include <limits>
#include <vector>
#include <iostream>
#include <ateam_geometry/ateam_geometry.hpp>
#include "obstacles.hpp"

namespace ateam_kenobi::path_planning
{

PathPlanner::PathPlanner(stp::Options stp_options)
: stp::Base(stp_options)
{
}

PathPlanner::Path PathPlanner::getPath(
  const Position & start, const Position & goal, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles,
  const PlannerOptions & options)
{
  start_time_ = std::chrono::steady_clock::now();

  std::vector<ateam_geometry::AnyShape> augmented_obstacles = obstacles;

  AddRobotObstacles(world.our_robots, start, augmented_obstacles);
  AddRobotObstacles(world.their_robots, augmented_obstacles);

  if (options.use_default_obstacles) {
    AddDefaultObstacles(world, augmented_obstacles);
  }

  if (options.avoid_ball) {
    augmented_obstacles.push_back(ateam_geometry::makeDisk(world.ball.pos, kBallRadius));
  }

  if (options.draw_obstacles) {
    drawObstacles(augmented_obstacles);
  }

  const auto goal_is_valid = isStateValid(goal, world, augmented_obstacles, options);

  if(!shouldReplan(start, goal, world, augmented_obstacles, options)) {
    if(cached_path_truncated_ && goal_is_valid) {
      used_cached_path_ = false;
    } else {
      if (!cached_path_truncated_) {
        cached_path_.back() = goal;
      }
      used_cached_path_ = true;
      return cached_path_;
    }
  }

  used_cached_path_ = false;

  if (!IsPointInBounds(start, world, false)) {
    cached_path_valid_ = false;
    return {};
  }

  if (options.ignore_start_obstacle) {
    removeCollidingObstacles(augmented_obstacles, start, options);
  } else if (!isStateValid(start, world, augmented_obstacles, options,
      BoundaryStrategy::OffsetIn))
  {
    cached_path_valid_ = false;
    return {};
  }

  Path path = {start, goal};

  if (!goal_is_valid) {
    const auto maybe_new_goal = findLastCollisionFreePoint(
      start, goal, world, augmented_obstacles,
      options);
    if (!maybe_new_goal) {
      cached_path_valid_ = false;
      return {};
    }
    path.back() = *maybe_new_goal;
    cached_path_truncated_ = true;
  } else {
    cached_path_truncated_ = false;
  }

  timed_out_ = false;

  while (true) {
    if (isTimeUp(options)) {
      RCLCPP_WARN(getLogger(), "Path planning timed out.");
      trimPathAfterCollision(path, world, augmented_obstacles, options);
      timed_out_ = true;
      break;
    }
    bool had_to_split = false;
    for (auto ind = 0u; ind < (path.size() - 1); ++ind) {
      auto split_result = splitSegmentIfNecessary(
        path, ind, ind + 1, world, augmented_obstacles,
        options);
      if (split_result.split_needed) {
        had_to_split = true;
        if (!split_result.split_succeeded) {
          // unable to find path around obstacle
          cached_path_valid_ = false;
          return {};
        }
        break;
      }
    }
    if (!had_to_split) {
      // no split -> path is collision free
      break;
    }
  }

  removeLoops(path);
  removeSkippablePoints(path, world, augmented_obstacles, options);
  smoothCorners(path, world, augmented_obstacles, options);

  if (!timed_out_) {
    cached_path_ = path;
    cached_path_goal_ = goal;
    cached_path_valid_ = true;
  }

  return path;
}

bool PathPlanner::isTimeUp(const PlannerOptions & options)
{
  return (std::chrono::steady_clock::now() - start_time_) >=
         std::chrono::duration<double>(options.search_time_limit);
}

void PathPlanner::removeCollidingObstacles(
  std::vector<ateam_geometry::AnyShape> & obstacles,
  const ateam_geometry::Point & point, const PlannerOptions & options)
{
  auto robot_footprint = ateam_geometry::makeDisk(
    point,
    kRobotRadius + options.footprint_inflation
  );

  auto is_obstacle_colliding = [&robot_footprint](const ateam_geometry::AnyShape & obstacle) {
      return ateam_geometry::doIntersect(robot_footprint, obstacle);
    };

  const auto new_end = std::remove_if(obstacles.begin(), obstacles.end(), is_obstacle_colliding);

  obstacles.erase(new_end, obstacles.end());
}

bool PathPlanner::isStateValid(
  const ateam_geometry::Point & state,
  const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles,
  const PlannerOptions & options, const BoundaryStrategy bounds_strat)
{
  if(bounds_strat != BoundaryStrategy::Ignore) {
    double offset = 0.0;
    switch(bounds_strat) {
      case BoundaryStrategy::OffsetIn:
        offset = kRobotRadius * 0.95;
        break;
      case BoundaryStrategy::OffsetOut:
        break;
      default:
        break;
    }
    if (!IsPointInBounds(state, world, offset)) {
      return false;
    }
  }

  auto robot_footprint = ateam_geometry::makeDisk(
    state,
    kRobotRadius + options.footprint_inflation
  );

  return std::ranges::none_of(
    obstacles, [&robot_footprint](const ateam_geometry::AnyShape & obstacle) {
      return ateam_geometry::doIntersect(robot_footprint, obstacle);
    });
}

std::optional<ateam_geometry::Point> PathPlanner::getCollisionPoint(
  const ateam_geometry::Point & p1, const ateam_geometry::Point & p2, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles, const PlannerOptions & options)
{
  const auto direction_vector = p2 - p1;
  const auto segment_length = std::sqrt(direction_vector.squared_length());
  const auto step_vector = direction_vector * (options.collision_check_resolution / segment_length);
  const int step_count = segment_length / options.collision_check_resolution;
  for (auto step = 0; step < step_count; ++step) {
    const auto state = p1 + (step * step_vector);
    if (!isStateValid(state, world, obstacles, options)) {
      return state;
    }
  }
  if (!isStateValid(p2, world, obstacles, options)) {
    return p2;
  }
  return std::nullopt;
}

PathPlanner::SplitResult PathPlanner::splitSegmentIfNecessary(
  Path & path, const std::size_t ind1,
  const std::size_t ind2, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles,
  const PlannerOptions & options)
{
  const auto maybe_collision_point = getCollisionPoint(
    path.at(ind1), path.at(
      ind2), world, obstacles, options);
  if (!maybe_collision_point) {
    return {
      .split_needed = false,
      .split_succeeded = true
    };
  }
  const auto collision_point = maybe_collision_point.value();
  const auto p1 = path[ind1];
  const auto p2 = path[ind2];
  auto change_vector = (ateam_geometry::normalize(p2 - p1) * kRobotRadius).perpendicular(
    CGAL::CLOCKWISE);
  auto new_point = collision_point;
  auto direction = 1;
  for (auto i = 1; i < 100; ++i) {
    new_point = new_point + (change_vector * i * direction);
    direction *= -1;
    if (isStateValid(new_point, world, obstacles, options)) {
      path.insert(path.begin() + ind2, new_point);
      return {
        .split_needed = true,
        .split_succeeded = true
      };
    }
  }
  return {
    .split_needed = true,
    .split_succeeded = false
  };
}

void PathPlanner::removeSkippablePoints(
  Path & path, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles,
  const PlannerOptions & options)
{
  if (path.size() < 3) {
    return;
  }

  auto were_points_removed = true;
  while (were_points_removed) {
    were_points_removed = false;
    auto candidate_index = 1ul;
    while (candidate_index < path.size() - 1) {
      auto maybe_collision_point =
        getCollisionPoint(
        path[candidate_index - 1], path[candidate_index + 1], world, obstacles,
        options);
      // Ignore collisions at path end since that means planning timed out
      if (maybe_collision_point &&
        CGAL::squared_distance(*maybe_collision_point, path.back()) > 1e-6)
      {
        candidate_index++;
        continue;
      }
      path.erase(path.begin() + candidate_index);
      were_points_removed = true;
    }
  }
}

void PathPlanner::removeLoops(Path & path)
{
  using ateam_geometry::Point;
  using ateam_geometry::Segment;
  if (path.size() < 4) {
    return;
  }
  // Check all but last two segments
  for (auto ind1 = 0ul; ind1 < path.size() - 3; ++ind1) {
    // Check all segments after ind1->ind1+1, skipping the first
    for (auto ind2 = ind1 + 2; ind2 < path.size() - 1; ) {
      const auto seg1 = Segment(path[ind1], path[ind1 + 1]);
      const auto seg2 = Segment(path[ind2], path[ind2 + 1]);
      const auto maybe_intersection = CGAL::intersection(seg1, seg2);
      if (!maybe_intersection) {
        ++ind2;
        continue;
      }
      const auto & intersection_var = maybe_intersection.value();
      Point new_point;
      if (const Point * intersection_point = boost::get<Point>(&intersection_var)) {
        new_point = *intersection_point;
      } else if (const Segment * intersection_seg = boost::get<Segment>(&intersection_var)) {
        new_point = intersection_seg->target();
      }
      // put intersection point in path (replacing first segment's end point)
      path[ind1 + 1] = new_point;
      // remove all points along the loop (keep second segment's end point)
      path.erase(path.begin() + ind1 + 2, path.begin() + ind2 + 1);
      ind2 = ind1 + 2;
      if (path.size() < 4) {
        // path now too small to have loops
        return;
      }
    }
  }
}

void PathPlanner::trimPathAfterCollision(
  Path & path, const World & world,
  std::vector<ateam_geometry::AnyShape> & obstacles,
  const PlannerOptions & options)
{
  for (auto ind = 0u; ind < (path.size() - 1); ++ind) {
    const auto maybe_collision = getCollisionPoint(
      path[ind], path[ind + 1], world, obstacles,
      options);
    if (maybe_collision) {
      path.erase(path.begin() + ind + 1, path.end());
      path.push_back(maybe_collision.value());
      break;
    }
  }
}

std::optional<ateam_geometry::Point> PathPlanner::findLastCollisionFreePoint(
  const ateam_geometry::Point & start, const ateam_geometry::Point & goal, const World & world,
  std::vector<ateam_geometry::AnyShape> & obstacles,
  const PlannerOptions & options)
{
  const auto direction_vector = start - goal;
  const auto segment_length = std::sqrt(direction_vector.squared_length());
  const auto step_vector = direction_vector * (options.collision_check_resolution / segment_length);
  const int step_count = segment_length / options.collision_check_resolution;
  for (auto step = 0; step < step_count; ++step) {
    const auto state = goal + (step * step_vector);
    if (isStateValid(state, world, obstacles, options)) {
      return state;
    }
  }
  if (isStateValid(start, world, obstacles, options)) {
    return start;
  }
  return std::nullopt;
}

void PathPlanner::drawObstacles(const std::vector<ateam_geometry::AnyShape> & obstacles)
{
  auto drawObstacle = [this, obstacle_ind = 0](const auto & shape) mutable {
      const auto name = "obstacle" + std::to_string(obstacle_ind);
      const auto color = "FF00007F";
      using ShapeT = std::decay_t<decltype(shape)>;
      if constexpr (std::is_same_v<ShapeT, ateam_geometry::Point>) {
        getOverlays().drawCircle(name, ateam_geometry::makeCircle(shape, 2.5), color, color);
      } else if constexpr (std::is_same_v<ShapeT, ateam_geometry::Segment>) {
        getOverlays().drawLine(name, {shape.source(), shape.target()}, color);
      } else if constexpr (std::is_same_v<ShapeT, ateam_geometry::Ray>) {
        getOverlays().drawLine(name, {shape.source(), shape.point(10)}, color);
      } else if constexpr (std::is_same_v<ShapeT, ateam_geometry::Rectangle>) {
        getOverlays().drawRectangle(name, shape, color, color);
      } else if constexpr (std::is_same_v<ShapeT, ateam_geometry::Circle>) {
        getOverlays().drawCircle(name, shape, color, color);
      } else if constexpr (std::is_same_v<ShapeT, ateam_geometry::Disk>) {
        getOverlays().drawCircle(name, shape.asCircle(), color, color);
      } else {
        RCLCPP_WARN(
          getLogger(), "Unsupported shape type in drawObstacles: %s ",
          typeid(shape).name());
      }
      obstacle_ind++;
    };

  std::ranges::for_each(obstacles, [&drawObstacle](const auto & s) {std::visit(drawObstacle, s);});
}

bool PathPlanner::shouldReplan(
  const Position & start, const Position & goal, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles,
  const PlannerOptions & options)
{
  if (options.force_replan) {
    return true;
  }
  if (!cached_path_valid_) {
    return true;
  }
  const auto goal_has_moved = CGAL::squared_distance(goal,
      cached_path_goal_) >= CGAL::square(options.replan_thresholds.goal_distance_);
  if (goal_has_moved) {
    return true;
  }

  double min_squared_start_distance = std::numeric_limits<double>::max();
  for(auto i = 0ul; i < cached_path_.size() - 1; ++i) {
    const auto a = cached_path_[i];
    const auto b = cached_path_[i + 1];
    if (getCollisionPoint(a, b, world, obstacles, options).has_value()) {
      return true;
    }
    const ateam_geometry::Segment s(a, b);
    const auto squared_start_distance = CGAL::squared_distance(s, start);
    min_squared_start_distance = std::min(squared_start_distance, min_squared_start_distance);
  }

  if(min_squared_start_distance >= CGAL::square(options.replan_thresholds.start_distance_)) {
    return true;
  }

  return false;
}


void PathPlanner::smoothCorners(
  Path & path, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles, const PlannerOptions & options)
{
  if(path.size() < 3) {
    return;
  }
  for(auto i = 1ul; i < path.size() - 1;) {
    if(isTimeUp(options)) {
      return;
    }
    const auto before_point = path[i - 1];
    const auto curr_point = path[i];
    const auto next_point = path[i + 1];
    const auto corner_angle = ateam_geometry::AngleBetweenPoints(before_point, curr_point,
        next_point);
    if(corner_angle >= options.corner_smoothing_angle_threshold) {
      ++i;
    } else {
      smoothCorner(path, i, world, obstacles, options);
      // Do not increment index as newly created corner might still be sharp
    }
  }
}

void PathPlanner::smoothCorner(
  Path & path, const size_t corner_point_index, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles, const PlannerOptions & options)
{
  const auto step_size = options.corner_smoothing_step_size;
  const auto before_point = path[corner_point_index - 1];
  const auto curr_point = path[corner_point_index];
  const auto after_point = path[corner_point_index + 1];
  const auto max_cut_side_length = std::min(ateam_geometry::norm(curr_point - before_point),
      ateam_geometry::norm(after_point, curr_point));
  const auto num_steps = static_cast<int>(max_cut_side_length / step_size);
  const auto before_vec = ateam_geometry::normalize(before_point - curr_point);
  const auto after_vec = ateam_geometry::normalize(after_point - curr_point);
  ateam_geometry::Point cut_point_before;
  ateam_geometry::Point cut_point_after;
  for(auto cut_step = 0; cut_step < num_steps; ++cut_step) {
    if(isTimeUp(options)) {
      break;
    }
    const auto cut_side_length = cut_step * step_size;
    const auto cut_point_before_candidate = curr_point + (cut_side_length * before_vec);
    const auto cut_point_after_candidate = curr_point + (cut_side_length * after_vec);
    if(!getCollisionPoint(cut_point_before_candidate, cut_point_after_candidate, world, obstacles, options)) {
      cut_point_before = cut_point_before_candidate;
      cut_point_after = cut_point_after_candidate;
    } else {
      break;
    }
  }
  path[corner_point_index] = cut_point_before;
  path.insert(path.begin() + corner_point_index + 1, cut_point_after);
}

}  // namespace ateam_kenobi::path_planning
