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

#include "motion_executor.hpp"
#include <ateam_geometry/nearest_point.hpp>
#include "core/path_planning/obstacles.hpp"
#include "core/path_planning/escape_velocity.hpp"
#include "world_to_body_vel.hpp"

namespace ateam_kenobi::motion
{

// helper type for the visitor
template<class ... Ts>
struct overloads : Ts ... { using Ts::operator() ...; };

MotionExecutor::MotionExecutor(rclcpp::Logger logger)
: logger_(std::move(logger))
{

}

std::array<std::optional<BodyVelocity>,
  16> MotionExecutor::RunFrame(
  std::array<std::optional<MotionIntent>, 16> intents,
  visualization::Overlays & overlays, const World & world)
{
  const auto current_time = std::chrono::duration_cast<std::chrono::duration<double>>(
    world.current_time.time_since_epoch()).count();

  std::array<std::optional<BodyVelocity>, 16> results;

  for (size_t i = 0; i < intents.size(); ++i) {
    if (!intents[i]) {
      results[i] = std::nullopt;
      continue;
    }

    const auto & robot = world.our_robots[i];
    auto & intent = intents[i].value();
    auto & planner = planners_[i];
    auto & controller = controllers_[i];

    BodyVelocity body_velocity;
    path_planning::Path path;

    bool use_controller_linvel = true;
    std::visit(overloads{
        [&](const intents::None &) {
          controller.reset_trajectory({robot.pos});
        },
        [&](const intents::linear::VelocityIntent & v) {
          body_velocity.linear = v.velocity;
          use_controller_linvel = false;
          controller.reset_trajectory({robot.pos});
        },
        [&](const intents::linear::PositionIntent & p) {
          path = planner.getPath(robot.pos, p.position, world, intent.obstacles,
          intent.planner_options);
          if(path.empty()) {
            return;
          }
          if(planner.usedCachedPath()) {
            controller.update_trajectory(path);
          } else {
            controller.reset_trajectory(path);
          }
        },
        [&](const intents::linear::VelocityAtPositionIntent & v) {
          path = planner.getPath(robot.pos, v.position, world, intent.obstacles,
          intent.planner_options);
          if(path.empty()) {
            return;
          }
          if(planner.usedCachedPath()) {
            controller.update_trajectory(path, v.velocity);
          } else {
            controller.reset_trajectory(path, v.velocity);
          }
        }
    }, intent.linear);

    bool use_controller_omega = true;
    std::visit(overloads{
        [](const intents::None &) {},
        [&](const intents::angular::VelocityIntent & v) {
          body_velocity.angular = v.omega;
          use_controller_omega = false;
        },
        [&](const intents::angular::HeadingIntent & h) {
          controller.face_absolute(h.theta);
        },
        [&](const intents::angular::FacingIntent & f) {
          controller.face_point(f.target);
        },
        [&](const intents::angular::FaceTravelIntent &) {
          controller.face_travel();
        }
    }, intent.angular);

    if(!path.empty()) {
      auto controller_vel = controller.get_command(robot, current_time, intent.motion_options);
      if (use_controller_linvel) {
        body_velocity.linear = controller_vel.linear;
      }
      if (use_controller_omega) {
        body_velocity.angular = controller_vel.angular;
      }
    }

    if (intent.enable_escape_velocities &&
      (std::holds_alternative<intents::linear::PositionIntent>(intent.linear) ||
      std::holds_alternative<intents::linear::VelocityAtPositionIntent>(intent.linear)))
    {
      auto escape_velocity = GenerateEscapeVelocity(world, robot, intent);
      if (escape_velocity) {
        body_velocity = escape_velocity.value();
      }
    }

    if(intent.callback.has_value()) {
      body_velocity = intent.callback.value()(body_velocity, path, robot, world);
    }

    DrawOverlays(overlays, world, robot, path, intent);

    results[i] = body_velocity;
  }

  return results;
}

std::optional<BodyVelocity> MotionExecutor::GenerateEscapeVelocity(
  const World & world,
  const Robot & robot,
  const MotionIntent & intent)
{
  std::vector<ateam_geometry::AnyShape> obstacles = intent.obstacles;
  if (intent.planner_options.use_default_obstacles) {
    path_planning::AddDefaultObstacles(world, obstacles);
  }
  path_planning::AddRobotObstacles(world.our_robots, robot.id, obstacles);
  path_planning::AddRobotObstacles(world.their_robots, obstacles);

  auto vel = path_planning::GenerateEscapeVelocity(robot, obstacles,
      intent.planner_options.footprint_inflation);
  if (vel) {
    return BodyVelocity{
      ateam_geometry::Vector(vel->linear.x, vel->linear.y),
      vel->angular.z
    };
  } else {
    return std::nullopt;
  }
}

void MotionExecutor::DrawOverlays(
  visualization::Overlays & overlays, const World & world,
  const Robot & robot, const path_planning::Path & path,
  const MotionIntent & intent)
{
  (void)world;
  const auto name_prefix = "motion/robot_" + std::to_string(robot.id) + "/";
  if(std::holds_alternative<intents::linear::PositionIntent>(intent.linear) ||
    std::holds_alternative<intents::linear::VelocityAtPositionIntent>(intent.linear))
  {
    const auto target_point = std::visit(overloads{
        [](const intents::linear::PositionIntent & p) {return p.position;},
        [](const intents::linear::VelocityAtPositionIntent & v) {return v.position;},
        [](const auto &){return ateam_geometry::Point();}
    }, intent.linear);
    if(path.empty()) {
      overlays.drawLine(name_prefix + "path", {robot.pos, target_point}, "Red");
    } else if(path.size() == 1) {
      overlays.drawLine(name_prefix + "path", {robot.pos, target_point}, "Purple");
    } else {
      const auto [closest_index, closest_point] = ProjectRobotOnPath(path, robot);
      std::vector<ateam_geometry::Point> path_done(path.begin(), path.begin() + (closest_index));
      path_done.push_back(closest_point);
      std::vector<ateam_geometry::Point> path_remaining(path.begin() + (closest_index),
        path.end());
      path_remaining.insert(path_remaining.begin(), closest_point);
      const auto translucent_purple = "#8000805F";
      overlays.drawLine(name_prefix + "path_done", path_done, translucent_purple);
      overlays.drawLine(name_prefix + "path_remaining", path_remaining, "Purple");
      const auto & planner = planners_[robot.id];
      if (planner.didTimeOut()) {
        overlays.drawLine(name_prefix + "afterpath", {path.back(), target_point}, "LightSkyBlue");
      } else if (planner.isPathTruncated()) {
        overlays.drawLine(name_prefix + "afterpath", {path.back(), target_point}, "LightPink");
      }
    }
  }
}

std::pair<size_t, ateam_geometry::Point> MotionExecutor::ProjectRobotOnPath(
  const path_planning::Path & path, const Robot & robot)
{
  if (path.empty()) {
    return {0, robot.pos};
  }
  if (path.size() == 1) {
    return {0, path[0]};
  }
  auto closest_point = ateam_geometry::nearestPointOnSegment(ateam_geometry::Segment(path[0],
      path[1]), robot.pos);
  size_t closest_index = 1;
  double min_distance = CGAL::squared_distance(closest_point, robot.pos);
  for (size_t i = 1; i < path.size() - 1; ++i) {
    const auto segment = ateam_geometry::Segment(path[i], path[i + 1]);
    const auto point_on_segment = ateam_geometry::nearestPointOnSegment(segment, robot.pos);
    const auto distance = CGAL::squared_distance(point_on_segment, robot.pos);
    if (distance <= min_distance) {
      min_distance = distance;
      closest_point = point_on_segment;
      closest_index = i + 1;
    }
  }
  return {closest_index, closest_point};
}

} // namespace ateam_kenobi::motion
