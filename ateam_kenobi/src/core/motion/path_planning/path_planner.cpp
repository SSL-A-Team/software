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


#include "path_planner.hpp"
#include <algorithm>
#include <memory>
#include <ateam_path_planning/planner.hpp>
#include "obstacles.hpp"

namespace ateam_kenobi::motion::path_planning
{

PathPlanner::PathPlanner()
: planner_(std::make_unique<ateam_path_planning::Planner>())
{
}

PathPlanner::~PathPlanner() = default;

void PathPlanner::Execute(
  std::array<std::optional<MotionCommand>, 16> & commands,
  const std::vector<PathPlanningTarget> & targets, const World & world,
  visualization::Overlays & overlays)
{
  std::array<std::optional<ateam_path_planning::Pose>, 16> target_poses;
  std::array<std::vector<ateam_path_planning::Obstacle>, 16> per_bot_obstacles;
  std::array<ateam_path_planning::PlannerOptions, 16> options;

  UnpackTargets(targets, world, target_poses, per_bot_obstacles, options, overlays);

  const auto expected_positions = planner_->GetExpectedLocations(options);
  for(auto id = 0; id < 16; ++id) {
    const auto overlay_name = "pathing/expected/" + std::to_string(id);
    if(!target_poses[id]) {
      overlays.clearItem(overlay_name);
      continue;
    }
    const auto & position = expected_positions[id];
    if(!position) {
      overlays.clearItem(overlay_name);
      continue;
    }
    const auto & bot_options = options[id];
    overlays.drawCircle(overlay_name,
        ateam_geometry::makeCircle(*position, bot_options.replan_thresholds.deviation_distance),
        "Purple", "#00000000");
  }

  std::vector<ateam_path_planning::Obstacle> global_obstacles;
  for(const auto & robot : world.their_robots) {
    if(robot.visible) {
      global_obstacles.push_back(ateam_path_planning::Obstacle{
          .shape = ateam_geometry::makeDisk(robot.pos, kRobotRadius),
          .expected_motion = robot.vel
      });
    }
  }

  const auto paths = planner_->PlanPathsForAllBots(target_poses, {}, world, global_obstacles,
      per_bot_obstacles, options);

  FillMotionCommands(paths, world, targets, commands, overlays);
}

void PathPlanner::UnpackTargets(
  const std::vector<PathPlanningTarget> & targets, const World & world,
  std::array<std::optional<ateam_path_planning::Pose>, 16> & target_poses,
  std::array<std::vector<ateam_path_planning::Obstacle>, 16> & per_bot_obstacles,
  std::array<ateam_path_planning::PlannerOptions, 16> & options, visualization::Overlays & overlays)
{
  ateam_path_planning::Obstacle ball_obstacle{
    .shape = ateam_geometry::makeDisk(world.ball.pos, kBallRadius),
    .expected_motion = world.ball.vel
  };

  std::vector<ateam_path_planning::Obstacle> default_obstacles;
  const auto kenobi_default_obstacles = GetDefaultObstacles(world);
  std::transform(kenobi_default_obstacles.begin(), kenobi_default_obstacles.end(),
      std::back_inserter(default_obstacles), [](const auto & o){
      return ateam_path_planning::Obstacle{
      .shape = o,
      .expected_motion = {}
      };
  });

  for(const auto & target : targets) {
    target_poses[target.robot_id] = ateam_path_planning::Pose{
      target.position,
      target.heading
    };

    std::transform(target.obstacles.begin(), target.obstacles.end(),
        std::back_inserter(per_bot_obstacles[target.robot_id]), [](const auto & o){
        return ateam_path_planning::Obstacle {
        .shape = o,
        .expected_motion = {}
        };
    });

    if(target.planner_options.avoid_ball) {
      per_bot_obstacles[target.robot_id].push_back(ball_obstacle);
    }

    if(target.planner_options.use_default_obstacles) {
      std::copy(default_obstacles.begin(), default_obstacles.end(),
          std::back_inserter(per_bot_obstacles[target.robot_id]));
    }

    if(target.planner_options.draw_obstacles) {
      DrawObstacles(overlays, per_bot_obstacles[target.robot_id]);
    }

    options[target.robot_id] = ateam_path_planning::PlannerOptions{
      .collision_check_resolution = target.planner_options.collision_check_resolution,
      .collision_check_horizon = target.planner_options.collision_check_horizon,
      .footprint_inflation = target.planner_options.footprint_inflation,
      .boundary_footprint_inflation = target.planner_options.boundary_footprint_inflation,
      .ignore_start_obstacles = target.planner_options.ignore_start_obstacles,
      .limits = {
        .linear_velocity = target.limits.linear_velocity,
        .linear_acceleration = target.limits.linear_acceleration,
        .angular_velocity = target.limits.angular_velocity,
        .angular_acceleration = target.limits.angular_acceleration
      },
      .replan_thresholds = target.planner_options.replan_thresholds
    };
  }
}

void PathPlanner::FillMotionCommands(
  const std::array<std::optional<ateam_path_planning::PathPlanResult>, 16> & results,
  const World & world, const std::vector<PathPlanningTarget> & targets,
  std::array<std::optional<MotionCommand>, 16> & commands, visualization::Overlays & overlays)
{
  for(auto i = 0ul; i < results.size(); ++i) {
    const auto & result = results[i];
    const auto target_iter = std::find_if(targets.begin(), targets.end(),
        [i](const auto & t) {return t.robot_id == static_cast<int>(i);});
    if(target_iter == targets.end()) {
      overlays.clearItem("pathingTarget/" + std::to_string(i));
      overlays.clearItem("collisionStop/" + std::to_string(i));
      continue;
    }
    const auto & target = *target_iter;
    DrawTrajectory(overlays, result, world.our_robots[i], target.position, target.planner_options);
    if(!result.has_value()) {
      overlays.clearItem("pathingTarget/" + std::to_string(i));
      overlays.clearItem("collisionStop/" + std::to_string(i));
      continue;
    }
    if(result->collision_stats.new_collision_start_time.has_value()) {
      const auto collision_time = *(result->collision_stats.new_collision_start_time);
      const auto stopping_time = ateam_geometry::norm(world.our_robots[i].vel) /
        target.limits.linear_acceleration;
      if(stopping_time < collision_time) {
        MotionCommand command;
        command.control_mode = ControlMode::LocalVelocity;
        command.velocity.x = 0.0;
        command.velocity.y = 0.0;
        command.velocity.theta = 0.0;
        command.limit_acc_angular = target.limits.angular_acceleration;
        command.limit_vel_angular = target.limits.angular_velocity;
        command.limit_acc_linear = target.limits.linear_acceleration;
        command.limit_vel_linear = target.limits.linear_velocity;
        commands[i] = command;
        overlays.drawStopsign("collisionStop/" + std::to_string(i), world.our_robots[i], "Orange");
      } else {
        MotionCommand command;
        command.control_mode = ControlMode::EStopBrake;
        commands[i] = command;
        overlays.drawStopsign("collisionStop/" + std::to_string(i), world.our_robots[i], "Pink");
      }
      overlays.clearItem("pathingTarget/" + std::to_string(i));
      continue;
    } else {
      overlays.clearItem("collisionStop/" + std::to_string(i));
    }

    const auto & path = result->path;
    const auto current_target_pose = path.GetTargetAtNow();
    if(!current_target_pose.has_value()) {
      overlays.clearItem("pathingTarget/" + std::to_string(i));
      continue;
    }
    overlays.drawCircle("pathingTarget/" + std::to_string(i),
        ateam_geometry::makeCircle(current_target_pose->position, 0.05), "#00000000", "DarkGreen");
    MotionCommand command;
    command.control_mode = ControlMode::GlobalPosition;
    command.pose.x = current_target_pose->position.x();
    command.pose.y = current_target_pose->position.y();
    command.pose.theta = current_target_pose->heading;
    command.limit_acc_angular = target.limits.angular_acceleration;
    command.limit_vel_angular = target.limits.angular_velocity;
    command.limit_acc_linear = target.limits.linear_acceleration;
    command.limit_vel_linear = target.limits.linear_velocity;
    commands[i] = command;
  }
}

void PathPlanner::DrawObstacles(
  visualization::Overlays & overlays,
  const std::vector<ateam_path_planning::Obstacle> & obstacles)
{
  auto drawObstacle = [&overlays, obstacle_ind = 0](const auto & shape) mutable {
      const auto name = "obstacle" + std::to_string(obstacle_ind);
      const auto color = "FF00007F";
      using ShapeT = std::decay_t<decltype(shape)>;
      if constexpr (std::is_same_v<ShapeT, ateam_geometry::Point>) {
        overlays.drawCircle(name, ateam_geometry::makeCircle(shape, 2.5), color, color);
      } else if constexpr (std::is_same_v<ShapeT, ateam_geometry::Segment>) {
        overlays.drawLine(name, {shape.source(), shape.target()}, color);
      } else if constexpr (std::is_same_v<ShapeT, ateam_geometry::Ray>) {
        overlays.drawLine(name, {shape.source(), shape.point(10)}, color);
      } else if constexpr (std::is_same_v<ShapeT, ateam_geometry::Rectangle>) {
        overlays.drawRectangle(name, shape, color, color);
      } else if constexpr (std::is_same_v<ShapeT, ateam_geometry::Circle>) {
        overlays.drawCircle(name, shape, color, color);
      } else if constexpr (std::is_same_v<ShapeT, ateam_geometry::Disk>) {
        overlays.drawCircle(name, shape.asCircle(), color, color);
      } else {
        // RCLCPP_WARN(
        //   getLogger(), "Unsupported shape type in drawObstacles: %s ",
        //   typeid(shape).name());
      }
      obstacle_ind++;
    };

  std::ranges::for_each(obstacles, [&drawObstacle](const auto & o) {
      std::visit(drawObstacle, o.shape);
    });
}

void PathPlanner::DrawTrajectory(
  visualization::Overlays & overlays,
  const std::optional<ateam_path_planning::PathPlanResult> & result, const Robot & robot,
  const ateam_geometry::Point & target, const PlannerOptions & options)
{
  constexpr double kTimeStep = 0.2;
  const auto name_prefix = "pathing/robot_" + std::to_string(robot.id) + "/";
  if(!result.has_value()) {
    overlays.drawLine(name_prefix + "nopath", {robot.pos, target}, "Red");
    return;
  } else {
    overlays.clearItem(name_prefix + "nopath");
  }
  const auto & path = result->path;
  const auto points = path.ToPoints(kTimeStep);
  auto points_iter = points.begin();

  if(path.GetSegmentCount() > 1) {
    const auto transition_point = path.GetFirstTransitionPoint();
    overlays.drawCircle(name_prefix + "transition",
        ateam_geometry::makeCircle(transition_point, 0.05), "#00000000", "Purple");
  } else {
    overlays.clearItem(name_prefix + "transition");
  }

  const auto start_time = path.GetStartTime();
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now -
      start_time).count();
  const auto elapsed_point_count = std::min(static_cast<ptrdiff_t>(elapsed / kTimeStep),
      static_cast<ptrdiff_t>(points.size()));
  if(elapsed_point_count > 0) {
    const auto translucent_purple = "#8000805F";
    overlays.drawLine(name_prefix + "done",
        std::vector<ateam_geometry::Point>(points_iter, points_iter + elapsed_point_count),
        translucent_purple);
    points_iter += elapsed_point_count - 1;
  } else {
    overlays.clearItem(name_prefix + "done");
  }

  const auto & collision_stats = result->collision_stats;
  const auto checked_point_count = std::min(static_cast<ptrdiff_t>(options.collision_check_horizon /
      kTimeStep), std::distance(points_iter, points.end()));
  if(collision_stats.new_collision_start_time.has_value() &&
    collision_stats.new_collision_start_time.value() > elapsed + kTimeStep)
  {
    const auto collision_time = collision_stats.new_collision_start_time.value() - elapsed;
    const auto collision_point_count = static_cast<ptrdiff_t>(collision_time / kTimeStep);
    overlays.drawLine(name_prefix + "colliding",
        std::vector<ateam_geometry::Point>(points_iter, points_iter + collision_point_count),
        "Yellow");
    points_iter += collision_point_count - 1;

    const auto remaining_checked_point_count = checked_point_count - collision_point_count;
    if(remaining_checked_point_count > 0) {
      overlays.drawLine(name_prefix + "checked",
        std::vector<ateam_geometry::Point>(points_iter,
        points_iter + remaining_checked_point_count), "Purple");
      points_iter += remaining_checked_point_count - 1;
    }
  } else {
    overlays.clearItem(name_prefix + "colliding");
    overlays.drawLine(name_prefix + "checked",
        std::vector<ateam_geometry::Point>(points_iter, points_iter + checked_point_count),
        "Purple");
    points_iter += checked_point_count - 1;
  }

  if(points_iter != points.end()) {
    overlays.drawLine(name_prefix + "unchecked",
        std::vector<ateam_geometry::Point>(points_iter, points.end()), "LightSkyBlue");
  } else {
    overlays.clearItem(name_prefix + "unchecked");
  }

  if(CGAL::squared_distance(points.back(), target) > 1e-2) {
    overlays.drawLine(name_prefix + "truncated", {points.back(), target}, "LightPink");
  } else {
    overlays.clearItem(name_prefix + "truncated");
  }
}

}  // namespace ateam_kenobi::motion::path_planning
