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
      .limits = {
        .linear_velocity = target.limits.linear_velocity,
        .linear_acceleration = target.limits.linear_acceleration,
        .angular_velocity = target.limits.angular_velocity,
        .angular_acceleration = target.limits.angular_acceleration
      },
      .replan_thresholds = target.planner_options.replan_thresholds
    };
  }

  std::vector<ateam_path_planning::Obstacle> global_obstacles;
  for(const auto & robot : world.their_robots) {
    global_obstacles.push_back(ateam_path_planning::Obstacle{
        .shape = ateam_geometry::makeDisk(robot.pos, kRobotRadius),
        .expected_motion = robot.vel
    });
  }

  const auto paths = planner_->PlanPathsForAllBots(target_poses, {}, world, global_obstacles,
      per_bot_obstacles, options);

  for(auto i = 0ul; i < paths.size(); ++i) {
    const auto & path = paths[i];
    DrawTrajectory(overlays, path, world.our_robots[i], targets[i].position,
        targets[i].planner_options);
    if(!path.has_value()) {
      continue;
    }
    const auto target = path->GetTargetAtNow();
    if(!target.has_value()) {
      continue;
    }
    MotionCommand command;
    command.control_mode = ControlMode::GlobalPosition;
    command.pose.x = target->position.x();
    command.pose.y = target->position.y();
    command.pose.theta = target->heading;
    command.limit_acc_angular = options[i].limits.angular_acceleration;
    command.limit_vel_angular = options[i].limits.angular_velocity;
    command.limit_acc_linear = options[i].limits.linear_acceleration;
    command.limit_vel_linear = options[i].limits.linear_velocity;
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
  const std::optional<ateam_path_planning::TrajectorySpline> & maybe_path, const Robot & robot,
  const ateam_geometry::Point & target, const PlannerOptions & options)
{
  constexpr double kTimeStep = 0.2;
  const auto name_prefix = "pathing/robot_" + std::to_string(robot.id) + "/";
  if(!maybe_path.has_value()) {
    overlays.drawLine(name_prefix + "path", {robot.pos, target}, "Ted");
    return;
  }
  const auto & path = *maybe_path;
  const auto start_time = path.GetStartTime();
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now -
      start_time).count();
  const auto elapsed_point_count = static_cast<size_t>(elapsed / kTimeStep);
  const auto collision_checked_time = elapsed + options.collision_check_horizon;
  const auto collision_checked_point_count = static_cast<size_t>(collision_checked_time /
    kTimeStep);
  const auto points = path.ToPoints(kTimeStep);
  std::vector<ateam_geometry::Point> points_done;
  std::copy_n(points.begin(), elapsed_point_count, std::back_inserter(points_done));
  const auto translucent_purple = "#8000805F";
  overlays.drawLine(name_prefix + "done", points_done, translucent_purple);

  std::vector<ateam_geometry::Point> points_checked;
  std::copy_n(points.begin() + elapsed_point_count,
      collision_checked_point_count - elapsed_point_count, std::back_inserter(points_checked));
  overlays.drawLine(name_prefix + "checked", points_checked, "Purple");

  std::vector<ateam_geometry::Point> points_unchecked;
  std::copy(points.begin() + elapsed_point_count + collision_checked_point_count, points.end(),
      std::back_inserter(points_unchecked));
  overlays.drawLine(name_prefix + "unchecked", points_unchecked, "LightSkyBlue");
}

}  // namespace ateam_kenobi::motion::path_planning
