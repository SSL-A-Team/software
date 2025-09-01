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


#include "easy_move_to.hpp"
#include <ranges>
#include <algorithm>
#include <chrono>
#include <utility>
#include <ateam_common/robot_constants.hpp>
#include "core/path_planning/escape_velocity.hpp"
#include "core/path_planning/obstacles.hpp"

namespace ateam_kenobi::play_helpers
{

EasyMoveTo::EasyMoveTo(stp::Options stp_options)
: stp::Base(stp_options),
  path_planner_(createChild<path_planning::PathPlanner>("path_planner"))
{
}

EasyMoveTo::EasyMoveTo(EasyMoveTo && other)
: stp::Base(other)
{
  target_position_ = other.target_position_;
  planner_options_ = other.planner_options_;
  path_planner_ = other.path_planner_;
  motion_controller_ = other.motion_controller_;
}

EasyMoveTo::EasyMoveTo(const EasyMoveTo & other)
: stp::Base(other)
{
  target_position_ = other.target_position_;
  planner_options_ = other.planner_options_;
  path_planner_ = other.path_planner_;
  motion_controller_ = other.motion_controller_;
}

EasyMoveTo & EasyMoveTo::operator=(EasyMoveTo && other)
{
  stp::Base::operator=(other);
  target_position_ = other.target_position_;
  planner_options_ = other.planner_options_;
  path_planner_ = other.path_planner_;
  motion_controller_ = other.motion_controller_;
  return *this;
}

EasyMoveTo & EasyMoveTo::operator=(const EasyMoveTo & other)
{
  stp::Base::operator=(other);
  target_position_ = other.target_position_;
  planner_options_ = other.planner_options_;
  path_planner_ = other.path_planner_;
  motion_controller_ = other.motion_controller_;
  return *this;
}

void EasyMoveTo::reset()
{
  target_position_ = ateam_geometry::Point();
  motion_controller_.reset();
}

void EasyMoveTo::setTargetPosition(
  ateam_geometry::Point target_position,
  ateam_geometry::Vector target_velocity)
{
  target_position_ = target_position;
  target_velocity_ = target_velocity;
}

const path_planning::PlannerOptions & EasyMoveTo::getPlannerOptions() const
{
  return planner_options_;
}

void EasyMoveTo::setPlannerOptions(path_planning::PlannerOptions options)
{
  planner_options_ = options;
}

void EasyMoveTo::setMotionOptions(MotionOptions options)
{
  motion_options_ = options;
}


void EasyMoveTo::face_point(std::optional<ateam_geometry::Point> point)
{
  motion_controller_.face_point(point);
}
void EasyMoveTo::face_absolute(double angle)
{
  motion_controller_.face_absolute(angle);
}
void EasyMoveTo::face_travel()
{
  motion_controller_.face_travel();
}
void EasyMoveTo::no_face()
{
  motion_controller_.no_face();
}

void EasyMoveTo::setMaxVelocity(double velocity)
{
  if (velocity > 3.0) {
    RCLCPP_WARN(getLogger(), "UNREASONABLY LARGE VELOCITY GIVEN TO SET MAX VELOCITY");
    return;
  }
  motion_controller_.v_max = velocity;
}

void EasyMoveTo::setMaxAngularVelocity(double velocity)
{
  if (velocity > 6.5) {
    RCLCPP_WARN(getLogger(), "UNREASONABLY LARGE VELOCITY GIVEN TO SET MAX ANGULAR VELOCITY");
    return;
  }
  motion_controller_.t_max = velocity;
}

void EasyMoveTo::setMaxAccel(double accel)
{
  if (accel > 8.0) {
    RCLCPP_WARN(getLogger(), "UNREASONABLY LARGE ACCELERATION GIVEN TO SET MAX ACCELERATION");
    return;
  }
  motion_controller_.accel_limit = accel;
}

void EasyMoveTo::setMaxDecel(double decel)
{
  if (decel > 8.0) {
    RCLCPP_WARN(getLogger(), "UNREASONABLY LARGE DECELERATION GIVEN TO SET MAX DECELERATION");
    return;
  }
  motion_controller_.decel_limit = decel;
}

void EasyMoveTo::setMaxThetaAccel(double accel)
{
  if (accel > 8.0) {
    RCLCPP_WARN(getLogger(), "UNREASONABLY LARGE ACCELERATION GIVEN TO SET MAX THETA ACCELERATION");
    return;
  }
  motion_controller_.t_accel_limit = accel;
}

void EasyMoveTo::setMaxAllowedTurnAngle(double angle)
{
  motion_controller_.max_allowed_turn_angle = angle;
}

ateam_msgs::msg::RobotMotionCommand EasyMoveTo::runFrame(
  const Robot & robot, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles)
{
  const auto escape_velocity = generateEscapeVelocity(world, robot, obstacles);
  if (escape_velocity) {
    return *escape_velocity;
  }
  const auto path = planPath(robot, world, obstacles);
  const auto motion_command = getMotionCommand(path, robot, world);
  drawTrajectoryOverlay(path, robot);
  return motion_command;
}

path_planning::PathPlanner::Path EasyMoveTo::planPath(
  const Robot & robot, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles)
{
  return path_planner_.getPath(robot.pos, target_position_, world, obstacles, planner_options_);
}

ateam_msgs::msg::RobotMotionCommand EasyMoveTo::getMotionCommand(
  const path_planning::PathPlanner::Path & path, const Robot & robot, const World & world)
{
  const auto current_time = std::chrono::duration_cast<std::chrono::duration<double>>(
    world.current_time.time_since_epoch()).count();

  // Robot should stop at the end of the path
  // if the plan doesn't reach the target point due to an obstacle
  auto velocity = ateam_geometry::Vector(0, 0);
  if (!path.empty() && (CGAL::squared_distance(path.back(),
      target_position_) < kRobotRadius * kRobotRadius))
  {
    velocity = target_velocity_;
  }

  const bool used_cached_path = path_planner_.usedCachedPath();
  if (used_cached_path) {
    motion_controller_.update_trajectory(path, velocity);
  } else {
    motion_controller_.reset_trajectory(path, velocity);
  }
  return motion_controller_.get_command(robot, current_time, motion_options_);
}

void EasyMoveTo::drawTrajectoryOverlay(
  const path_planning::PathPlanner::Path & path,
  const Robot & robot)
{
  auto & overlays = getOverlays();
  if(path.empty()) {
    overlays.drawLine("path", {robot.pos, target_position_}, "Red");
  } else if(path.size() == 1) {
    overlays.drawLine("path", {robot.pos, target_position_}, "Purple");
  } else {
    const auto [closest_index, closest_point] = ProjectRobotOnPath(path, robot);
    std::vector<ateam_geometry::Point> path_done(path.begin(), path.begin() + (closest_index));
    path_done.push_back(closest_point);
    std::vector<ateam_geometry::Point> path_remaining(path.begin() + (closest_index),
      path.end());
    path_remaining.insert(path_remaining.begin(), closest_point);
    const auto translucent_purple = "#8000805F";
    overlays.drawLine("path_done", path_done, translucent_purple);
    overlays.drawLine("path_remaining", path_remaining, "Purple");
    if (path_planner_.didTimeOut()) {
      overlays.drawLine("afterpath", {path.back(), target_position_}, "LightSkyBlue");
    } else if (path_planner_.isPathTruncated()) {
      overlays.drawLine("afterpath", {path.back(), target_position_}, "LightPink");
    }
  }
}

std::optional<ateam_msgs::msg::RobotMotionCommand> EasyMoveTo::generateEscapeVelocity(
  const World & world,
  const Robot & robot,
  std::vector<ateam_geometry::AnyShape> obstacles)
{
  if (planner_options_.use_default_obstacles) {
    path_planning::AddDefaultObstacles(world, obstacles);
  }
  path_planning::AddRobotObstacles(world.our_robots, robot.id, obstacles);
  path_planning::AddRobotObstacles(world.their_robots, obstacles);

  const auto twist = path_planning::GenerateEscapeVelocity(robot, obstacles,
      planner_options_.footprint_inflation);

  if(!twist) {
    return std::nullopt;
  }

  ateam_msgs::msg::RobotMotionCommand command;
  command.twist = *twist;
  return command;
}

std::pair<size_t, ateam_geometry::Point> EasyMoveTo::ProjectRobotOnPath(
  const path_planning::PathPlanner::Path & path, const Robot & robot)
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

}  // namespace ateam_kenobi::play_helpers
