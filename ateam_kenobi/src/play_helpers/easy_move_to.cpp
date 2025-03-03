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
#include <ateam_common/robot_constants.hpp>
#include "path_planning/escape_velocity.hpp"
#include "path_planning/obstacles.hpp"

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

void EasyMoveTo::setTargetPosition(ateam_geometry::Point target_position)
{
  target_position_ = target_position;
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
  motion_controller_.set_trajectory(path);
  return motion_controller_.get_command(robot, current_time, motion_options_);
}

void EasyMoveTo::drawTrajectoryOverlay(
  const path_planning::PathPlanner::Path & path,
  const Robot & robot)
{
  if (path.empty()) {
    const std::vector<ateam_geometry::Point> points = {
      robot.pos,
      target_position_
    };
    getOverlays().drawLine("path", points, "red");
  } else {
    getOverlays().drawLine("path", path, "purple");
    if (CGAL::squared_distance(path.back(), target_position_) > kRobotRadius * kRobotRadius) {
      getOverlays().drawLine("afterpath", {path.back(), target_position_}, "red");
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

}  // namespace ateam_kenobi::play_helpers
