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


#include "play_helpers/easy_move_to.hpp"
#include "easy_move_to.hpp"
#include <chrono>

namespace ateam_kenobi::play_helpers
{

void EasyMoveTo::CreateArray(
  std::array<EasyMoveTo, 16> & dst,
  visualization::Overlays overlays)
{
  std::generate(
    dst.begin(), dst.end(), [&overlays, ind = 0]() mutable {
      return EasyMoveTo(overlays.getChild(std::to_string(ind++)));
    });
}

EasyMoveTo::EasyMoveTo(visualization::Overlays overlays)
: path_planner_(overlays.getChild("path_planner")),
  overlays_(overlays)
{
}


EasyMoveTo & EasyMoveTo::operator=(EasyMoveTo && other)
{
  instance_name_ = other.instance_name_;
  target_position_ = other.target_position_;
  planner_options_ = other.planner_options_;
  path_planner_ = other.path_planner_;
  motion_controller_ = other.motion_controller_;
  overlays_ = other.overlays_;
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
    std::cerr << "UNREASONABLY LARGE VELOCITY GIVEN TO SET MAX VELOCITY\n";
    return;
  }
  motion_controller_.v_max = velocity;
}

void EasyMoveTo::setMaxAngularVelocity(double velocity)
{
  if (velocity > 6.5) {
    std::cerr << "UNREASONABLY LARGE VELOCITY GIVEN TO SET MAX ANGULAR VELOCITY\n";
    return;
  }
  motion_controller_.t_max = velocity;
}

ateam_msgs::msg::RobotMotionCommand EasyMoveTo::runFrame(
  const Robot & robot, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles)
{
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
    overlays_.drawLine(instance_name_ + "_path", points, "red");
  } else {
    overlays_.drawLine(instance_name_ + "_path", path, "purple");
  }
}

}  // namespace ateam_kenobi::play_helpers
