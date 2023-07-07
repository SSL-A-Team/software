#include "play_helpers/easy_move_to.hpp"
#include "easy_move_to.hpp"
#include <chrono>

namespace ateam_kenobi::play_helpers
{

std::size_t EasyMoveTo::instance_index_ = 0;

void EasyMoveTo::CreateArray(std::array<EasyMoveTo,16> & dst,
  visualization::OverlayPublisher & overlay_publisher)
{
  std::generate(
    dst.begin(), dst.end(), [&overlay_publisher]() {
      return EasyMoveTo(overlay_publisher);
    });
}

EasyMoveTo::EasyMoveTo(visualization::OverlayPublisher & overlay_publisher)
: instance_name_("EasyMoveToViz" + std::to_string(instance_index_)),
  overlay_publisher_(&overlay_publisher)
{
  std::cerr << "EasyMoveTo instance " << instance_index_ << '\n';
  instance_index_++;
}


EasyMoveTo & EasyMoveTo::operator=(EasyMoveTo && other)
{
  instance_name_ = other.instance_name_;
  target_position_ = other.target_position_;
  planner_options_ = other.planner_options_;
  path_planner_ = other.path_planner_;
  motion_controller_ = other.motion_controller_;
  overlay_publisher_ = other.overlay_publisher_;
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

void EasyMoveTo::setFacingTowards(std::optional<ateam_geometry::Point> target)
{
  motion_controller_.face_towards = target;
}

void EasyMoveTo::setMaxVelocity(double velocity) {
  motion_controller_.v_max = velocity;
}

void EasyMoveTo::setMaxAngularVelocity(double velocity) {
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
  return motion_controller_.get_command(robot, current_time);
}

void EasyMoveTo::drawTrajectoryOverlay(
  const path_planning::PathPlanner::Path & path,
  const Robot & robot)
{
  if (!overlay_publisher_) {
    // Thid shouldn't be hit in normal code, but just in case, fail gracefully.
    return;
  }
  if (path.empty()) {
    const std::vector<ateam_geometry::Point> points = {
      robot.pos,
      target_position_
    };
    overlay_publisher_->drawLine(instance_name_ + "_path", points, "red");
  } else {
    overlay_publisher_->drawLine(instance_name_ + "_path", path, "purple");
  }
}

} // namespace ateam_kenobi::play_helpers
