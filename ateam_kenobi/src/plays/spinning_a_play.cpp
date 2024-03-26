#include "spinning_a_play.hpp"
#include <ateam_common/robot_constants.hpp>
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

SpinningAPlay::SpinningAPlay()
: BasePlay("SpinningAPlay")
{
  base_shape_ = {
    ateam_geometry::Point(0, 0),
    ateam_geometry::Point(-0.44, -0.44),
    ateam_geometry::Point(-0.22, 0),
    ateam_geometry::Point(0, 0.44),
    ateam_geometry::Point(0.22, 0),
    ateam_geometry::Point(0.44, -0.44),
  };
}

void SpinningAPlay::reset()
{
  angle_ = 0.0;
  for (auto & emt : easy_move_tos_) {
    path_planning::PlannerOptions options;
    options.footprint_inflation = 0.1;
    emt.setPlannerOptions(options);
    emt.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> SpinningAPlay::runFrame(const World & world)
{

  const auto available_robots = play_helpers::getAvailableRobots(world);

  const auto num_robots = std::min(available_robots.size(), base_shape_.size());

  CGAL::Aff_transformation_2<ateam_geometry::Kernel> rotate_transform(CGAL::ROTATION,
    std::sin(angle_), std::cos(angle_));

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  for (auto ind = 0ul; ind < num_robots; ++ind) {
    const auto & robot = available_robots[ind];
    const auto point = base_shape_[ind].transform(rotate_transform);
    auto & emt = easy_move_tos_[robot.id];
    emt.setTargetPosition(point);
    emt.face_absolute(-M_PI_2 + angle_);
    motion_commands[robot.id] = emt.runFrame(robot, world);
    overlays_.drawCircle(
      "dest" + std::to_string(ind),
      ateam_geometry::makeCircle(point, kRobotRadius));
  }

  if (angle_ < kNumRotations * 2 * M_PI) {
    angle_ += kAngleSpeed;
  }

  return motion_commands;
}

} // namespace ateam_kenobi::plays
