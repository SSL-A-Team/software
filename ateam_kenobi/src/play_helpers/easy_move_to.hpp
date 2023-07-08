#ifndef PLAY_HELPERS__EASY_MOVE_TO_HPP_
#define PLAY_HELPERS__EASY_MOVE_TO_HPP_

#include <ateam_geometry/types.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "path_planning/path_planner.hpp"
#include "motion/motion_controller.hpp"
#include "visualization/overlay_publisher.hpp"
#include "types/robot.hpp"
#include "types/world.hpp"

namespace ateam_kenobi::play_helpers
{

class EasyMoveTo
{
public:
  static void CreateArray(std::array<EasyMoveTo, 16> & dst, visualization::OverlayPublisher & overlay_publisher);

  /**
   * @brief DO NOT USE THIS CONSTRUCTOR IN PLAY CODE
   * This is meant for internal use only.
   */
  EasyMoveTo() {};

  EasyMoveTo(visualization::OverlayPublisher & overlay_publisher);

  EasyMoveTo& operator=(EasyMoveTo&& other);

  void reset();
  
  void setTargetPosition(ateam_geometry::Point target_position);

  void setPlannerOptions(path_planning::PlannerOptions options);

  void face_point(std::optional<ateam_geometry::Point> point);
  void face_absolute(double angle);
  void face_travel();
  void no_face();

  void setMaxVelocity(double velocity);

  void setMaxAngularVelocity(double velocity);

  ateam_msgs::msg::RobotMotionCommand runFrame(const Robot & robot, const World & world, const std::vector<ateam_geometry::AnyShape> & obstacles = {});

  ateam_geometry::Point getTargetPosition() const {
    return target_position_;
  }

private:

  static std::size_t instance_index_;  // used for naming visualizations
  std::string instance_name_;
  ateam_geometry::Point target_position_;
  path_planning::PlannerOptions planner_options_;
  path_planning::PathPlanner path_planner_;
  MotionController motion_controller_;
  MotionOptions motion_options_;
  visualization::OverlayPublisher * overlay_publisher_;

  path_planning::PathPlanner::Path planPath(const Robot & robot, const World & world, const std::vector<ateam_geometry::AnyShape> & obstacles);

  ateam_msgs::msg::RobotMotionCommand getMotionCommand(const path_planning::PathPlanner::Path & path, const Robot & robot, const World & world);

  void drawTrajectoryOverlay(const path_planning::PathPlanner::Path & path, const Robot & robot);
};

}  // namespace ateam_kenobi::play_helpers

#endif  // PLAY_HELPERS__EASY_MOVE_TO_HPP_
