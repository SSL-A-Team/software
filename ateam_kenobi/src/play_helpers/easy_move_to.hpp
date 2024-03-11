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


#ifndef PLAY_HELPERS__EASY_MOVE_TO_HPP_
#define PLAY_HELPERS__EASY_MOVE_TO_HPP_

#include <vector>
#include <string>
#include <ateam_geometry/types.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "path_planning/path_planner.hpp"
#include "motion/motion_controller.hpp"
#include "visualization/overlays.hpp"
#include "types/robot.hpp"
#include "types/world.hpp"

namespace ateam_kenobi::play_helpers
{

class EasyMoveTo
{
public:
  static void CreateArray(
    std::array<EasyMoveTo, 16> & dst,
    visualization::Overlays overlays);

  /**
   * @brief DO NOT USE THIS CONSTRUCTOR IN PLAY CODE
   * This is meant for internal use only.
   */
  EasyMoveTo() {}

  explicit EasyMoveTo(visualization::Overlays overlays);

  EasyMoveTo & operator=(EasyMoveTo && other);

  void reset();

  void setTargetPosition(ateam_geometry::Point target_position);

  void setPlannerOptions(path_planning::PlannerOptions options);

  void face_point(std::optional<ateam_geometry::Point> point);
  void face_absolute(double angle);
  void face_travel();
  void no_face();

  void setMaxVelocity(double velocity);

  void setMaxAngularVelocity(double velocity);

  ateam_msgs::msg::RobotMotionCommand runFrame(
    const Robot & robot, const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles = {});

  ateam_geometry::Point getTargetPosition() const
  {
    return target_position_;
  }

private:
  std::string instance_name_;
  ateam_geometry::Point target_position_;
  path_planning::PlannerOptions planner_options_;
  path_planning::PathPlanner path_planner_;
  MotionController motion_controller_;
  MotionOptions motion_options_;
  visualization::Overlays overlays_;

  path_planning::PathPlanner::Path planPath(
    const Robot & robot, const World & world,
    const std::vector<ateam_geometry::AnyShape> & obstacles);

  ateam_msgs::msg::RobotMotionCommand getMotionCommand(
    const path_planning::PathPlanner::Path & path, const Robot & robot, const World & world);

  void drawTrajectoryOverlay(const path_planning::PathPlanner::Path & path, const Robot & robot);
};

}  // namespace ateam_kenobi::play_helpers

#endif  // PLAY_HELPERS__EASY_MOVE_TO_HPP_
