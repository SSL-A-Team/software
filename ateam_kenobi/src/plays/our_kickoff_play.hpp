// Copyright 2023 A Team
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

#ifndef PLAYS__OUR_KICKOFF_PLAY_HPP_
#define PLAYS__OUR_KICKOFF_PLAY_HPP_

#include <vector>

#include "base_play.hpp"
#include "path_planning/path_planner.hpp"
#include "motion/motion_controller.hpp"
#include "ateam_geometry/types.hpp"
#include "types/robot.hpp"

namespace ateam_kenobi::plays
{
class OurKickoffPlay : public BasePlay
{
public:
  explicit OurKickoffPlay(visualization::OverlayPublisher & overlay_publisher);

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

private:
  path_planning::PathPlanner path_planner_;
  std::array<MotionController, 16> motion_controllers_;
  std::array<std::optional<std::vector<ateam_geometry::Point>>, 16> saved_paths_;
  std::vector<Robot> available_robots_;

  std::vector<ateam_geometry::Point> positions_to_assign_;


  int prev_assigned_id_ = -1;
};
}  // namespace ateam_kenobi::plays

#endif // PLAYS__OUR_KICKOFF_PLAY_HPP_
