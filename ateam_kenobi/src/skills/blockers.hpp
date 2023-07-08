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


#ifndef SKILLS__BLOCKERS_HPP_
#define SKILLS__BLOCKERS_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "visualization/overlay_publisher.hpp"
#include "types/world.hpp"
#include "play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::skills
{

class Blockers
{
public:
  Blockers(visualization::OverlayPublisher & overlay_publisher);

  void reset();

  std::vector<ateam_geometry::Point> getAssignmentPoints(const World & world);

  std::vector<ateam_msgs::msg::RobotMotionCommand> runFrame(
    const World & world,
    const std::vector<Robot> & robots);

private:
  visualization::OverlayPublisher & overlay_publisher_;
  std::array<play_helpers::EasyMoveTo, 16> easy_move_tos_;

  std::vector<Robot> getRankedBlockableRobots(const World & world);

  ateam_geometry::Point getBlockingPosition(const World & world, const Robot & blockee);

};

}

#endif  // SKILLS__BLOCKERS_HPP_
