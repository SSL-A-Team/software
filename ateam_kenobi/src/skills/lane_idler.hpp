// Copyright 2024 A Team
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

#ifndef SKILLS__LANE_IDLER_HPP_
#define SKILLS__LANE_IDLER_HPP_

#include <vector>
#include <ateam_geometry/any_shape.hpp>
#include "core/stp/skill.hpp"
#include "core/play_helpers/easy_move_to.hpp"
#include "core/play_helpers/lanes.hpp"

namespace ateam_kenobi::skills
{

class LaneIdler : public stp::Skill
{
public:
  explicit LaneIdler(stp::Options stp_options);

  void Reset();

  ateam_geometry::Point GetAssignmentPoint(const World & world);

  ateam_msgs::msg::RobotMotionCommand RunFrame(const World & world, const Robot & robot);

  void SetLane(play_helpers::lanes::Lane lane)
  {
    lane_ = lane;
  }

  void SetExtraObstacles(std::vector<ateam_geometry::AnyShape> obstacles)
  {
    extra_obstacles_ = obstacles;
  }

private:
  play_helpers::lanes::Lane lane_ = play_helpers::lanes::Lane::Center;
  play_helpers::EasyMoveTo easy_move_to_;
  std::vector<ateam_geometry::AnyShape> extra_obstacles_;

  ateam_geometry::Point GetIdlingPosition(const World & world);
};

}  // namespace ateam_kenobi::skills


#endif  // SKILLS__LANE_IDLER_HPP_
