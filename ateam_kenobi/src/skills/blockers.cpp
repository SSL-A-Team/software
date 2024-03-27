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


#include "blockers.hpp"
#include <vector>
#include "play_helpers/available_robots.hpp"
#include <ateam_geometry/ateam_geometry.hpp>
#include <ateam_common/robot_constants.hpp>

namespace ateam_kenobi::skills
{

Blockers::Blockers(visualization::Overlays overlays)
{
  play_helpers::EasyMoveTo::CreateArray(easy_move_tos_, overlays.getChild("EasyMoveTo"));
}

void Blockers::reset()
{
  for (auto & move_to : easy_move_tos_) {
    move_to.reset();
  }
}

std::vector<ateam_geometry::Point> Blockers::getAssignmentPoints(const World & world)
{
  const auto blockable_robots = getRankedBlockableRobots(world);
  std::vector<ateam_geometry::Point> positions;
  std::ranges::transform(
    blockable_robots, std::back_inserter(positions), [this, &world](const Robot & robot) {
      return getBlockingPosition(world, robot);
    });
  return positions;
}

std::vector<ateam_msgs::msg::RobotMotionCommand> Blockers::runFrame(
  const World & world,
  const std::vector<Robot> & robots, nlohmann::json * play_info)
{
  const auto blockable_robots = getRankedBlockableRobots(world);
  std::vector<ateam_geometry::Point> positions;
  std::ranges::transform(
    blockable_robots, std::back_inserter(positions), [this, &world](const Robot & blockee) {
      return getBlockingPosition(world, blockee);
    });

  // only keep as many positions as we have robots to use
  positions.erase(positions.begin() + robots.size(), positions.end());

  std::vector<ateam_msgs::msg::RobotMotionCommand> motion_commands;

  for (auto robot_index = 0ul; robot_index < robots.size(); ++robot_index) {
    const auto & robot = robots[robot_index];
    const auto & position = positions[robot_index];
    const auto robot_id = robot.id;
    auto & move_to = easy_move_tos_[robot_id];
    move_to.setTargetPosition(position);
    move_to.face_point(world.ball.pos);
    motion_commands.push_back(move_to.runFrame(robot, world));
    if (play_info) {
      (*play_info)["Blockers"][std::to_string(robot.id)]["Blocking"] =
        blockable_robots[robot_index].id;
    }
  }

  return motion_commands;
}

std::vector<Robot> Blockers::getRankedBlockableRobots(const World & world)
{
  auto visible_opponents = play_helpers::getVisibleRobots(world.their_robots);
  play_helpers::removeRobotWithId(visible_opponents, world.referee_info.their_goalie_id);
  std::ranges::sort(
    visible_opponents, [&world](const Robot & r1, const Robot & r2) {
      const auto r1_dist = ateam_geometry::norm(world.ball.pos, r1.pos);
      const auto r2_dist = ateam_geometry::norm(world.ball.pos, r2.pos);
      return r1_dist < r2_dist;
    });
  if (visible_opponents.empty()) {
    return visible_opponents;
  }
  // Pop first robot, assuming it's handling the ball
  visible_opponents.erase(visible_opponents.begin());
  return visible_opponents;
}

ateam_geometry::Point Blockers::getBlockingPosition(const World & world, const Robot & blockee)
{
  return blockee.pos +
         ((kRobotDiameter * 2.5) * ateam_geometry::normalize(world.ball.pos - blockee.pos));
}
}  // namespace ateam_kenobi::skills
