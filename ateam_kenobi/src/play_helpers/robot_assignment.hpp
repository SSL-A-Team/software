
#ifndef PLAY_HELPERS__ROBOT_ASSIGNMENT_
#define PLAY_HELPERS__ROBOT_ASSIGNMENT_

#include <vector>
#include "types/robot.hpp"

namespace ateam_kenobi::play_helpers
{

std::vector<std::optional<Robot>> assignRobots(
  const std::vector<Robot> & available_robots,
  const std::vector<ateam_geometry::Point> & positions,
  const std::vector<std::vector<int>> & disallowed_robot_ids = {});

}  // namespace ateam_kenobi::play_helpers

#endif  // PLAY_HELPERS__ROBOT_ASSIGNMENT_
